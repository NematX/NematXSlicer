"""Populate the staged resources with the official NematX profile.

Each package fetches the current v3 vendor branch into a temporary directory.
The script checks the bundle and its assets before replacing the staged profiles.
It does not change source resources or profiles installed by the user.

    main: read the path to the package resources and report failures
        stage_profiles: fetch and identify the vendor revision
            read_bundle: check the vendor, version, models and referenced assets
        stage_profiles: copy the verified files, then replace the staged directory
"""

import argparse
import configparser
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


REPOSITORY = "https://github.com/NematX/NematX-Profile.git"
BRANCH = "beta-slicer"


def read_bundle(ini: Path, assets: Path) -> tuple[str, dict[str, Path]]:
    """Return the profile version and the files to copy, keyed by package path."""
    config = configparser.ConfigParser(interpolation=None, delimiters=("=",))
    with ini.open(encoding="utf-8-sig") as source:
        config.read_file(source)
    if config.get("vendor", "id", fallback="") != "NematX":
        raise ValueError("NematX.ini must identify the NematX vendor.")
    version = config.get("vendor", "config_version", fallback="")
    if not version:
        raise ValueError("NematX.ini is missing config_version.")

    sections = config.sections()
    models = [name for name in sections if name.startswith("printer_model:")]
    if not models:
        raise ValueError("NematX.ini does not define any printer models.")
    for kind in ("printer", "print", "filament"):
        if not any(name.startswith(f"{kind}:") and not name.split(":", 1)[1].startswith("*") for name in sections):
            raise ValueError(f"NematX.ini does not define any selectable {kind} profiles.")
    if not assets.is_dir():
        raise ValueError("The NematX asset directory is missing.")

    # Compare names as text so a Windows build also checks case for Linux.
    files = {path.relative_to(assets).as_posix(): path for path in assets.rglob("*")
             if path.is_file() and ".git" not in path.relative_to(assets).parts}
    for name in models:
        model = config[name]
        if not model.get("variants", "").strip(" ;"):
            raise ValueError(f"{name} does not define any printer variants.")
        for key in ("bed_model", "bed_texture", "thumbnail"):
            filename = model.get(key, "")
            # Copy this thumbnail with the spelling used by the printer model.
            if filename == "NEX01_thumbnail.png" and filename not in files and "Nex01_thumbnail.png" in files:
                files[filename] = files.pop("Nex01_thumbnail.png")
            if filename and filename not in files:
                raise ValueError(f"{name}: missing {key} asset {filename!r}.")
    return version, files


def stage_profiles(resources: Path) -> None:
    """Fetch and validate the bundle, then replace the profiles in the package only."""
    if not resources.is_dir():
        raise ValueError(f"Package resources directory does not exist: {resources}")

    with tempfile.TemporaryDirectory(prefix="nematx-profile-") as directory:
        root = Path(directory)
        checkout = root / "source"
        print(f"Fetching {REPOSITORY} ({BRANCH})", flush=True)
        subprocess.run(
            ["git", "clone", "--depth", "1", "--single-branch", "--branch", BRANCH, REPOSITORY, str(checkout)],
            check=True, timeout=120,
        )
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=checkout, text=True, timeout=30,
        ).strip()
        ini = checkout / "profiles" / "NematX.ini"
        version, files = read_bundle(ini, ini.parent / "NematX")

        # Prepare all output before replacing profiles left by an earlier build.
        staged = root / "profiles"
        (staged / "NematX").mkdir(parents=True)
        shutil.copyfile(ini, staged / "NematX.ini")
        shutil.copyfile(checkout / "LICENSE", staged / "NematX-LICENSE.txt")
        for name, source in files.items():
            destination = staged / "NematX" / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

        profiles = resources / "profiles"
        if profiles.exists():
            shutil.rmtree(profiles)
        shutil.move(str(staged), str(profiles))
        print(f"Packaged NematX profile {version} from {BRANCH} at {revision}")


def main() -> int:
    """Require a path to the package resources and return a nonzero code on failure."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("resources", type=Path, help="path to the staged package resources")
    args = parser.parse_args()
    try:
        stage_profiles(args.resources)
    except (OSError, ValueError, configparser.Error, subprocess.SubprocessError) as error:
        print(f"Could not package the NematX profile: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
