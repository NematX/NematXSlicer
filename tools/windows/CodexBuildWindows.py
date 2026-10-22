"""Run a CMake build without leaking Codex's environment into MSBuild nodes.

The Windows environment is case-insensitive, but process launchers may still
provide both ``Path`` and ``PATH``. .NET Framework's MSBuild tool tasks reject
that environment while starting CL.exe. This wrapper rebuilds the environment
with one PATH entry and passes ``/nr:false`` so MSBuild workers remain usable
for the complete build but exit instead of being reused by Visual Studio.

Visual Studio's generated ``ZERO_CHECK`` project may regenerate every project
while MSBuild is already traversing them in parallel. In this repository that
can make the parent build cancel its custom-build task, return early and leave
idle worker nodes behind. The wrapper therefore performs CMake's glob and
build-system checks synchronously before starting MSBuild. ``ZERO_CHECK`` then
has no project regeneration to perform while ``cl.exe`` compiles through /MP.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import msvcrt


class _BuildDirectoryLock:
    """Serialize wrapper invocations that target the same build directory."""

    def __init__(self, build_directory: Path) -> None:
        self._path = build_directory / ".codex-build.lock"
        self._file = None

    def __enter__(self) -> "_BuildDirectoryLock":
        self._file = self._path.open("a+b")
        self._file.seek(0, os.SEEK_END)
        if self._file.tell() == 0:
            self._file.write(b"\0")
            self._file.flush()

        waiting_reported = False
        while True:
            try:
                self._file.seek(0)
                msvcrt.locking(self._file.fileno(), msvcrt.LK_NBLCK, 1)
                return self
            except OSError:
                if not waiting_reported:
                    print(
                        f"Waiting for the build already using {self._path.parent}...",
                        flush=True,
                    )
                    waiting_reported = True
                time.sleep(1)

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        assert self._file is not None
        self._file.seek(0)
        msvcrt.locking(self._file.fileno(), msvcrt.LK_UNLCK, 1)
        self._file.close()


def _normalized_environment() -> dict[str, str]:
    """Return a PATH-safe environment without the unsafe MSBuild override."""
    path_value = os.environ.get("PATH", os.environ.get("Path", ""))
    environment = {
        key: value
        for key, value in os.environ.items()
        if key.casefold() not in {"path", "msbuilddisablenodereuse"}
    }
    environment["PATH"] = path_value
    return environment


def _cache_values(build_directory: Path) -> dict[str, str]:
    """Read the few CMake cache entries needed by the regeneration preflight."""
    cache_path = build_directory / "CMakeCache.txt"
    values: dict[str, str] = {}
    try:
        lines = cache_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        raise RuntimeError(f"Cannot read {cache_path}: {exc}") from exc

    for line in lines:
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        declaration, value = line.split("=", 1)
        name = declaration.split(":", 1)[0]
        if name in {"CMAKE_GENERATOR", "CMAKE_HOME_DIRECTORY", "CMAKE_PROJECT_NAME"}:
            values[name] = value
    return values


def _build_directory(arguments: list[str]) -> Path:
    """Return the build tree named by ``cmake --build <directory>`` arguments."""
    if not arguments or arguments[0].startswith("-"):
        raise RuntimeError(
            "The wrapper requires an explicit build directory as its first argument."
        )
    return Path(arguments[0]).resolve()


def _run_visual_studio_preflight(
    cmake: str, build_directory: Path, environment: dict[str, str]
) -> int:
    """Complete CMake regeneration before parallel MSBuild sees the projects."""
    cache = _cache_values(build_directory)
    if not cache.get("CMAKE_GENERATOR", "").startswith("Visual Studio"):
        return 0

    source_directory = cache.get("CMAKE_HOME_DIRECTORY")
    project_name = cache.get("CMAKE_PROJECT_NAME")
    if not source_directory or not project_name:
        raise RuntimeError(
            "CMakeCache.txt does not identify the source directory and project name."
        )

    verify_globs = build_directory / "CMakeFiles" / "VerifyGlobs.cmake"
    if verify_globs.exists():
        result = subprocess.run(
            [cmake, "-P", str(verify_globs)], env=environment, check=False
        )
        if result.returncode != 0:
            return result.returncode

    stamp_list = build_directory / "CMakeFiles" / "generate.stamp.list"
    solution = build_directory / f"{project_name}.sln"
    if not stamp_list.exists() or not solution.exists():
        raise RuntimeError(
            "The Visual Studio build tree is missing generate.stamp.list or its solution."
        )

    # This is the same up-to-date check generated in ZERO_CHECK.vcxproj. Running
    # it directly makes any regeneration finish before MSBuild loads the graph.
    return subprocess.run(
        [
            cmake,
            f"-S{source_directory}",
            f"-B{build_directory}",
            "--check-stamp-list",
            "CMakeFiles/generate.stamp.list",
            "--vs-solution-file",
            str(solution),
        ],
        cwd=build_directory,
        env=environment,
        check=False,
    ).returncode


def _cmake_command(arguments: list[str], environment: dict[str, str]) -> list[str]:
    """Build the command with stable Visual Studio project scheduling."""
    cmake = shutil.which("cmake.exe", path=environment["PATH"])
    if cmake is None:
        raise RuntimeError("cmake.exe was not found in PATH")

    command = [cmake, "--build", *arguments]
    native_separator = command.index("--") if "--" in command else len(command)
    if native_separator == len(command):
        command.append("--")
        native_separator = len(command) - 1

    native_options = {
        "/nr:": "/nr:false",
        "/p:buildinparallel=": "/p:BuildInParallel=false",
    }
    found_options: set[str] = set()
    for index in range(native_separator + 1, len(command)):
        lower_argument = command[index].lower()
        for prefix, replacement in native_options.items():
            if lower_argument.startswith(prefix):
                command[index] = replacement
                found_options.add(prefix)
                break
    for prefix, replacement in native_options.items():
        if prefix not in found_options:
            command.append(replacement)
    return command


def main() -> int:
    """Prepare one build tree, run it once and return its real exit status."""
    if os.name != "nt":
        print("CodexBuildWindows.py is only intended for Windows.", file=sys.stderr)
        return 2
    if not sys.argv[1:]:
        print(
            "Usage: python cmake/CodexBuildWindows.py <build-dir> [options]",
            file=sys.stderr,
        )
        return 2

    try:
        environment = _normalized_environment()
        arguments = sys.argv[1:]
        build_directory = _build_directory(arguments)
        command = _cmake_command(arguments, environment)
        cmake = command[0]

        with _BuildDirectoryLock(build_directory):
            preflight_status = _run_visual_studio_preflight(
                cmake, build_directory, environment
            )
            if preflight_status != 0:
                return preflight_status
            return subprocess.run(
                command, env=environment, check=False
            ).returncode
    except (OSError, RuntimeError) as exc:
        print(f"CodexBuildWindows.py: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
