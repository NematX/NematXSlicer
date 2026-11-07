"""Copy the host rectilinear fill algorithms used by the wipe tower maintenance.

Maintenance tool, NOT a build dependency. Input: the pinned v3 src/libslic3r directory.
The tower fills its unused area with the host's Monotonic (first layer) and Grid
(upper layers) algorithms. They are only reachable through STEP_INFILL in the
SDK, so the package keeps a reviewed copy. Whole files are copied; only the
namespace, the include paths and the boost dependencies change:
- BOOST_LOG_TRIVIAL error lines become one fprintf on stderr;
- boost::container::small_vector becomes std::vector (same order, no fixed buffer);
- boost::random becomes std random; only the scattered pattern uses it, never the tower.
The committed generated files do not change with later host changes.
"""
import argparse
import hashlib
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "rectilinear"
# File name -> SHA-256 of the reviewed host source, with LF line endings.
HASHES = {
    "InfillWorkBuffers.hpp": "48df7ecf5d6b7d826c1bb1bcbd58b2060db174be555e891abf8c16ea1a439763",
    "RectilinearWorkBuffers.hpp": "2c95791504285392aeb3cb3915f037fb4b3c2e4a54436e13e9ab7d0f413c9f26",
    "RectilinearClipper.hpp": "8c53e36f3825044c609132c5bcfa7d4c47d5390107b56788a7b32f858c9446e3",
    "RectilinearEdgeGrid.hpp": "7569930ba7881f7badb0e2a4e73e54c680b712df664bf6c5d83a77e6ef710e07",
    "RectilinearEdgeGrid.cpp": "1a68ae7d13caa432abc68c04cec8e15622968017123191289a0bf7ed8f308ba1",
    "RectilinearConnect.hpp": "fa4971916782751644480a4596c4f25007346e291de8f97c1cd92105a0c3d754",
    "RectilinearFills.hpp": "b3cf4b5c28b9f1c77996f0dca64d42884b3c05a1233bc89426264df8ee92b0b9",
    "RectilinearFills.cpp": "b6c79befd2038a7e715711a2903d8ebacadd3fd3d7a900a46425169276501f52",
}


def source_path(source, name):
    """Host location of one copied file."""
    folder = source / "Plugins" / "Infill"
    return folder / name if name == "InfillWorkBuffers.hpp" else folder / "Rectilinear" / name


def convert(name, text):
    """Rewrite one host file so that it builds inside the package."""
    if name == "RectilinearFills.cpp":
        text = text.replace("#include <boost/log/trivial.hpp>\n", "")
        text = text.replace("#include <boost/random/mersenne_twister.hpp>\n", "")
        text = text.replace("#include <boost/random/uniform_real_distribution.hpp>\n", "")
        text = text.replace("boost::container::small_vector<MonotonicRegion*, 4>", "std::vector<MonotonicRegion*>")
        text = text.replace("boost::random::mt19937", "std::mt19937")
        text = text.replace("boost::random::uniform_real_distribution<>", "std::uniform_real_distribution<>")
        lines = []
        for line in text.split("\n"):
            stripped = line.lstrip()
            if stripped.startswith("BOOST_LOG_TRIVIAL(error)"):
                indent = line[:len(line) - len(stripped)]
                message = re.match(r'BOOST_LOG_TRIVIAL\(error\) << "([^"]*)"', stripped).group(1)
                line = indent + 'std::fprintf(stderr, "nematx-wipetower: %s\\n", "' + message + '");'
            lines.append(line)
        text = "\n".join(lines)
        if "boost::" in text or "#include <boost" in text:
            raise ValueError("Unconverted boost dependency in RectilinearFills.cpp")
    text = text.replace("namespace Infill", "namespace NematXWipeTowerCopy")
    text = text.replace("slic3r_api::Infill", "slic3r_api::NematXWipeTowerCopy")
    text = text.replace('"libslic3r/Plugins/Infill/InfillWorkBuffers.hpp"', '"InfillWorkBuffers.hpp"')
    text = text.replace("plugins_infill_", "nematx_wipetower_copy_")
    return "\n".join(line.rstrip() for line in text.splitlines()) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="host src/libslic3r directory")
    parser.add_argument("--output", type=Path, default=ROOT)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Input must be a separate host checkout")
    for name, expected in HASHES.items():
        path = source_path(source, name)
        data = path.read_bytes().replace(b"\r\n", b"\n")
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f"Unreviewed source revision: {path}")
        output.mkdir(parents=True, exist_ok=True)
        (output / name).write_text(convert(name, data.decode("utf-8")), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
