"""Copy the host rectilinear fill algorithms used by the wipe tower maintenance.

Maintenance tool, NOT a build dependency. Input: the pinned v3 src/libslic3r directory.
The tower fills its unused area with the host's Monotonic (first layer) and Grid
(upper layers) algorithms. They are only reachable through STEP_INFILL in the
SDK, so the package keeps a reviewed copy of them and of the shared plugin
helpers they use (Plugins/Common). Whole files are copied; only the namespaces,
the include paths and the boost dependencies change:
- the host namespaces Infill and PluginCommon both become NematXWipeTowerCopy;
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
    "WorkBuffers.hpp": "a4121d62a9298c9db02d7c178cea8def3c3cb4f233bd5f1d15dff620a19c3f99",
    "PolygonWorkBuffers.hpp": "8389fc6f43ea9d67b1ea32bc677657bf89c1632cb5be9aba929d0eb65561650f",
    "ClipperHelpers.hpp": "b1d8bb4b90ecfb44f53599d329f7986bb538db1aee5cc740a5416096cd666424",
    "PluginEdgeGrid.hpp": "a091a8cbe33ef461c8df671aca3849bb4575143bded6b7795e8355f7c2fb35d9",
    "PluginEdgeGrid.cpp": "1ead8c7f362a87d0ce1d2d273c1b06f91c5cacc7aa01d0d18dff3034fdfc7078",
    "RectilinearConnect.hpp": "f7c15af206de45d4ce7d2bf7227b3b2fd5c07939dc6f5e5ff54ffcbcbf4bb65e",
    "RectilinearFills.hpp": "cf2d20c47c10a1c5bfcd602f0edf5012fa91a05a68aa0c0d850b52815051669b",
    "RectilinearFills.cpp": "b6c79befd2038a7e715711a2903d8ebacadd3fd3d7a900a46425169276501f52",
}
# Files taken from the shared plugin helpers; the others come from the rectilinear infill.
COMMON = {"WorkBuffers.hpp", "PolygonWorkBuffers.hpp", "ClipperHelpers.hpp", "PluginEdgeGrid.hpp", "PluginEdgeGrid.cpp"}


def source_path(source, name):
    """Host location of one copied file."""
    plugins = source / "Plugins"
    return plugins / "Common" / name if name in COMMON else plugins / "Infill" / "Rectilinear" / name


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
    for host in ("Infill", "PluginCommon"):
        # Namespace uses only: comments and messages keep the word Infill.
        text = re.sub(r"\b(namespace |slic3r_api::)" + host + r"\b", r"\1NematXWipeTowerCopy", text)
        text = text.replace(host + "::", "NematXWipeTowerCopy::")
    text = text.replace('"libslic3r/Plugins/Common/', '"')
    text = text.replace("plugins_infill_", "nematx_wipetower_copy_")
    text = text.replace("plugins_common_", "nematx_wipetower_copy_common_")
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
