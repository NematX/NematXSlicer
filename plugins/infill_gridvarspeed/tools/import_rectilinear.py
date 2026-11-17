"""Reproduce the vendored non-monotonic single-sweep subset. Never writes upstream.

Maintenance tool, NOT a build dependency. Input: pinned v3 src/libslic3r directory.
The committed generated files are independent of subsequent host changes.
"""
import hashlib
import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "rectilinear"
HASHES = {
    "RectilinearFills.cpp": "b6c79befd2038a7e715711a2903d8ebacadd3fd3d7a900a46425169276501f52",
    "PolygonWorkBuffers.hpp": "8389fc6f43ea9d67b1ea32bc677657bf89c1632cb5be9aba929d0eb65561650f",
    "ClipperHelpers.hpp": "b1d8bb4b90ecfb44f53599d329f7986bb538db1aee5cc740a5416096cd666424",
    "WorkBuffers.hpp": "a4121d62a9298c9db02d7c178cea8def3c3cb4f233bd5f1d15dff620a19c3f99",
}


def source_path(source, name):
    """Host location of one copied file: the fill algorithm, or a shared plugin helper."""
    plugins = source / "Plugins"
    return plugins / "Infill" / "Rectilinear" / name if name == "RectilinearFills.cpp" else plugins / "Common" / name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Input must be a separate upstream checkout")
    for name, expected in HASHES.items():
        path = source_path(source, name)
        data = path.read_bytes().replace(b"\r\n", b"\n")
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f"Unreviewed source revision: {path}")
        text = data.decode("utf-8").replace("\r\n", "\n")
        if name == "RectilinearFills.cpp":
            lines = text.splitlines(keepends=True)
            # raster / contour graph, without phony monotonic intersections;
            # graph traversal; single-sweep public boundary adapter.
            text = "".join(lines[:368] + lines[386:1285] + lines[1428:1730] + lines[2835:2957])
            text = text.replace("#include <boost/log/trivial.hpp>\n", "")
            text = text.replace("#include <boost/random/mersenne_twister.hpp>\n", "")
            text = text.replace("#include <boost/random/uniform_real_distribution.hpp>\n", "")
            begin = text.index("    //FIXME this is a hack to get the monotonic infill rolling.")
            end = text.index("        std::vector<SegmentedIntersectionLine> segs_save", begin)
            text = text[:begin] + "    { // gridvarspeed accepts only the ordinary connected sweep.\n" + text[end:]
            text = "\n".join(
                '            std::fprintf(stderr, "GridVarSpeed: invalid raster intersections\\n");'
                if line.lstrip().startswith("BOOST_LOG_TRIVIAL") else line
                for line in text.splitlines()
            ) + "\n\n} // namespace fills\n}}} // namespace slic3r_api::GridVarSpeedCopy::RectilinearPatternsPlugin\n"
        for host in ("Infill", "PluginCommon"):
            # Namespace uses only: comments and messages keep the word Infill.
            text = re.sub(r"\b(namespace |slic3r_api::)" + host + r"\b", r"\1GridVarSpeedCopy", text)
            text = text.replace(host + "::", "GridVarSpeedCopy::")
        text = text.replace('"libslic3r/Plugins/Common/', '"')
        text = text.replace("plugins_infill_", "nematx_gridvarspeed_copy_")
        text = text.replace("plugins_common_", "nematx_gridvarspeed_copy_common_")
        text = "\n".join(line.rstrip() for line in text.splitlines()) + "\n"
        output.mkdir(parents=True, exist_ok=True)
        (output / name).write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
