"""Pinned upstream inputs must work with either checkout line-ending policy."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

script = Path(__file__).resolve().parents[1] / "tools/import_rectilinear.py"
spec = importlib.util.spec_from_file_location("import_rectilinear", script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
source = Path(sys.argv[1])
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    results = []
    for label, newline in (("lf", b"\n"), ("crlf", b"\r\n")):
        checkout, output = root / label, root / (label + "-output")
        for name in module.HASHES:
            relative = Path("Plugins/Infill") / (name if name == "InfillWorkBuffers.hpp" else "Rectilinear/" + name)
            path = checkout / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((source / relative).read_bytes().replace(b"\r\n", b"\n").replace(b"\n", newline))
        subprocess.run([sys.executable, str(script), str(checkout), "--output", str(output)], check=True)
        result = {name: (output / name).read_bytes() for name in module.HASHES}
        for name, data in result.items():
            assert data == (module.ROOT / name).read_bytes().replace(b"\r\n", b"\n"), name
        results.append(result)
        # A real source change must fail, regardless of checkout line endings.
        path.write_bytes(path.read_bytes() + b"// changed\n")
        rejected = subprocess.run([sys.executable, str(script), str(checkout), "--output", str(output)], capture_output=True)
        assert rejected.returncode != 0 and b"Unreviewed source revision" in rejected.stderr
    assert results[0] == results[1]
print("LF/CRLF import reproduction and content rejection passed")
