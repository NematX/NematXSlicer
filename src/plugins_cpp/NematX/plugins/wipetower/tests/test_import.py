"""The committed rectilinear copy is reproduced from the pinned host files, with LF or CRLF checkouts."""
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
            path = module.source_path(checkout, name)
            path.parent.mkdir(parents=True, exist_ok=True)
            data = module.source_path(source, name).read_bytes().replace(b"\r\n", b"\n")
            path.write_bytes(data.replace(b"\n", newline))
        subprocess.run([sys.executable, str(script), str(checkout), "--output", str(output)], check=True)
        result = {name: (output / name).read_bytes() for name in module.HASHES}
        for name, data in result.items():
            assert data == (module.ROOT / name).read_bytes().replace(b"\r\n", b"\n"), name
        results.append(result)
        # A changed host file must be refused, whatever the checkout line endings.
        path.write_bytes(path.read_bytes() + b"// changed\n")
        rejected = subprocess.run([sys.executable, str(script), str(checkout), "--output", str(output)],
                                  capture_output=True)
        assert rejected.returncode != 0 and b"Unreviewed source revision" in rejected.stderr
    assert results[0] == results[1]
print("LF/CRLF import reproduction and content rejection passed")
