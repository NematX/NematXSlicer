"""Run the SDK geometry executable with explicit host/dependency library paths.

The extra directories are test inputs, not paths inferred from another checkout.
Failed child exits remain failures and include their numeric status for diagnosis.
    argparse -> subprocess.run -> preserve the child exit status
"""
import argparse
import os
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--exe", required=True)
parser.add_argument("--runtime-path", action="append", default=[])
args = parser.parse_args()
environment = dict(os.environ)
key = "PATH" if os.name == "nt" else "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
environment[key] = os.pathsep.join(args.runtime_path + [environment.get(key, "")])
result = subprocess.run([args.exe], env=environment)
if result.returncode:
    print(f"SDK geometry exit: {result.returncode} (0x{result.returncode & 0xffffffff:08x})", file=sys.stderr)
sys.exit(result.returncode)
