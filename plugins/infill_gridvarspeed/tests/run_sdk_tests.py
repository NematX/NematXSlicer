"""Run an SDK test with the existing host library directory on its runtime path."""
import argparse
import os
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--library-dir", required=True)
parser.add_argument("--exe", required=True)
args = parser.parse_args()
env = os.environ.copy()
key = "PATH" if os.name == "nt" else "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
env[key] = args.library_dir + os.pathsep + env.get(key, "")
sys.exit(subprocess.run([args.exe], env=env).returncode)
