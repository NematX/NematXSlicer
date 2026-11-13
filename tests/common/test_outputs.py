"""Bounded disk use of the export tests.

Each export case gets its own host data directory, in which the host installs
and caches its plugin packages: several megabytes per case, whereas the G-code
and the log that a reader needs weigh a few kilobytes. These helpers keep only
what a diagnosis needs:

    root = new_run(results, "run-")   # removes older runs: KEEP_RUNS remain
    ...                               # for each case: export and check, then
    remove_data(case / "datadir")     # once the case passed
    finish_run(root, passed)          # a passed run is removed entirely

A failed case keeps its data directory, and a failed run keeps all its files.
Two variants of a test sharing one results directory need different prefixes,
so that one never removes a run of the other.
"""
from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

# Runs kept per test and prefix, the new one included: the latest failed run
# stays next to the new run.
KEEP_RUNS = 2


def new_run(results: Path, prefix: str) -> Path:
    """Create a new run directory under results, after removing the older runs with the same prefix."""
    results.mkdir(parents=True, exist_ok=True)
    runs = sorted((path for path in results.iterdir() if path.is_dir() and path.name.startswith(prefix)),
                  key=lambda path: path.stat().st_mtime)
    for old in runs[:max(0, len(runs) - (KEEP_RUNS - 1))]:
        shutil.rmtree(old, ignore_errors=True)
    return Path(tempfile.mkdtemp(prefix=prefix, dir=results))


def remove_data(datadir: Path) -> None:
    """Remove the host data directory of a passed case; its G-code and log stay beside it."""
    shutil.rmtree(datadir, ignore_errors=True)


def finish_run(root: Path, passed: bool) -> None:
    """Remove a passed run entirely; a failed run keeps every file for diagnosis."""
    if passed:
        shutil.rmtree(root, ignore_errors=True)
