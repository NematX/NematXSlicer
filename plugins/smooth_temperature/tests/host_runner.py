"""Bound host commands by a shared wall-clock deadline and retain diagnostics.

HostRunner.run writes output directly to a case log, waits only for the remaining
budget, and kills the child's process tree on timeout. No captured pipe can keep
the runner alive after an orphaned child inherits it. Calls and their durations
are recorded for the E2E report; the budget starts before fixture preparation.

    HostRunner -> run -> Popen.wait -> kill process tree on timeout
"""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path


class HostRunner:
    def __init__(self, seconds: float, env: dict[str, str]):
        """Start one budget shared by all commands of a suite."""
        self.started = time.monotonic()
        self.deadline = self.started + seconds
        self.env = env
        self.calls: list[dict] = []

    def remaining(self) -> float:
        """Fail rather than silently omit a scenario when the total budget is exhausted."""
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError("SmoothTemperature E2E exceeded its global wall-clock budget")
        return remaining

    def run(self, args: list[str], log: Path, per_command: float = 12.0) -> None:
        """Execute one command and retain partial output even on a timeout or failure."""
        timeout = min(per_command, self.remaining())
        start = time.monotonic()
        options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
        with log.open("wb") as stream:
            process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, env=self.env, **options)
            try:
                process.wait(timeout=timeout)
            except BaseException:
                # Stop descendants before the parent disappears, preserving ancestry on Windows.
                if os.name == "nt":
                    try:
                        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, timeout=3, check=False)
                    finally:
                        if process.poll() is None:
                            process.kill()
                else:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                process.wait(timeout=3)
                raise
            finally:
                self.calls.append({"case": log.parent.name, "seconds": time.monotonic() - start,
                                   "exit": process.returncode, "args": args})
        if process.returncode:
            raise AssertionError(f"Host exited with {process.returncode}; see {log}")
        self.remaining()
