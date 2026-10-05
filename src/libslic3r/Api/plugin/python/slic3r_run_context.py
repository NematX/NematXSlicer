#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Common high-level view over ``plugin_run_context``.

Every object-step payload starts with the same host-owned run context.  This
module keeps those operations in one binding class so calls such as
``ctx.report_error()`` require only ``slic3r_plugin_run_context.h``.  Step
modules inherit this view and give their payload-specific accessors unique
names; the static ABI analyzer can therefore identify the precise step contract
without inferring the receiver's Python type.

The view borrows the run context and its storage.  It is valid only for the
current plugin callback and never frees either object.
"""

from __future__ import annotations

import ctypes
import threading

from slic3r_api_generated import (
    PLUGIN_IS_CANCELLED,
    PLUGIN_REPORT,
    PLUGIN_REPORT_PROGRESS,
    PluginRunContext,
    SLIC3R_PARALLEL_FOR_STORAGE,
)
from slic3r_abi import abi_requires_headers


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    return int(handle)


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


def _optional_bytes(text: str | None) -> bytes | None:
    return None if text is None else text.encode("utf-8")


@abi_requires_headers("slic3r_plugin_run_context.h")
class PluginRunContextView:
    """Expose operations shared by every step-specific run context."""

    def __init__(self, api, common: PluginRunContext) -> None:
        self.api = api
        self.common = common

    def plugin_storage(self) -> int:
        return _address(self.common.plugin_storage)

    def is_cancelled(self) -> bool:
        if not self.common.is_cancelled:
            return False
        return bool(PLUGIN_IS_CANCELLED(self.common.is_cancelled)(self.common.host_context))

    def throw_if_cancelled(self) -> None:
        """Raise ``PluginCancelled`` when the host requests cooperative stop."""
        if self.is_cancelled():
            raise PluginCancelled()

    def report_warning(self, message: str) -> None:
        if self.common.report_warning:
            PLUGIN_REPORT(self.common.report_warning)(self.common.host_context, _as_bytes(message))

    def report_error(self, message: str) -> None:
        if self.common.report_error:
            PLUGIN_REPORT(self.common.report_error)(self.common.host_context, _as_bytes(message))

    def report_progress(self, progress: float, message: str | None = None) -> None:
        if self.common.report_progress:
            PLUGIN_REPORT_PROGRESS(self.common.report_progress)(
                self.common.host_context, float(progress), _optional_bytes(message)
            )

    @abi_requires_headers("steps/slic3r_step_support_demand.h")
    def support_demand_get_island_mutable(self, island):
        """
        Borrow one island's support demand for in-place edits.

        During a pipeline run it is allowed from STEP_SUPPORT_DEMAND through
        STEP_SUPPORT; in another step the host reports an error and None is
        returned. The collection is owned by the
        island: never free it.
        """
        from slic3r_geometry_views import MutableExPolygonCollection

        handle = self.api.host.support_demand_get_island_mutable(
            ctypes.c_void_p(ctypes.addressof(self.common)), ctypes.c_void_p(_address(island))
        )
        return None if not handle else MutableExPolygonCollection(self.api, handle)


class PluginCancelled(Exception):
    """
    Cooperative cancellation signal consumed by the Python plugin bridge.

    Plugins may raise it through ``throw_if_cancelled()`` to unwind nested
    Python code without reporting a plugin failure. It owns no native state.
    """

    __slic3r_plugin_cancelled__ = True


def _clear_exception_references(error: BaseException) -> None:
    """Release traceback frames, including chained exceptions, before scratch cleanup."""
    pending = [error]
    visited = set()
    while pending:
        current = pending.pop()
        identity = id(current)
        if identity in visited:
            continue
        visited.add(identity)
        if current.__cause__ is not None:
            pending.append(current.__cause__)
        if current.__context__ is not None:
            pending.append(current.__context__)

        traceback = current.__traceback__
        while traceback is not None:
            try:
                traceback.tb_frame.clear()
            except RuntimeError:
                # The callback frame is still executing. Its exception target
                # is cleared automatically when this except block ends.
                pass
            traceback = traceback.tb_next
        current.__traceback__ = None
        current.__cause__ = None
        current.__context__ = None


def _exception_snapshot(error: BaseException) -> tuple[type[BaseException], str, bool]:
    """Copy only inert diagnostic data, never traceback locals or exception arguments."""
    try:
        message = str(error)
    except BaseException:
        message = "exception message could not be formatted"
    return type(error), message, isinstance(error, PluginCancelled)


def _raise_exception_snapshot(snapshot: tuple[type[BaseException], str, bool]) -> None:
    """Rebuild a safe exception after the host has destroyed worker scratch storage."""
    exception_type, message, cancelled = snapshot
    if cancelled:
        raise PluginCancelled(message)
    try:
        rebuilt = exception_type(message)
    except BaseException:
        rebuilt = RuntimeError(f"{exception_type.__name__}: {message}")
    raise rebuilt


def _context_view(context) -> PluginRunContextView | None:
    if context is None:
        return None
    if isinstance(context, PluginRunContextView):
        return context
    if isinstance(context, PluginRunContext):
        return PluginRunContextView(None, context)
    address = _address(context)
    if not address:
        return None
    return PluginRunContextView(None, PluginRunContext.from_address(address))


@abi_requires_headers("slic3r_plugin_run_context.h")
def throw_if_cancelled(context) -> None:
    """Raise ``PluginCancelled`` if a borrowed run context was cancelled."""
    view = _context_view(context)
    if view is not None:
        view.throw_if_cancelled()


@abi_requires_headers("slic3r_plugin_run_context.h")
class PluginProgress:
    """
    Thread-safe work counter with up to sixteen sequential visible phases.

    ``reset()`` borrows only the host callback and host-context pointer, not the
    run-context structure. Those pointers must remain valid through all setup
    and run callbacks of the current plugin execution. The helper allocates no
    native memory and must be reset before reuse for another execution.
    """

    DEFAULT_PHASE = 0
    MAX_PHASES = 16

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset(None, 0)

    def reset(
        self,
        context,
        run_count: int,
        message_format: str = "Plugin progress: {done} / {max}",
    ) -> None:
        view = _context_view(context)
        with self._lock:
            self._host_context = None if view is None else view.common.host_context
            self._report = None
            if view is not None and view.common.report_progress:
                self._report = PLUGIN_REPORT_PROGRESS(view.common.report_progress)
            self._run_count = max(0, int(run_count))
            self._phases = [
                {"done": 0, "max": 0, "finished": 0, "active": False,
                 "format": message_format}
                for _unused in range(self.MAX_PHASES)
            ]
            self._phases[self.DEFAULT_PHASE]["active"] = True
            self._last_percent = -1
            self._last_phase = -1

    def set_phase_format(self, phase: int, message_format: str) -> None:
        with self._lock:
            state = self._phase(phase)
            state["format"] = str(message_format)
            state["active"] = True

    def add_max(self, count: int, phase: int = DEFAULT_PHASE) -> None:
        if int(count) < 0:
            raise ValueError("progress maximum increments must not be negative")
        with self._lock:
            state = self._phase(phase)
            state["active"] = True
            state["max"] += int(count)

    def increment(self, phase: int = DEFAULT_PHASE) -> None:
        self.increment_by(1, phase)

    def increment_by(self, count: int, phase: int = DEFAULT_PHASE, message: str | None = None) -> None:
        with self._lock:
            state = self._phase(phase)
            state["active"] = True
            state["done"] += int(count)
            self._report_if_needed(int(phase), message)

    def finish_run(self, phase: int = DEFAULT_PHASE) -> None:
        """Finish one run and publish 100% when it completes the final visible phase."""
        with self._lock:
            state = self._phase(phase)
            state["active"] = True
            state["finished"] += 1
            completed_by_this_call = state["finished"] == self._run_count
            if completed_by_this_call:
                state["done"] = max(state["done"], state["max"])
            self._report_if_needed(
                int(phase), None, allow_completed_updated_phase=completed_by_this_call
            )

    def done(self, phase: int = DEFAULT_PHASE) -> int:
        with self._lock:
            return int(self._phase(phase)["done"])

    def maximum(self, phase: int = DEFAULT_PHASE) -> int:
        with self._lock:
            return int(self._phase(phase)["max"])

    def run_count(self) -> int:
        with self._lock:
            return self._run_count

    def _phase(self, phase: int):
        phase = int(phase)
        if phase < 0 or phase >= self.MAX_PHASES:
            raise IndexError(f"progress phase {phase} is outside [0, {self.MAX_PHASES})")
        return self._phases[phase]

    def _first_visible_phase(self) -> int | None:
        for index, state in enumerate(self._phases):
            if not state["active"]:
                continue
            if state["finished"] < self._run_count or state["done"] < state["max"]:
                return index
        return None

    @staticmethod
    def _message(state) -> str:
        done = min(max(int(state["done"]), 0), int(state["max"]))
        try:
            return str(state["format"]).format(done=done, max=int(state["max"]))
        except (IndexError, KeyError, ValueError):
            return f"Plugin progress: {done} / {int(state['max'])}"

    def _report_if_needed(
        self,
        updated_phase: int,
        message: str | None,
        allow_completed_updated_phase: bool = False,
    ) -> None:
        visible = self._first_visible_phase()
        if self._report is None:
            return
        if visible is None:
            # The final completed phase is no longer normally visible. Only
            # the exact finish transition may expose it for its terminal 100%.
            if not allow_completed_updated_phase:
                return
            visible = updated_phase
        state = self._phases[visible]
        maximum = int(state["max"])
        if maximum == 0:
            return
        done = min(max(int(state["done"]), 0), maximum)
        percent = done * 100 // maximum
        phase_changed = visible != self._last_phase
        if not phase_changed and percent == self._last_percent:
            return
        self._last_phase = visible
        self._last_percent = percent
        text = message if updated_phase == visible and message is not None else self._message(state)
        self._report(self._host_context, percent / 100.0, _optional_bytes(text))


@abi_requires_headers("slic3r_utils.h", "slic3r_plugin_run_context.h")
def parallel_for_storage_with_progress(
    api,
    begin: int,
    end: int,
    context,
    progress: PluginProgress | None,
    worker,
    *args,
) -> None:
    """
    Run independent indices through the host executor with scratch storage.

    ``worker(index, scratch_storage, *args)`` must not retain handles allocated
    from ``scratch_storage`` after it returns: the host destroys that storage
    immediately. The call waits for every worker. The first Python exception is
    rebuilt and raised afterward because exceptions and their tracebacks cannot
    cross a ctypes callback or outlive its scratch storage.
    Shared Python/native mutations remain the caller's synchronization duty.
    """
    first = int(begin)
    last = int(end)
    if first < 0 or last < first or last > 0xFFFFFFFF:
        raise ValueError("parallel range must be an ordered uint32 half-open range")
    failures = []
    failure_lock = threading.Lock()

    def invoke(index, scratch_storage, _user_data) -> None:
        with failure_lock:
            if failures:
                return
        try:
            throw_if_cancelled(context)
            worker(int(index), _address(scratch_storage), *args)
            if progress is not None:
                progress.increment()
        except BaseException as exc:
            snapshot = _exception_snapshot(exc)
            _clear_exception_references(exc)
            with failure_lock:
                if not failures:
                    failures.append(snapshot)

    callback = SLIC3R_PARALLEL_FOR_STORAGE(invoke)
    api.host.slic3r_parallel_for_storage(first, last, None, callback)
    if failures:
        _raise_exception_snapshot(failures[0])


__all__ = [
    "PluginCancelled",
    "PluginProgress",
    "PluginRunContextView",
    "parallel_for_storage_with_progress",
    "throw_if_cancelled",
]
