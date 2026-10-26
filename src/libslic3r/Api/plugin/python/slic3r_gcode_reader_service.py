#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Plugin-side client for executing one active G-code Reader.

The helper builds the normal Reader payload and executes it through
``OrchestratorView``. It owns every ctypes callback for the duration of the
synchronous call, validates emitted events independently, and copies the final
state into Python values before any borrowed pointer expires.
"""

from __future__ import annotations

import ctypes
import math
import threading

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    GCODE_READER,
    GCODE_READER_EMIT_EVENT,
    GCODE_READER_EMIT_FINAL_STATE,
    GCODE_READER_GET_NEXT_CHUNK,
    RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND,
    RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT,
    RAW_GCODE_READER_EVENT_DIAGNOSTIC,
    RAW_GCODE_READER_EVENT_INVALIDATE,
    RAW_GCODE_READER_EVENT_MARKER,
    RAW_GCODE_READER_EVENT_STATE_UPDATE,
    RAW_GCODE_READER_ITEM_CUSTOM_BEGIN,
    RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE,
    RAW_GCODE_READER_ITEM_LINE_NUMBER,
    RAW_GCODE_READER_ITEM_MARKER,
    RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT,
    RAW_GCODE_READER_ITEM_READER_ID,
    RAW_GCODE_READER_MARKER_CUSTOM_GCODE,
    RAW_GCODE_READER_MARKER_SEAM,
    RAW_GCODE_READER_OPERATION_READ,
    RAW_GCODE_READER_STATUS_ERROR,
    RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_READER_STATUS_SUCCESS,
    RAW_PLUGIN_EXECUTION_CANCELLED,
    RAW_PLUGIN_EXECUTION_INACTIVE,
    RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT,
    RAW_PLUGIN_EXECUTION_PLUGIN_ERROR,
    RAW_PLUGIN_EXECUTION_SUCCESS,
    RawGcodeReaderEventItem,
    RawGcodeReaderState,
    RunCtxGcodeReader,
)
from slic3r_orchestrator_views import OrchestratorView


DEFAULT_GCODE_READER_PLUGIN_ID = "gcode.reader.default"


def _copied_items(raw_state, event_kind=None) -> tuple[tuple[int, int, float], ...]:
    """Validate and copy borrowed items through MAX_LAYER_HEIGHT; invalid items raise ValueError."""
    count = int(raw_state.item_count)
    if count and not raw_state.items:
        raise ValueError("a G-code Reader item array is null")
    copied = []
    seen = set()
    last_standard = int(RAW_GCODE_READER_ITEM_MAX_LAYER_HEIGHT)
    for index in range(count):
        item = raw_state.items[index]
        item_type = int(item.type)
        target = int(item.target)
        value = float(item.value)
        if item_type <= 0 or (
            item_type > last_standard
            and item_type < int(RAW_GCODE_READER_ITEM_CUSTOM_BEGIN)
        ):
            raise ValueError(f"unknown G-code Reader item type {item_type}")
        if not math.isfinite(value):
            raise ValueError("G-code Reader item values must be finite")
        key = (item_type, target)
        if key in seen:
            raise ValueError("a G-code Reader item array contains a duplicate key")
        seen.add(key)
        copied.append((item_type, target, value))
    line = next(
        (item for item in copied if item[0] == int(RAW_GCODE_READER_ITEM_LINE_NUMBER)),
        None,
    )
    if line is None or line[1] != 0 or line[2] < 0 or line[2] > 0xFFFFFFFF or not line[2].is_integer():
        raise ValueError("a G-code Reader state or event needs an integral line number at target zero")
    reader_id = next(
        (item for item in copied if item[0] == int(RAW_GCODE_READER_ITEM_READER_ID)),
        None,
    )
    if reader_id is not None and (
        reader_id[1] != 0
        or reader_id[2] < 0
        or reader_id[2] > 0xFFFFFFFF
        or not reader_id[2].is_integer()
    ):
        raise ValueError("a G-code Reader identity must be an integral uint32 at target zero")
    marker = next(
        (item for item in copied if item[0] == int(RAW_GCODE_READER_ITEM_MARKER)),
        None,
    )
    if marker is not None and (
        marker[1] != 0
        or marker[2] < int(RAW_GCODE_READER_MARKER_SEAM)
        or marker[2] > int(RAW_GCODE_READER_MARKER_CUSTOM_GCODE)
        or not marker[2].is_integer()
    ):
        raise ValueError("a G-code Reader marker must be a declared marker type at target zero")
    if event_kind is None and marker is not None:
        raise ValueError("a complete G-code Reader state cannot contain an event-local marker")
    if event_kind == int(RAW_GCODE_READER_EVENT_MARKER) and marker is None:
        raise ValueError("a G-code Reader marker event needs a marker type")
    if (
        event_kind is not None
        and event_kind != int(RAW_GCODE_READER_EVENT_MARKER)
        and marker is not None
    ):
        raise ValueError("a non-marker G-code Reader event cannot contain a marker type")
    diagnostic = next(
        (item for item in copied if item[0] == int(RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE)),
        None,
    )
    if diagnostic is not None and (
        diagnostic[1] != 0
        or diagnostic[2] < int(RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND)
        or diagnostic[2] > int(RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT)
        or not diagnostic[2].is_integer()
    ):
        raise ValueError("a G-code Reader diagnostic must be a declared diagnostic type at target zero")
    if event_kind is None and diagnostic is not None:
        raise ValueError("a complete G-code Reader state cannot contain an event-local diagnostic")
    if event_kind == int(RAW_GCODE_READER_EVENT_DIAGNOSTIC) and diagnostic is None:
        raise ValueError("a G-code Reader diagnostic event needs a diagnostic type")
    if (
        event_kind is not None
        and event_kind != int(RAW_GCODE_READER_EVENT_DIAGNOSTIC)
        and diagnostic is not None
    ):
        raise ValueError("a non-diagnostic G-code Reader event cannot contain a diagnostic type")
    return tuple(copied)


@abi_requires_headers(
    "slic3r_gcode_reader.h",
    "slic3r_orchestrator.h",
    "steps/slic3r_step_gcode_reader.h",
)
class GCodeReaderService:
    """Execute an exact active Reader through the generic orchestrator API.

    ``api`` and ``print_view`` are borrowed. Calls on one helper are serialized;
    all chunks, events and final-state pointers are consumed before ``read``
    returns. An empty Reader id explicitly selects ``gcode.reader.default`` and
    never performs automatic detection.
    """

    def __init__(self, api, print_view=None) -> None:
        self.api = api
        self.print_view = print_view
        if print_view is not None and getattr(print_view, "api", api) is not api:
            raise ValueError("print must use this API instance")
        self._orchestrator = OrchestratorView(api)
        self._lock = threading.Lock()

    def validated_reader_id(self, reader_plugin_id: str) -> str:
        """Return the exact usable Reader id, applying only the default-id rule."""
        selected = reader_plugin_id or DEFAULT_GCODE_READER_PLUGIN_ID
        plugin = self._orchestrator.find_plugin(selected)
        if not plugin:
            raise RuntimeError(
                f"G-code reader plugin {selected!r} is unavailable in the host activation set"
            )
        if plugin.step() != int(GCODE_READER):
            raise RuntimeError(f"plugin {selected!r} is not a G-code reader")
        return selected

    def read(self, reader_plugin_id: str, initial_items, source):
        """Consume text or a chunk source and return copied state triples.

        ``source`` may be one ``str``/``bytes`` value, an iterable of such
        chunks, or a callable returning the next chunk. An empty chunk marks
        end-of-stream. Each chunk is copied to immutable bytes and retained
        until the Reader asks for the next one, so the native pointer remains
        valid for the complete callback interval.
        """
        with self._lock:
            selected = self.validated_reader_id(reader_plugin_id)
            plugin = self._orchestrator.find_plugin(selected)
            if not plugin:
                raise RuntimeError(
                    f"G-code reader plugin {selected!r} left the activation set before execution"
                )

            if isinstance(source, (str, bytes, bytearray, memoryview)):
                pending = iter((source,))
                pull = lambda: next(pending, b"")
            elif callable(source):
                pull = source
            else:
                pending = iter(source)
                pull = lambda: next(pending, b"")

            initial_values = [RawGcodeReaderEventItem(*item) for item in initial_items]
            initial_array = (RawGcodeReaderEventItem * len(initial_values))(*initial_values)
            initial = RawGcodeReaderState(
                initial_array if initial_values else None, len(initial_values)
            )
            _copied_items(initial)
            invocation = {
                "end_reached": False,
                "final_items": None,
                "error": None,
                "active_chunk": b"",
            }

            def fail(result_pointer, status, message):
                invocation["error"] = str(message).encode("utf-8", errors="replace")
                if result_pointer:
                    result_pointer.contents.status = status
                    result_pointer.contents.error_message = invocation["error"]

            def next_chunk(_context, chunk_pointer, result_pointer):
                try:
                    chunk_pointer.contents.bytes = None
                    chunk_pointer.contents.byte_count = 0
                    if invocation["end_reached"]:
                        result_pointer.contents.status = RAW_GCODE_READER_STATUS_SUCCESS
                        return
                    raw_chunk = pull()
                    if isinstance(raw_chunk, str):
                        raw_chunk = raw_chunk.encode("utf-8")
                    elif raw_chunk is None:
                        raw_chunk = b""
                    elif not isinstance(raw_chunk, bytes):
                        raw_chunk = bytes(raw_chunk)
                    invocation["active_chunk"] = raw_chunk
                    if not raw_chunk:
                        invocation["end_reached"] = True
                    else:
                        chunk_pointer.contents.bytes = raw_chunk
                        chunk_pointer.contents.byte_count = len(raw_chunk)
                    result_pointer.contents.status = RAW_GCODE_READER_STATUS_SUCCESS
                    result_pointer.contents.error_message = None
                except BaseException as exc:
                    fail(result_pointer, RAW_GCODE_READER_STATUS_ERROR, exc)

            def emit_event(_context, event_pointer, result_pointer):
                try:
                    if invocation["final_items"] is not None:
                        raise ValueError("the Reader emitted an event after its final state")
                    event = event_pointer.contents
                    kind = int(event.kind)
                    if kind < int(RAW_GCODE_READER_EVENT_STATE_UPDATE) or kind > int(RAW_GCODE_READER_EVENT_INVALIDATE):
                        raise ValueError("the Reader emitted an unknown event kind")
                    _copied_items(event, kind)
                    result_pointer.contents.status = RAW_GCODE_READER_STATUS_SUCCESS
                    result_pointer.contents.error_message = None
                except BaseException as exc:
                    fail(result_pointer, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, exc)

            def emit_final_state(_context, state_pointer, result_pointer):
                try:
                    if not invocation["end_reached"]:
                        raise ValueError("the Reader published its final state before end-of-stream")
                    if invocation["final_items"] is not None:
                        raise ValueError("the Reader published more than one final state")
                    invocation["final_items"] = _copied_items(state_pointer.contents)
                    result_pointer.contents.status = RAW_GCODE_READER_STATUS_SUCCESS
                    result_pointer.contents.error_message = None
                except BaseException as exc:
                    fail(result_pointer, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, exc)

            next_callback = GCODE_READER_GET_NEXT_CHUNK(next_chunk)
            event_callback = GCODE_READER_EMIT_EVENT(emit_event)
            final_callback = GCODE_READER_EMIT_FINAL_STATE(emit_final_state)
            # Zero-initialization leaves the operation-specific probe callback
            # null without asking ctypes to convert None into a function type.
            payload = RunCtxGcodeReader()
            payload.operation = RAW_GCODE_READER_OPERATION_READ
            payload.print = (
                self.print_view.c_handle() if self.print_view is not None else None
            )
            payload.initial_state = ctypes.pointer(initial)
            payload.callback_context = None
            payload.get_next_chunk = next_callback
            payload.emit_event = event_callback
            payload.emit_final_state = final_callback
            status = self._orchestrator.execute_one(
                plugin, payload, self.print_view
            )
            # Callback rejection is terminal even if a malformed provider
            # ignores the result and reports an overall success status.
            if invocation["error"]:
                raise RuntimeError(
                    invocation["error"].decode("utf-8", errors="replace")
                )
            if status != int(RAW_PLUGIN_EXECUTION_SUCCESS):
                descriptions = {
                    int(RAW_PLUGIN_EXECUTION_CANCELLED): "was cancelled",
                    int(RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT): "received an invalid execution request",
                    int(RAW_PLUGIN_EXECUTION_INACTIVE): "is no longer active",
                    int(RAW_PLUGIN_EXECUTION_PLUGIN_ERROR): "failed",
                }
                raise RuntimeError(
                    f"G-code reader plugin {selected!r} "
                    f"{descriptions.get(status, 'returned an unknown execution status')}"
                )
            if invocation["final_items"] is None:
                raise RuntimeError("G-code Reader returned without a final state")
            return invocation["final_items"]


__all__ = ["GCodeReaderService"]
