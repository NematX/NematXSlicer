#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Provider and consumer adapters for the ``GCODE_FIRMWARE`` service.

A provider creates one :class:`GCodeFirmwareSession` per export and publishes
it from ``run()`` through :class:`GCodeFirmwareContext`. The raw instance then
belongs to the host. Its callbacks receive borrowed Print, PrintingPlan and
ExtrusionEntity views and return Python text that is encoded as UTF-8.

Each published session owns persistent output and diagnostic buffers. A new
callback replaces both buffers, exactly matching the C rule that pointers stay
valid until the next callback on that session or its destruction. Firmware
callbacks are sequential and non-reentrant; the adapter serializes accidental
concurrent entries but provider code must still follow the documented nesting.

The consumer :class:`GCodeFirmwareView` borrows an instance and copies every
result before returning. It never destroys the instance. The owner that
received ``raw_gcode_firmware_instance`` must call its destroy callback once.

To implement a provider, derive a GCodeFirmwareSession and publish a new instance
through GCodeFirmwareContext for each export. A consumer borrows that instance
through GCodeFirmwareView, enters the documented boundaries in nesting order,
and writes each copied result before advancing. The owner of the raw instance
destroys it once after the writer finishes; a borrowed view never destroys it.
"""

from __future__ import annotations

import ctypes
import threading
import weakref

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    EXTRUDER_ID_UNKNOWN,
    GCODE_FIRMWARE,
    GCODE_FIRMWARE_BEGIN_GROUP,
    GCODE_FIRMWARE_BEGIN_LAYER,
    GCODE_FIRMWARE_BEGIN_PRINT,
    GCODE_FIRMWARE_BEGIN_TOOL_GROUP,
    GCODE_FIRMWARE_DESTROY,
    GCODE_FIRMWARE_END_SCOPE,
    GCODE_FIRMWARE_READER_PLUGIN_ID,
    GCODE_FIRMWARE_WRITE_EVENT,
    GCODE_FIRMWARE_WRITE_EXTRUSION,
    GCODE_SCRIPT_TYPE_INVALID,
    PluginRunContext,
    RAW_GCODE_FIRMWARE_STATUS_ERROR,
    RAW_GCODE_FIRMWARE_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_FIRMWARE_STATUS_SUCCESS,
    RAW_GCODE_FIRMWARE_STATUS_UNSET,
    RAW_GCODE_SCRIPT_STATUS_SUCCESS,
    RAW_GCODE_THUMBNAIL_STATUS_ERROR,
    RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_THUMBNAIL_STATUS_SUCCESS,
    RAW_GCODE_THUMBNAIL_STATUS_UNSET,
    RawGcodeFirmwareInstance,
    RawGcodeFirmwareResult,
    RawGcodeThumbnailRenderRequest,
    RawGcodeScriptProcessor,
    RawGcodeScriptResult,
    RunCtxGcodeFirmware,
    SLIC3R_CONFIG_OPTION_BOOL,
    SLIC3R_CONFIG_OPTION_FLOAT,
    SLIC3R_CONFIG_OPTION_FLOATS,
    SLIC3R_CONFIG_OPTION_INT,
    SLIC3R_CONFIG_OPTION_INTS,
    SLIC3R_CONFIG_OPTION_STRING,
)
from slic3r_datatree_views import Config, MutableConfig, Print
from slic3r_extrusion_views import ExtrusionEntity
from slic3r_gcode_reader_service import GCodeReaderService
from slic3r_printing_plan_views import (
    PrintingPlan,
    PrintingExtrusion,
    PrintingGroup,
    PrintingLayerGroup,
    PrintingToolGroup,
)


def _address(handle) -> int:
    """Return one borrowed pointer value without changing ownership."""
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    if hasattr(handle, "address"):
        return int(handle.address)
    try:
        return int(handle)
    except (TypeError, ValueError):
        return int(ctypes.cast(handle, ctypes.c_void_p).value or 0)


def _raw_pointer(structure, field_name: str) -> int:
    """Read a char pointer field without ctypes converting it to truncated bytes."""
    field = getattr(type(structure), field_name)
    return int(
        ctypes.c_void_p.from_address(ctypes.addressof(structure) + field.offset).value
        or 0
    )


@abi_requires_headers("slic3r_gcode_firmware.h")
class RawGcodeFirmwareVTable(ctypes.Structure):
    """ctypes layout of ``raw_gcode_firmware_vtable``."""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("destroy", GCODE_FIRMWARE_DESTROY),
        ("begin_print", GCODE_FIRMWARE_BEGIN_PRINT),
        ("begin_group", GCODE_FIRMWARE_BEGIN_GROUP),
        ("begin_layer", GCODE_FIRMWARE_BEGIN_LAYER),
        ("begin_tool_group", GCODE_FIRMWARE_BEGIN_TOOL_GROUP),
        ("write_extrusion", GCODE_FIRMWARE_WRITE_EXTRUSION),
        ("write_event", GCODE_FIRMWARE_WRITE_EVENT),
        ("end_tool_group", GCODE_FIRMWARE_END_SCOPE),
        ("end_layer", GCODE_FIRMWARE_END_SCOPE),
        ("end_group", GCODE_FIRMWARE_END_SCOPE),
        ("end_print", GCODE_FIRMWARE_END_SCOPE),
        ("reader_plugin_id", GCODE_FIRMWARE_READER_PLUGIN_ID),
    ]


@abi_requires_headers("slic3r_gcode_script.h", "slic3r_config.h", "slic3r_config_option.h")
class GCodeScriptConfig:
    """Restricted mutable Config prepared by the host for one script.

    Options already exist with fixed runtime types and vector sizes. This view
    cannot create, remove or resize them. It and every option borrowed from it
    become invalid when the same processor is prepared again.
    """

    def __init__(self, context: "GCodeScriptContext", handle) -> None:
        self._context = context
        self._config = MutableConfig(context.api, handle)

    def _option(self, key: str, expected_type: int, *, mutable: bool = False):
        self._context._validate()
        option = (
            self._config.get_mutable(key) if mutable else self._config.get(key)
        )
        if option is None:
            raise ValueError(f"unknown G-code script option: {key}")
        if option.type() != int(expected_type):
            raise TypeError(f"wrong type for G-code script option: {key}")
        return option

    def has(self, key: str) -> bool:
        self._context._validate()
        return self._config.has(key)

    def set_int(self, key: str, value: int) -> None:
        self._option(key, SLIC3R_CONFIG_OPTION_INT, mutable=True).set_int(value)

    def set_float(self, key: str, value: float) -> None:
        self._option(key, SLIC3R_CONFIG_OPTION_FLOAT, mutable=True).set_float(value)

    def set_bool(self, key: str, value: bool) -> None:
        self._option(key, SLIC3R_CONFIG_OPTION_BOOL, mutable=True).set_bool(value)

    def set_string(self, key: str, value: str) -> None:
        self._option(key, SLIC3R_CONFIG_OPTION_STRING, mutable=True).set_string(value)

    def set_floats(self, key: str, values) -> None:
        option = self._option(key, SLIC3R_CONFIG_OPTION_FLOATS, mutable=True)
        copied = [float(value) for value in values]
        if option.size() != len(copied):
            raise ValueError(f"wrong vector size for G-code script option: {key}")
        for index, value in enumerate(copied):
            option.set_float(value, index)

    def set_ints(self, key: str, values) -> None:
        option = self._option(key, SLIC3R_CONFIG_OPTION_INTS, mutable=True)
        copied = [int(value) for value in values]
        if option.size() != len(copied):
            raise ValueError(f"wrong vector size for G-code script option: {key}")
        for index, value in enumerate(copied):
            option.set_int(value, index)

    def get_int(self, key: str) -> int:
        return self._option(key, SLIC3R_CONFIG_OPTION_INT).get_int()

    def get_float(self, key: str) -> float:
        return self._option(key, SLIC3R_CONFIG_OPTION_FLOAT).get_float()

    def get_bool(self, key: str) -> bool:
        return self._option(key, SLIC3R_CONFIG_OPTION_BOOL).get_bool()

    def get_string(self, key: str) -> str:
        return self._option(key, SLIC3R_CONFIG_OPTION_STRING).get_string()

    def get_floats(self, key: str) -> list[float]:
        option = self._option(key, SLIC3R_CONFIG_OPTION_FLOATS)
        return [option.get_float(index) for index in range(option.size())]

    def get_ints(self, key: str) -> list[int]:
        option = self._option(key, SLIC3R_CONFIG_OPTION_INTS)
        return [option.get_int(index) for index in range(option.size())]


class _GCodeScriptProcessorState:
    """Invalidation state shared by every Python view of one native processor."""

    def __init__(self) -> None:
        self.generation = 0
        self.lock = threading.RLock()


_script_processor_states = weakref.WeakValueDictionary()
_script_processor_states_lock = threading.RLock()


def _script_processor_state(address: int) -> _GCodeScriptProcessorState:
    """Return shared state without extending the native processor's lifetime."""
    with _script_processor_states_lock:
        state = _script_processor_states.get(address)
        if state is None:
            state = _GCodeScriptProcessorState()
            _script_processor_states[address] = state
        return state


@abi_requires_headers("slic3r_gcode_script.h", "slic3r_config.h")
class GCodeScriptContext:
    """Short-lived interval between one processor ``prepare`` and ``process``."""

    def __init__(self, processor: "GCodeScriptProcessorView", config, generation: int):
        self.api = processor.api
        self._processor = processor
        self._config_address = _address(config)
        self._generation = generation

    def _validate(self) -> None:
        if self._generation != self._processor._state.generation:
            raise RuntimeError("the G-code script context was invalidated by prepare()")

    def config(self) -> GCodeScriptConfig:
        self._validate()
        return GCodeScriptConfig(self, self._config_address)

    def process(self, script: str, current_extruder: int) -> str:
        """Process one script and copy the host-owned UTF-8 result immediately."""
        if not isinstance(script, str):
            raise TypeError("script must be a string")
        if not isinstance(current_extruder, int):
            raise TypeError("current extruder must be an integer")
        if current_extruder < EXTRUDER_ID_UNKNOWN or current_extruder > 0x7FFF:
            raise ValueError(
                "current extruder must be UNKNOWN or fit in extruder_index_t"
            )
        with self._processor._state.lock:
            self._validate()
            result = RawGcodeScriptResult()
            result.struct_size = ctypes.sizeof(RawGcodeScriptResult)
            processor = self._processor._pointer.contents
            processor.process(
                processor.context,
                script.encode("utf-8"),
                int(current_extruder),
                ctypes.byref(result),
            )
            if int(result.status) != int(RAW_GCODE_SCRIPT_STATUS_SUCCESS):
                message = (
                    result.error_message.decode("utf-8", errors="replace")
                    if result.error_message
                    else "the G-code script processor failed"
                )
                raise RuntimeError(message)
            text_address = _raw_pointer(result, "text")
            if not text_address and int(result.text_size) != 0:
                raise RuntimeError("the G-code script processor returned invalid text")
            raw = ctypes.string_at(text_address, int(result.text_size)) if text_address else b""
        return raw.decode("utf-8")


@abi_requires_headers("slic3r_gcode_script.h", "slic3r_config.h")
class GCodeScriptProcessorView:
    """Borrow a host script processor that remains owned by the G-code step."""

    def __init__(self, api, processor=None) -> None:
        self.api = api
        self._pointer = (
            ctypes.POINTER(RawGcodeScriptProcessor)()
            if not _address(processor)
            else ctypes.cast(processor, ctypes.POINTER(RawGcodeScriptProcessor))
        )
        if self._pointer:
            value = self._pointer.contents
            if (
                int(value.struct_size) < ctypes.sizeof(RawGcodeScriptProcessor)
                or not value.context
                or not value.prepare
                or not value.process
            ):
                raise ValueError("the G-code script processor is incomplete")
        self._state = _script_processor_state(_address(self._pointer))

    def valid(self) -> bool:
        return bool(self._pointer)

    def prepare(
        self, script_type: int, producer_config: Config | None = None
    ) -> GCodeScriptContext:
        """Prepare one context; this invalidates every context returned earlier."""
        if not self.valid():
            raise RuntimeError("no G-code script processor is available")
        if int(script_type) == int(GCODE_SCRIPT_TYPE_INVALID):
            raise ValueError("a G-code script needs a valid type")
        if producer_config is not None:
            if not isinstance(producer_config, Config) or producer_config.api is not self.api:
                raise ValueError("producer_config must use this API instance")
        with self._state.lock:
            self._state.generation += 1
            generation = self._state.generation
            config = self._pointer.contents.prepare(
                self._pointer.contents.context,
                int(script_type),
                None if producer_config is None else producer_config.c_handle(),
            )
        if not _address(config):
            raise RuntimeError("the host could not prepare the G-code script context")
        return GCodeScriptContext(self, config, generation)


@abi_requires_headers("slic3r_gcode_firmware.h")
class GCodeFirmwareSession:
    """Base class for one stateful, sequential G-code export.

    Returned strings are appended verbatim, so providers supply line endings.
    Arguments are borrowed only for the duration of the callback and must not
    be freed or retained. Calls follow the documented begin/write/end nesting.
    """

    def reader_plugin_id(self) -> str:
        """Return the stable Reader id for this export; empty selects default."""
        return ""

    def begin_print(
        self,
        print_view: Print,
        plan: PrintingPlan,
    ) -> str:
        """Emit preamble and plan.before events once. Views are read-only and
        borrowed for this call; do not retain or free them. The writer will not
        issue a separate write_event call for the plan's before-events.
        """
        raise NotImplementedError

    def begin_group(self, group: PrintingGroup) -> str:
        raise NotImplementedError

    def begin_layer(self, layer_group: PrintingLayerGroup) -> str:
        raise NotImplementedError

    def begin_tool_group(self, tool_group: PrintingToolGroup) -> str:
        raise NotImplementedError

    def write_extrusion(self, extrusion: PrintingExtrusion) -> str:
        raise NotImplementedError

    def write_event(self, event_root: ExtrusionEntity) -> str:
        raise NotImplementedError

    def end_tool_group(self) -> str:
        raise NotImplementedError

    def end_layer(self) -> str:
        raise NotImplementedError

    def end_group(self) -> str:
        raise NotImplementedError

    def end_print(self) -> str:
        raise NotImplementedError


class _PublishedFirmwareSession:
    """Own one Python session and the C strings borrowed from its last call."""

    def __init__(self, api, session: GCodeFirmwareSession) -> None:
        self.api = api
        self.session = session
        self.callback_lock = threading.RLock()
        self.output = None
        self.error = None


_published_lock = threading.RLock()
_published_sessions: dict[int, _PublishedFirmwareSession] = {}


def _set_result_defaults(result: RawGcodeFirmwareResult) -> None:
    """Reset every result field before dispatching one provider callback."""
    result.status = RAW_GCODE_FIRMWARE_STATUS_UNSET
    result.text = None
    result.text_size = 0
    result.error_message = None


def _invoke_firmware_callback(session_address, result_pointer, operation) -> None:
    """Run one provider operation and contain all failures at the C boundary."""
    if not result_pointer:
        return
    try:
        result = result_pointer.contents
        if int(result.struct_size) < ctypes.sizeof(RawGcodeFirmwareResult):
            return
        _set_result_defaults(result)
        with _published_lock:
            published = _published_sessions.get(_address(session_address))
        if published is None:
            result.status = RAW_GCODE_FIRMWARE_STATUS_INVALID_ARGUMENT
            return

        with published.callback_lock:
            published.output = None
            published.error = None
            try:
                text = operation(published)
                if not isinstance(text, str):
                    raise TypeError("G-code firmware callbacks must return str")
                encoded = text.encode("utf-8")
                published.output = ctypes.create_string_buffer(encoded)
                result.text = ctypes.cast(published.output, ctypes.c_char_p)
                result.text_size = len(encoded)
                result.status = RAW_GCODE_FIRMWARE_STATUS_SUCCESS
            except BaseException as exc:
                message = str(exc) or exc.__class__.__name__
                published.error = ctypes.create_string_buffer(
                    message.encode("utf-8", errors="replace")
                )
                result.status = RAW_GCODE_FIRMWARE_STATUS_ERROR
                result.error_message = ctypes.cast(published.error, ctypes.c_char_p)
    except BaseException:
        return


def _require_callback_handle(handle, name: str) -> int:
    """Reject null borrowed callback arguments before constructing Python views."""
    value = _address(handle)
    if not value:
        raise ValueError(f"firmware {name} received a null handle")
    return value


def _destroy_published_session(session_address) -> None:
    """Destroy one host-owned Python session and its borrowed result buffers."""
    try:
        with _published_lock:
            published = _published_sessions.pop(_address(session_address), None)
        if published is not None:
            with published.callback_lock:
                published.output = None
                published.error = None
    except BaseException:
        return


def _begin_print(session, print_handle, plan_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.begin_print(
            Print(holder.api, _require_callback_handle(print_handle, "begin_print")),
            PrintingPlan(holder.api, _require_callback_handle(plan_handle, "begin_print plan")),
        ),
    )


def _begin_group(session, group_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.begin_group(
            PrintingGroup(holder.api, _require_callback_handle(group_handle, "begin_group"))
        ),
    )


def _begin_layer(session, layer_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.begin_layer(
            PrintingLayerGroup(holder.api, _require_callback_handle(layer_handle, "begin_layer"))
        ),
    )


def _begin_tool_group(session, tool_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.begin_tool_group(
            PrintingToolGroup(
                holder.api, _require_callback_handle(tool_handle, "begin_tool_group")
            )
        ),
    )


def _write_extrusion(session, extrusion_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.write_extrusion(
            PrintingExtrusion(
                holder.api, _require_callback_handle(extrusion_handle, "write_extrusion")
            )
        ),
    )


def _write_event(session, event_handle, result) -> None:
    _invoke_firmware_callback(
        session,
        result,
        lambda holder: holder.session.write_event(
            ExtrusionEntity(holder.api, _require_callback_handle(event_handle, "write_event"))
        ),
    )


def _end_tool_group(session, result) -> None:
    _invoke_firmware_callback(session, result, lambda holder: holder.session.end_tool_group())


def _end_layer(session, result) -> None:
    _invoke_firmware_callback(session, result, lambda holder: holder.session.end_layer())


def _end_group(session, result) -> None:
    _invoke_firmware_callback(session, result, lambda holder: holder.session.end_group())


def _end_print(session, result) -> None:
    _invoke_firmware_callback(session, result, lambda holder: holder.session.end_print())


def _reader_plugin_id(session, result) -> None:
    _invoke_firmware_callback(session, result, lambda holder: holder.session.reader_plugin_id())


_destroy_callback = GCODE_FIRMWARE_DESTROY(_destroy_published_session)
_begin_print_callback = GCODE_FIRMWARE_BEGIN_PRINT(_begin_print)
_begin_group_callback = GCODE_FIRMWARE_BEGIN_GROUP(_begin_group)
_begin_layer_callback = GCODE_FIRMWARE_BEGIN_LAYER(_begin_layer)
_begin_tool_group_callback = GCODE_FIRMWARE_BEGIN_TOOL_GROUP(_begin_tool_group)
_write_extrusion_callback = GCODE_FIRMWARE_WRITE_EXTRUSION(_write_extrusion)
_write_event_callback = GCODE_FIRMWARE_WRITE_EVENT(_write_event)
_end_tool_group_callback = GCODE_FIRMWARE_END_SCOPE(_end_tool_group)
_end_layer_callback = GCODE_FIRMWARE_END_SCOPE(_end_layer)
_end_group_callback = GCODE_FIRMWARE_END_SCOPE(_end_group)
_end_print_callback = GCODE_FIRMWARE_END_SCOPE(_end_print)
_reader_plugin_id_callback = GCODE_FIRMWARE_READER_PLUGIN_ID(_reader_plugin_id)
_session_vtable = RawGcodeFirmwareVTable(
    ctypes.sizeof(RawGcodeFirmwareVTable),
    _destroy_callback,
    _begin_print_callback,
    _begin_group_callback,
    _begin_layer_callback,
    _begin_tool_group_callback,
    _write_extrusion_callback,
    _write_event_callback,
    _end_tool_group_callback,
    _end_layer_callback,
    _end_group_callback,
    _end_print_callback,
    _reader_plugin_id_callback,
)
_session_vtable_pointer = ctypes.pointer(_session_vtable)


@abi_requires_headers("slic3r_gcode_firmware.h")
def make_gcode_firmware_instance(
    session: GCodeFirmwareSession, api=None
) -> RawGcodeFirmwareInstance:
    """Transfer a fresh Python firmware session into one host-owned C instance.

    ``api`` supplies the wrapper factory used for borrowed callback arguments.
    A provider context passes it automatically. Direct tests or adapters may
    instead expose the same object as ``session.api``.
    """
    if not isinstance(session, GCodeFirmwareSession):
        raise TypeError("session must derive from GCodeFirmwareSession")
    session_api = api if api is not None else getattr(session, "api", None)
    if session_api is None:
        raise ValueError("a G-code firmware instance needs a Slic3rAPI")
    with _published_lock:
        if getattr(session, "_slic3r_gcode_firmware_consumed", False):
            raise ValueError("the G-code firmware session has already been consumed")
        session._slic3r_gcode_firmware_consumed = True
        published = _PublishedFirmwareSession(session_api, session)
        key = id(published)
        _published_sessions[key] = published
    return RawGcodeFirmwareInstance(
        ctypes.sizeof(RawGcodeFirmwareInstance),
        ctypes.c_void_p(key),
        ctypes.cast(_session_vtable_pointer, ctypes.c_void_p),
    )


def _instance_vtable(instance_pointer):
    """Validate a borrowed instance and return its complete callback table."""
    if not instance_pointer:
        return None
    instance = instance_pointer.contents
    if (
        int(instance.struct_size) < ctypes.sizeof(RawGcodeFirmwareInstance)
        or not instance.session
        or not instance.vtable
    ):
        return None
    vtable = ctypes.cast(instance.vtable, ctypes.POINTER(RawGcodeFirmwareVTable))
    if int(vtable.contents.struct_size) < ctypes.sizeof(RawGcodeFirmwareVTable):
        return None
    callbacks = (
        vtable.contents.destroy,
        vtable.contents.begin_print,
        vtable.contents.begin_group,
        vtable.contents.begin_layer,
        vtable.contents.begin_tool_group,
        vtable.contents.write_extrusion,
        vtable.contents.write_event,
        vtable.contents.end_tool_group,
        vtable.contents.end_layer,
        vtable.contents.end_group,
        vtable.contents.end_print,
        vtable.contents.reader_plugin_id,
    )
    return vtable if all(callbacks) else None


@abi_requires_headers("slic3r_gcode_firmware.h")
class GCodeFirmwareView:
    """Borrow and invoke a complete firmware instance without owning it.

    Every method copies and decodes the returned UTF-8 bytes before returning.
    The raw instance and all argument handles must outlive the call. This view
    must not invoke ``destroy``; ownership remains with the G-code host.
    """

    def __init__(self, api, instance) -> None:
        self.api = api
        self._instance_owner = instance if isinstance(instance, RawGcodeFirmwareInstance) else None
        self._instance = (
            ctypes.pointer(instance)
            if isinstance(instance, RawGcodeFirmwareInstance)
            else ctypes.cast(instance, ctypes.POINTER(RawGcodeFirmwareInstance))
        )
        if _instance_vtable(self._instance) is None:
            raise ValueError("the G-code firmware instance is incomplete")

    def valid(self) -> bool:
        return _instance_vtable(self._instance) is not None

    def _invoke(self, callback, *arguments) -> str:
        result = RawGcodeFirmwareResult()
        result.struct_size = ctypes.sizeof(RawGcodeFirmwareResult)
        callback(self._instance.contents.session, *arguments, ctypes.byref(result))
        if int(result.status) != int(RAW_GCODE_FIRMWARE_STATUS_SUCCESS):
            message = (
                result.error_message.decode("utf-8", errors="replace")
                if result.error_message
                else "the G-code firmware callback failed"
            )
            raise RuntimeError(message)
        text_address = _raw_pointer(result, "text")
        if not text_address and int(result.text_size) != 0:
            raise RuntimeError("the G-code firmware returned invalid text")
        raw = ctypes.string_at(text_address, int(result.text_size)) if text_address else b""
        return raw.decode("utf-8")

    def _argument(self, value, expected_type, name: str):
        if not isinstance(value, expected_type) or value.api is not self.api:
            raise ValueError(f"{name} must use this API instance")
        return value.c_handle()

    def begin_print(
        self,
        print_view: Print,
        plan: PrintingPlan,
    ) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.begin_print,
            self._argument(print_view, Print, "print"),
            self._argument(plan, PrintingPlan, "plan"),
        )

    def begin_group(self, group: PrintingGroup) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.begin_group,
            self._argument(group, PrintingGroup, "group"),
        )

    def begin_layer(self, layer_group: PrintingLayerGroup) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.begin_layer,
            self._argument(layer_group, PrintingLayerGroup, "layer group"),
        )

    def begin_tool_group(self, tool_group: PrintingToolGroup) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.begin_tool_group,
            self._argument(tool_group, PrintingToolGroup, "tool group"),
        )

    def write_extrusion(self, extrusion: PrintingExtrusion) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.write_extrusion,
            self._argument(extrusion, PrintingExtrusion, "printing extrusion"),
        )

    def write_event(self, event_root: ExtrusionEntity) -> str:
        return self._invoke(
            _instance_vtable(self._instance).contents.write_event,
            self._argument(event_root, ExtrusionEntity, "event root"),
        )

    def _invoke_end(self, field_name: str) -> str:
        return self._invoke(getattr(_instance_vtable(self._instance).contents, field_name))

    def end_tool_group(self) -> str:
        return self._invoke_end("end_tool_group")

    def end_layer(self) -> str:
        return self._invoke_end("end_layer")

    def end_group(self) -> str:
        return self._invoke_end("end_group")

    def end_print(self) -> str:
        return self._invoke_end("end_print")

    def reader_plugin_id(self) -> str:
        """Copy the Reader id before another firmware callback replaces it."""
        return self._invoke(_instance_vtable(self._instance).contents.reader_plugin_id)


@abi_requires_headers("steps/slic3r_step_gcode_firmware.h")
class GCodeFirmwareContext:
    """Borrow the service payload used to publish one firmware session."""

    def __init__(self, api, common: PluginRunContext, payload: RunCtxGcodeFirmware) -> None:
        self.api = api
        self.common = common
        self.payload = payload

    @classmethod
    def from_gcode_firmware_run_context(cls, api, run_ctx_address: int):
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if int(common.step) != int(GCODE_FIRMWARE) or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxGcodeFirmware)).contents
        return cls(api, common, payload)

    def gcode_firmware_print(self) -> Print:
        """Borrow the Print whose configuration applies to this export."""
        if not self.payload.print:
            raise ValueError("the G-code firmware context has no Print")
        return Print(self.api, self.payload.print)

    def gcode_firmware_script_processor(self) -> GCodeScriptProcessorView:
        """Borrow the processor that remains valid for the published session."""
        return GCodeScriptProcessorView(self.api, self.payload.script_processor)

    @abi_requires_headers(
        "slic3r_gcode_reader.h",
        "slic3r_orchestrator.h",
        "steps/slic3r_step_gcode_reader.h",
    )
    def gcode_reader_service(self) -> GCodeReaderService:
        """Create a Reader client for this export's orchestrator and Print."""
        return GCodeReaderService(self.api, self.gcode_firmware_print())

    def publish_gcode_firmware(self, session: GCodeFirmwareSession) -> None:
        """Transfer one fresh provider session into the host-owned payload."""
        if self.payload.instance.session or self.payload.instance.vtable:
            raise ValueError("the G-code firmware payload already owns a session")
        if not isinstance(session, GCodeFirmwareSession):
            raise TypeError("session must derive from GCodeFirmwareSession")
        if hasattr(session, "api") and session.api is not self.api:
            raise ValueError("the G-code firmware session must use this API instance")
        self.payload.instance = make_gcode_firmware_instance(session, self.api)


__all__ = [
    "GCodeFirmwareContext",
    "GCodeFirmwareSession",
    "GCodeFirmwareView",
    "GCodeReaderService",
    "GCodeScriptConfig",
    "GCodeScriptContext",
    "GCodeScriptProcessorView",
    "RawGcodeFirmwareVTable",
    "make_gcode_firmware_instance",
]
