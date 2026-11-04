#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""This module lets Python code find plugins, describe selection settings and execute jobs.

The host is the slicer application. Orchestrator manages its plugins, objects
that perform slicing jobs. Construct OrchestratorView with the API object, then
find_plugin() obtains an enabled PluginView. Both references borrow application
objects and must not outlive the manager.

An exclusive group contains alternative plugins for one processing stage. Its
selector is a setting naming a member. register_exclusive_group() copies a
description while the member's package directory is current during loading.
register_selector_auto_choice() copies a setting key/value condition and
preferred member. The application retains these strings after the call.

execute() accepts per-call payloads, data whose interpretation belongs to
the plugin, and keeps them alive until the C call returns. It calls setup once,
prepares individual calls with setup_run, waits, then calls run. Preparations
and runs within one execution may be parallel. Execute a PluginView only through
the OrchestratorView that returned it. Executions are synchronous; concurrent
or recursive executions of the same plugin instance are not supported. Start
executions sequentially and use the parallelism within their callbacks.
"""

from __future__ import annotations

import ctypes
from collections.abc import Iterable

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    ConstStrings,
    ORCHESTRATOR_RUNTIME_ID_INVALID,
    RAW_CO_NONE,
    RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT,
    RawPluginRunPayload,
    RawExclusiveGroupDef,
    RAW_CONTAINER_TYPE_PROJECT,
    RAW_OPTION_CATEGORY_SLICING,
    RAW_PRESET_TYPE_FFF_PRINT,
    STEP_NONE,
)
from slic3r_bridge_detector_views import BridgeDetector, create_bridge_detector
from slic3r_config_def_views import (
    ConfigOptionDefinition,
    ConfigOptionKey,
    option_definition,
    option_keys,
)


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    if hasattr(handle, "address"):
        return int(handle.address)
    return int(handle)


def _payload_address(payload) -> int:
    """Resolve one opaque payload address without copying its native object."""
    if payload is None:
        return 0
    if isinstance(payload, (int, ctypes.c_void_p)):
        return _address(payload)
    # ctypes.addressof(pointer) addresses the Python pointer container, not the
    # native object it points to. Resolve typed pointers first so null remains
    # null and non-null pointers expose their pointee address to the plugin.
    if isinstance(payload, ctypes._Pointer):
        return int(ctypes.cast(payload, ctypes.c_void_p).value or 0)
    try:
        return ctypes.addressof(payload)
    except TypeError:
        pass
    try:
        return int(ctypes.cast(payload, ctypes.c_void_p).value or 0)
    except (TypeError, ValueError, ctypes.ArgumentError) as exc:
        raise TypeError(
            "payloads must be ctypes structures, arrays, pointers, addresses, or None"
        ) from exc


@abi_requires_headers("slic3r_orchestrator.h")
class PluginView:
    """Borrowed identity of one plugin registered in an orchestrator."""

    def __init__(self, api, orchestrator_handle, plugin_handle=0) -> None:
        self.api = api
        self.orchestrator_address = _address(orchestrator_handle)
        self.address = _address(plugin_handle)

    def valid(self) -> bool:
        return self.address != 0

    def c_handle(self) -> ctypes.c_void_p:
        return ctypes.c_void_p(self.address)

    def id(self) -> str:
        """Copy the stable plugin id, or return an empty string when invalid."""
        if not self.valid():
            return ""
        value = self.api.host.orchestrator_plugin_id(self.c_handle())
        return "" if not value else value.decode("utf-8", errors="replace")

    def step(self) -> int:
        """Return the registered step, or STEP_NONE for an invalid view."""
        if not self.valid():
            return int(STEP_NONE)
        return int(self.api.host.orchestrator_plugin_step(self.c_handle()))

    def __bool__(self) -> bool:
        return self.valid()


@abi_requires_headers("slic3r_orchestrator.h")
class OrchestratorView:
    """Borrowed orchestrator used to discover and execute plugin services."""

    def __init__(self, api, handle=None) -> None:
        self.api = api
        selected = api.orchestrator if handle is None else handle
        self.address = _address(selected)
        if not self.address:
            raise ValueError("orchestrator handle must not be null")

    def c_handle(self) -> ctypes.c_void_p:
        return ctypes.c_void_p(self.address)

    def register_activation_group(self, group_id: str, members: Iterable[str]) -> None:
        """Accumulate one solidarity declaration; strings are copied by the host."""
        names = [member.encode("utf-8") for member in members]
        array = (ctypes.c_char_p * len(names))(*names)
        values = ConstStrings(
            ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)), len(names)
        )
        if not self.api.host.orchestrator_register_activation_group(
            self.c_handle(), group_id.encode("utf-8"), values
        ):
            raise ValueError(
                f"cannot register activation group {group_id!r}: expected a "
                "nonempty id and at least two distinct nonempty members"
            )

    def register_step(self, namespaced_name: str, invalidates_step: int) -> int:
        """Register a runtime service step and return its orchestrator-local id."""
        step = int(
            self.api.host.orchestrator_register_step(
                self.c_handle(), namespaced_name.encode("utf-8"), int(invalidates_step)
            )
        )
        if step == STEP_NONE:
            raise RuntimeError(
                f"dynamic step {namespaced_name!r} could not be registered"
            )
        return step

    @abi_requires_headers("slic3r_config_def.h")
    def register_exclusive_group(self, step, *, exclusive_group, declaring_plugin_id, label="", tooltip="",
                                 container_type=RAW_CONTAINER_TYPE_PROJECT, category=RAW_OPTION_CATEGORY_SLICING,
                                 option_preset_type=RAW_PRESET_TYPE_FFF_PRINT, ui_target_file="print.ui",
                                 ui_page="", ui_group="", option_key="", invalidates_step=None,
                                 strict_selection=False, read_with_one_provider=False, show_with_one_provider=False,
                                 provider_settings_rules=True, auto_value=False, value_when_absent=None):
        """Copy a group description naming its registered member; return False for a duplicate and raise ValueError on rejection."""
        definition = RawExclusiveGroupDef()
        definition.struct_size = ctypes.sizeof(RawExclusiveGroupDef)
        encoded = {key: value.encode("utf-8") for key, value in (
            ("exclusive_group", exclusive_group), ("declaring_plugin_id", declaring_plugin_id),
            ("label", label), ("tooltip", tooltip), ("ui_target_file", ui_target_file or "print.ui"),
            ("ui_page", ui_page), ("ui_group", ui_group), ("option_key", option_key))}
        for key, value in encoded.items():
            setattr(definition, key, value)
        definition.container_type = int(container_type)
        definition.category = int(category)
        definition.option_preset_type = int(option_preset_type)
        definition.has_invalidates_step = int(invalidates_step is not None)
        definition.invalidates_step = STEP_NONE if invalidates_step is None else int(invalidates_step)
        definition.strict_selection = int(bool(strict_selection))
        definition.read_with_one_provider = int(bool(read_with_one_provider))
        definition.show_with_one_provider = int(bool(show_with_one_provider))
        definition.provider_settings_rules = int(bool(provider_settings_rules))
        definition.auto_value = int(bool(auto_value))
        absent = None if value_when_absent is None else value_when_absent.encode("utf-8")
        definition.has_value_when_absent = int(absent is not None)
        definition.value_when_absent = absent
        result = int(self.api.host.orchestrator_register_exclusive_group(self.c_handle(), int(step), ctypes.byref(definition)))
        if result < 0:
            raise ValueError("Cannot register step selector traits")
        return result == 1

    @abi_requires_headers("slic3r_config_def.h")
    def register_selector_auto_choice(self, step, exclusive_group, condition_key, condition_value, provider_id):
        """Copy a setting condition and preferred member; return False for a duplicate and raise ValueError on rejection."""
        encoded = [value.encode("utf-8") for value in (exclusive_group, condition_key, condition_value, provider_id)]
        result = int(self.api.host.orchestrator_register_selector_auto_choice(self.c_handle(), int(step), *encoded))
        if result < 0:
            raise ValueError("Cannot register selector automatic choice")
        return result == 1

    def step_name(self, step: int) -> str:
        """Copy a dynamic step name; built-in and unknown steps return empty."""
        value = self.api.host.orchestrator_step_name(self.c_handle(), int(step))
        return "" if not value else value.decode("utf-8", errors="replace")

    def register_runtime_id(self, id_type: str, namespaced_name: str) -> int:
        """Intern a stable name and return its orchestrator-local numeric id.

        Persist ``id_type`` and ``namespaced_name``, never the returned number.
        Separate types own independent numeric namespaces.
        """
        value = int(
            self.api.host.orchestrator_register_runtime_id(
                self.c_handle(), id_type.encode("utf-8"), namespaced_name.encode("utf-8")
            )
        )
        if value == int(ORCHESTRATOR_RUNTIME_ID_INVALID):
            raise ValueError(
                "runtime id registration failed: type and name must be nonempty "
                "and the registry must have free dynamic ids"
            )
        return value

    def runtime_id_name(self, id_type: str, runtime_id: int) -> str:
        """Copy the registered name, or return an empty string when unknown."""
        value = self.api.host.orchestrator_runtime_id_name(
            self.c_handle(), id_type.encode("utf-8"), int(runtime_id)
        )
        return "" if not value else value.decode("utf-8", errors="replace")

    @abi_requires_headers("slic3r_config_def.h")
    def option_keys(self, type_filter: int = RAW_CO_NONE) -> tuple[ConfigOptionKey, ...]:
        """Copy global option keys in stable order, optionally filtered by type."""
        return option_keys(self.api, type_filter)

    @abi_requires_headers("slic3r_config_def.h")
    def option_definition(self, key: str) -> ConfigOptionDefinition | None:
        """Copy a complete normalized definition, or return None when absent."""
        return option_definition(self.api, key)

    def find_plugin(self, plugin_id: str) -> PluginView:
        """Find an active plugin by exact id.

        An empty or unknown id, or a plugin outside the activation set
        established by the host, produces a false ``PluginView``. ``execute()``
        defensively validates the handle and its activation again.
        """
        handle = self.api.host.orchestrator_find_plugin(
            self.c_handle(), plugin_id.encode("utf-8")
        )
        return PluginView(self.api, self.address, handle)

    def select_plugin(self, config, step: int, exclusive_group: str) -> PluginView:
        """Select the active provider configured for one exclusive group.

        ``config`` may be None. In that case, or when no selector exists, the
        host falls back to the active provider with normal priority ordering.
        """
        if config is not None and getattr(config, "api", self.api) is not self.api:
            raise ValueError("config must use this API instance")
        handle = self.api.host.orchestrator_select_plugin(
            self.c_handle(),
            ctypes.c_void_p(_address(config)),
            int(step),
            exclusive_group.encode("utf-8"),
        )
        return PluginView(self.api, self.address, handle)

    def execute(self, plugin: PluginView, payloads=(), print=None) -> int:
        """Execute a plugin with zero, one, or many opaque payload objects.

        A scalar ctypes object, pointer, integer address, or None represents one
        payload. Other iterables represent multiple payloads. The call blocks
        until both parallel phases complete, so all supplied Python objects stay
        alive for every native callback. The raw execution status is returned
        unchanged, allowing callers to distinguish cancellation, inactivity,
        invalid arguments, and plugin errors.
        """
        if not isinstance(plugin, PluginView):
            raise TypeError("plugin must be a PluginView")
        if plugin.api is not self.api or plugin.orchestrator_address != self.address:
            raise ValueError("plugin belongs to a different orchestrator")
        if print is not None and getattr(print, "api", self.api) is not self.api:
            raise ValueError("print must use this API instance")

        if payloads is None or isinstance(
            payloads, (int, ctypes.c_void_p, ctypes.Structure, ctypes.Array)
        ) or not isinstance(payloads, Iterable):
            values = [payloads]
        else:
            values = list(payloads)
        if len(values) > 0xFFFFFFFF:
            return int(RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT)

        raw_type = RawPluginRunPayload * len(values)
        raw_payloads = raw_type(
            *(RawPluginRunPayload(ctypes.c_void_p(_payload_address(value))) for value in values)
        )
        raw_pointer = None if not values else raw_payloads
        return int(
            self.api.host.orchestrator_execute_plugin(
                self.c_handle(),
                plugin.c_handle(),
                ctypes.c_void_p(_address(print)),
                raw_pointer,
                len(values),
            )
        )

    def execute_one(self, plugin: PluginView, payload, print=None) -> int:
        """Execute exactly one payload, including a deliberate null payload."""
        return self.execute(plugin, [payload], print)

    def execute_empty(self, plugin: PluginView, print=None) -> int:
        """Run only plugin setup with no per-run payload callbacks."""
        return self.execute(plugin, (), print)

    @abi_requires_headers("slic3r_bridge_detector.h", "slic3r_orchestrator.h")
    def create_bridge_detector(self, **kwargs) -> BridgeDetector:
        """Create an owning bridge-detector instance through this orchestrator."""
        return create_bridge_detector(self.api, self.c_handle(), **kwargs)


__all__ = ["OrchestratorView", "PluginView"]
