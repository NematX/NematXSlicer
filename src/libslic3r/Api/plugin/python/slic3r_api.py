#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Small Python helper layer for SuperSlicer plugins.

Python plugins are loaded by python_plugin_loader.dll. The loader imports both
the scripts bundled in plugins/python/python/plugins and Python-only sibling
packages installed in plugins/<package-id>. Every script is called through:

    register_plugin(api: Slic3rAPI)

The function may return one PluginBase instance, a list/tuple of PluginBase
instances, or None if the script only registers settings/UI fragments.

The C ABI vtable is built by the loader DLL, not by Python. This avoids fragile
callback signatures while keeping Python plugin code simple. Slic3rAPI binds the
public host C functions with ctypes; raw functions are available as
api.host.function_name(...), and a smaller set of Python-friendly helpers is
provided for common tasks. Low-level handles and run contexts remain available
as integer addresses, while data-tree, geometry, extrusion, and printing-plan
views make their ownership and mutation rules explicit.

Plugins should import this module from the Python runtime directory prepared by
the loader:

    from slic3r_api import PluginBase, STEP_POST_SLICING

Geometry helpers are re-exported from slic3r_geometry_views.py. They wrap raw
handles in Python objects while leaving the raw C ABI available:

    polygon = api.polygon(polygon_handle)
    owned_polyline = api.new_polyline(storage_handle)
    owned_polyline.push_back(make_point(0, 0))

Extrusion helpers follow the same pattern:

    entity = api.extrusion(entity_handle)
    owned_entity = api.new_extrusion(storage_handle)
    owned_entity.set_points([make_point(0, 0), make_point(1000000, 0)])
    thin_wall = api.medial_axis_thin_wall(flow).medial_widths(min_w, max_w).build(storage_handle, expolygon)

Clipper helpers wrap temporary Clipper operands bound to a storage_handle:

    clip = api.clipper(storage_handle)
    result = clip.diff(clip(subject_polygons), clip(mask_expolygons))
    result.write_expolygons_to(output_expolygons)

RegionSettings helpers let a plugin process a whole LayerIsland while still
splitting work where region/modifier settings differ:

    settings = RegionSettings(api, storage_handle, island, [["fuzzy_skin"]])
    settings.segregate(island.slice())
    for value, area_clip in settings.get_areas("fuzzy_skin"):
        selected = area_clip.intersections(candidate_areas)

Do not copy this file, or slic3r_api_generated.py, into each plugin. The loader
adds the shared Python runtime and the current package root to sys.path before
loading plugin scripts, so common helpers and package-local modules are both
available.

slic3r_api_generated.py is generated from Api/plugin/c headers by
generate_slic3r_api.py. It is copied next to this file in the Python runtime
root prepared by the loader. In a development build, build python_plugin_loader
to regenerate and copy the current definitions into that runtime directory:

    cmake --build build --config Debug --target python_plugin_loader -- /m:8

For a quick Python-only import test during development, use the generated
runtime directory:

    python -c "import sys; sys.path.insert(0, r'build/src/Debug/plugins/python'); import slic3r_api"
"""

from __future__ import annotations

import ctypes
import math
from collections.abc import Mapping
from typing import Iterable, Optional

try:
    from slic3r_api_generated import *
    from slic3r_api_generated import _bind_c_function
except ModuleNotFoundError as exc:
    if exc.name != "slic3r_api_generated":
        raise
    raise ModuleNotFoundError(
        "slic3r_api_generated.py is a build artifact generated from the C plugin ABI headers. "
        "Import slic3r_api from the Python runtime root prepared by python_plugin_loader "
        "(for example build/src/<Config>/plugins/python in a development build), or run "
        "generate_slic3r_api.py with an output path next to this file for local experiments."
    ) from exc

from slic3r_abi import abi_requires_headers
from slic3r_property_key import *
from slic3r_geometry_views import *
from slic3r_extrusion_views import *
from slic3r_surface_views import *
from slic3r_datatree_views import *
from slic3r_clipper_views import *
from slic3r_config_def_views import *
from slic3r_region_settings_views import *
from slic3r_printing_plan_views import *
from slic3r_print_helpers import *
from slic3r_bridge_detector_views import *
from slic3r_orchestrator_views import *
from slic3r_gui_rule_helpers import *
from slic3r_run_context import *
from steps.layer_height import *
from steps.slicing import *
from steps.post_slicing import *
from steps.perimeter import *
from steps.perimeter_module import *
from steps.seam_placer import *
from steps.gcode_firmware import *
from steps.gcode_thumbnail import *
from steps.post_perimeter import *
from steps.post_infill import *
from steps.surface_generation import *
from steps.execution import *


_registered_apis = []


def _as_bytes(text: Optional[str]) -> Optional[bytes]:
    return None if text is None else text.encode("utf-8")


def _gui_rule_clause(clause) -> "RawGuiRuleClause":
    """Normalize one GUI rule clause specification.

    A clause is either a RawGuiRuleClause, a mapping using the clause field
    names, or a sequence (condition, condition_key[, condition_index
    [, condition_float_value[, condition_string_value]]]). The numeric operand
    rounds for INT_* and is compared directly for FLOAT_* conditions. The string
    operand is compared with the serialized option value for STRING_EQUALS.
    Missing fields keep the native defaults.
    """
    if isinstance(clause, RawGuiRuleClause):
        return clause

    if isinstance(clause, Mapping):
        condition_value = clause.get("condition", RAW_GUI_RULE_CONDITION_BOOL_TRUE)
        condition_key = clause.get("condition_key")
        condition_index_value = clause.get("condition_index", RAW_GUI_RULE_INDEX_ALL)
        condition_float_value = clause.get("condition_float_value", 0.0)
        condition_string_value = clause.get("condition_string_value")
    else:
        try:
            fields = tuple(clause)
        except TypeError as exc:
            raise ValueError(
                "A GUI rule clause must be a RawGuiRuleClause, a mapping or a sequence."
            ) from exc
        if len(fields) < 2 or len(fields) > 5:
            raise ValueError(
                "A GUI rule clause sequence needs 2 to 5 items: "
                "condition, condition_key[, condition_index[, condition_float_value"
                "[, condition_string_value]]]."
            )
        condition_value = fields[0]
        condition_key = fields[1]
        condition_index_value = fields[2] if len(fields) > 2 else RAW_GUI_RULE_INDEX_ALL
        condition_float_value = fields[3] if len(fields) > 3 else 0.0
        condition_string_value = fields[4] if len(fields) > 4 else None

    if condition_key is None:
        raise ValueError("A GUI rule clause needs a condition_key.")

    return RawGuiRuleClause(
        int(condition_value),
        _as_bytes(str(condition_key)),
        int(condition_index_value),
        float(condition_float_value),
        _as_bytes(str(condition_string_value)) if condition_string_value is not None else None,
    )


def round_coord(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def scale_i(value: float) -> int:
    return round_coord(value * UNSCALING_FACTOR)


def unscaled(value: float) -> float:
    return float(value) * SCALING_FACTOR


@abi_requires_headers("slic3r_plugin_run_context.h")
def run_context(run_ctx_address: int) -> Optional[PluginRunContext]:
    if not run_ctx_address:
        return None
    return ctypes.cast(ctypes.c_void_p(run_ctx_address), ctypes.POINTER(PluginRunContext)).contents


@abi_requires_headers("steps/slic3r_step_post_slicing.h")
def raw_post_slicing_context(run_ctx_address: int) -> Optional[RunCtxPostSlicing]:
    ctx = run_context(run_ctx_address)
    if ctx is None or ctx.step != STEP_POST_SLICING or not ctx.data:
        return None
    return ctypes.cast(ctx.data, ctypes.POINTER(RunCtxPostSlicing)).contents


@abi_requires_headers("slic3r_plugin_run_context.h")
def is_cancelled(run_ctx_address: int) -> bool:
    ctx = run_context(run_ctx_address)
    if ctx is None or not ctx.is_cancelled:
        return False
    fn = PLUGIN_IS_CANCELLED(ctx.is_cancelled)
    return bool(fn(ctx.host_context))


@abi_requires_headers("slic3r_plugin_run_context.h")
def report_warning(run_ctx_address: int, message: str) -> None:
    ctx = run_context(run_ctx_address)
    if ctx is not None and ctx.report_warning:
        PLUGIN_REPORT(ctx.report_warning)(ctx.host_context, _as_bytes(message))


@abi_requires_headers("slic3r_plugin_run_context.h")
def report_error(run_ctx_address: int, message: str) -> None:
    ctx = run_context(run_ctx_address)
    if ctx is not None and ctx.report_error:
        PLUGIN_REPORT(ctx.report_error)(ctx.host_context, _as_bytes(message))


@abi_requires_headers("slic3r_plugin_run_context.h")
def report_progress(run_ctx_address: int, progress: float, message: Optional[str] = None) -> None:
    ctx = run_context(run_ctx_address)
    if ctx is not None and ctx.report_progress:
        PLUGIN_REPORT_PROGRESS(ctx.report_progress)(ctx.host_context, progress, _as_bytes(message))


@abi_requires_headers("slic3r_orchestrator.h")
def store_error_message(message: str) -> int:
    """Register one failure message and return its diagnostic id.

    Use it from PluginBase.clear() (and future status-returning callbacks)
    before returning the id to the host. The registered text stays valid for
    the lifetime of the application. Returns PLUGIN_ERROR_GENERAL when no API
    is registered or the host cannot store the message.
    """
    if not _registered_apis:
        return PLUGIN_ERROR_GENERAL
    api = _registered_apis[-1]
    return int(api.host.orchestrator_set_error_message(api.orchestrator, _as_bytes(message)))


def used_config_key(
    key: str,
    type: int,
    container_type: int = RAW_CONTAINER_TYPE_NONE,
    option_preset_type: int = RAW_PRESET_TYPE_NONE,
) -> tuple[str, int, int, int]:
    """
    Declare one setting read or defined by a Python plugin.

    The type is mandatory because the host validates that the setting provider
    and the reader agree on the ConfigOption representation. container_type and
    option_preset_type are optional filters; leave them at *_NONE when the key
    may legitimately come from several places. Every name in
    defined_config_keys must have a matching typed entry created here.
    """
    return (key, int(type), int(container_type), int(option_preset_type))


@abi_requires_headers("slic3r_plugin_types.h")
class PluginBase:
    """
    Base class for Python plugins.

    Set plugin_id, name, description, step, priority, dependencies and
    used_config_keys in __init__ by calling the base constructor. Entries in
    used_config_keys should be created with used_config_key(...), so the host
    can validate the expected option type. This list must also contain every
    key in defined_config_keys. When exclusive_group is not supplied, it
    defaults to plugin_id and behaves as a singleton group. If several plugins
    are alternatives for the same work, give them the same exclusive_group so
    the host can expose a selector and run only one of them. Addressable
    services that are selected explicitly by id may pass an empty string so all
    their providers can remain active together. Override initialize/setup/
    setup_run/run as needed.

    Callback arguments are raw C pointer addresses represented as Python int:
    - initialize(storage_address)
    - setup(run_ctx_address, run_count)
    - setup_run(run_ctx_address)
    - run(run_ctx_address)
    - clear(print_address, plugin_storage_address), when the plugin owns
      per-Print state that must be discarded before replay; return
      PLUGIN_ERROR_OK (or None) when the state was released, or a diagnostic id
      obtained from store_error_message() to report why it survived.

    The clear status must be a signed 32-bit int, not bool. Omit clear entirely
    when no per-Print cleanup is needed.

    Use run_context(), is_cancelled(), report_warning(), report_error(),
    report_progress() and store_error_message() to inspect or use the common
    callbacks.
    ``progress`` is reset before setup() and may then be shared by concurrent
    run() calls. Raise PluginCancelled (normally through throw_if_cancelled())
    to stop cooperatively without reporting a plugin failure.
    """

    def __init__(
        self,
        plugin_id: str,
        step: int,
        name: Optional[str] = None,
        description: str = "",
        priority: int = 0,
        dependencies: Iterable[str] = (),
        used_config_keys: Iterable[object] = (),
        defined_config_keys: Iterable[str] = (),
        exclusive_group: Optional[str] = None,
        exclusive_group_label: str = "",
        exclusive_group_tooltip: str = "",
    ) -> None:
        self.plugin_id = plugin_id
        self.name = name or plugin_id
        self.description = description
        self.exclusive_group = plugin_id if exclusive_group is None else exclusive_group
        self.exclusive_group_label = exclusive_group_label
        self.exclusive_group_tooltip = exclusive_group_tooltip
        self.step = int(step)
        self.priority = int(priority)
        self.dependencies = list(dependencies)
        self.used_config_keys = list(used_config_keys)
        self.defined_config_keys = list(defined_config_keys)
        self.progress = PluginProgress()

    def initialize(self, storage_address: int) -> None:
        pass

    def setup(self, run_ctx_address: int, run_count: int) -> None:
        pass

    def setup_run(self, run_ctx_address: int) -> None:
        pass

    def run(self, run_ctx_address: int) -> None:
        pass

    def progress_message_format(self) -> str:
        """Return the ``str.format`` template used by this plugin's progress counter."""
        return "Plugin progress: {done} / {max}"

    def _slic3r_setup(self, run_ctx_address: int, run_count: int) -> None:
        """Host bridge: reset progress, then contain user-code exceptions."""
        self._slic3r_call_safely(
            "setup", run_ctx_address, run_count, reset_progress=True
        )

    def _slic3r_setup_run(self, run_ctx_address: int) -> None:
        """Host bridge: run setup_run without letting Python exceptions cross C."""
        self._slic3r_call_safely("setup_run", run_ctx_address)

    def _slic3r_run(self, run_ctx_address: int) -> None:
        """Host bridge: run one job without letting Python exceptions cross C."""
        self._slic3r_call_safely("run", run_ctx_address)

    def _slic3r_call_safely(
        self,
        method_name: str,
        run_ctx_address: int,
        *args,
        reset_progress: bool = False,
    ) -> None:
        """Translate Python failures to the host error channel and cancellation to success."""
        try:
            if reset_progress:
                self.progress.reset(
                    run_ctx_address,
                    args[0],
                    self.progress_message_format(),
                )
            getattr(self, method_name)(run_ctx_address, *args)
        except PluginCancelled:
            return
        except BaseException as exc:
            context = run_context(run_ctx_address)
            if context is not None and context.report_error:
                PLUGIN_REPORT(context.report_error)(
                    context.host_context,
                    _as_bytes(f"Python plugin {method_name} failed: {exc}"),
                )


class Slic3rAPI:
    """
    Thin wrapper around exported host C functions.

    host_library_path is usually the Slic3r.dll path provided by the Python
    loader. The wrapper keeps the ctypes.CDLL object alive, so C calls remain
    valid after register_plugin() returns.
    """

    def __init__(self, orchestrator_address: int, host_library_path: str) -> None:
        self.orchestrator = ctypes.c_void_p(orchestrator_address)
        self.host = ctypes.CDLL(host_library_path)
        self._bind_host_functions()
        _registered_apis.append(self)

    def _bind_host_functions(self) -> None:
        missing = []
        for name, restype, argtypes in C_FUNCTION_SIGNATURES:
            if not _bind_c_function(self.host, name, restype, argtypes):
                missing.append(name)
        if missing:
            raise RuntimeError(
                "The host library is missing declared plugin API exports: "
                + ", ".join(missing)
            )

    @abi_requires_headers("slic3r_orchestrator.h")
    def orchestrator_view(self) -> OrchestratorView:
        """Borrow this API's orchestrator for service lookup and execution."""
        return OrchestratorView(self)

    @abi_requires_headers("slic3r_config_def.h")
    def option_keys(self, type_filter: int = RAW_CO_NONE) -> tuple[ConfigOptionKey, ...]:
        """Copy global option keys in stable order, optionally filtered by type."""
        return option_keys(self, type_filter)

    @abi_requires_headers("slic3r_config_def.h")
    def option_definition(self, key: str) -> ConfigOptionDefinition | None:
        """Copy a complete normalized global option definition."""
        return option_definition(self, key)

    @abi_requires_headers("slic3r_bridge_detector.h", "slic3r_orchestrator.h")
    def create_bridge_detector(self, **kwargs) -> BridgeDetector:
        """Create an owning detector using borrowed input geometry views."""
        return self.orchestrator_view().create_bridge_detector(**kwargs)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def object_extruders(self, print_view: Print, obj: Object) -> tuple[int, ...]:
        """Return copied zero-based extruder ids used by one borrowed Object."""
        return object_extruders(print_view, obj)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def collect_object_printing_extruders(
        self, print_view: Print, obj: Object, region: PrintRegion
    ) -> tuple[int, ...]:
        """Return copied extruder ids used by one borrowed printing region."""
        return collect_object_printing_extruders(print_view, obj, region)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def collect_print_extruders_for_skirt(
        self, print_view: Print, objects=None
    ) -> tuple[int, ...]:
        """Return copied zero-based skirt extruders for borrowed Objects."""
        return collect_print_extruders_for_skirt(print_view, objects)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def get_object_first_layer_height(self, print_view: Print, obj: Object) -> int:
        """Return one Object's first-layer height using the pinned C++ port."""
        return get_object_first_layer_height(print_view, obj)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def get_min_first_layer_height(self, print_view: Print) -> int:
        """Return the minimum first-layer height using the pinned C++ port."""
        return get_min_first_layer_height(print_view)

    @abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
    def e_per_mm(self, print_view: Print, extruder_id: int, mm3_per_mm: float) -> float:
        """Convert volumetric flow using the pinned C++ extruder formula."""
        return e_per_mm(print_view, extruder_id, mm3_per_mm)

    @abi_requires_headers("slic3r_def.h")
    def check_z_step(self, value: int, z_step: int) -> int:
        """Round one scaled coordinate using the pinned C++ Z-step formula."""
        return check_z_step(value, z_step)

    @abi_requires_headers("slic3r_utils.h")
    def safe_rand(self, max_value: int = 2**31 - 1) -> int:
        """Return the host's inclusive thread-safe random integer."""
        return int(self.host.slic3r_safe_rand(int(max_value)))

    def add_ui_fragment(self, target_file: str, fragment_id: str, ui_fragment: str, priority: int = 0) -> int:
        return self.host.orchestrator_add_ui_fragment(
            self.orchestrator,
            _as_bytes(target_file),
            _as_bytes(fragment_id),
            _as_bytes(ui_fragment),
            int(priority),
        )

    def register_activation_group(self, group_id: str, members) -> None:
        """Accumulate jointly activated plugin IDs; all member existence checks are deferred."""
        names = [_as_bytes(member) for member in members]
        array = (ctypes.c_char_p * len(names))(*names)
        if not self.host.orchestrator_register_activation_group(
            self.orchestrator, _as_bytes(group_id),
            ConstStrings(ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)), len(names))
        ):
            raise ValueError(
                f"Cannot register activation group {group_id!r}: ID must be nonempty and "
                "each declaration needs at least two distinct nonempty members."
            )

    @abi_requires_headers("slic3r_orchestrator.h")
    def register_runtime_id(self, id_type: str, namespaced_name: str) -> int:
        """Intern a stable typed name and return its orchestrator-local id."""
        return self.orchestrator_view().register_runtime_id(id_type, namespaced_name)

    @abi_requires_headers("slic3r_orchestrator.h")
    def runtime_id_name(self, id_type: str, runtime_id: int) -> str:
        """Copy the stable name for an id, or return empty when it is unknown."""
        return self.orchestrator_view().runtime_id_name(id_type, runtime_id)

    def register_translation_catalog(self, domain: str, locale_directory: str) -> int:
        """Register one gettext domain from this plugin package during registration."""
        return int(self.host.orchestrator_register_translation_catalog(
            self.orchestrator,
            _as_bytes(domain),
            _as_bytes(locale_directory),
        ))

    @abi_requires_headers("slic3r_orchestrator.h")
    def add_gui_rule(
        self,
        *,
        target_key: str,
        condition_key: Optional[str] = None,
        condition: int = RAW_GUI_RULE_CONDITION_BOOL_TRUE,
        target_index: int = RAW_GUI_RULE_INDEX_ALL,
        condition_index: int = RAW_GUI_RULE_INDEX_ALL,
        condition_float_value: float = 0.0,
        condition_string_value: Optional[str] = None,
        clauses: Optional[Iterable] = None,
        scope: int = RAW_PRESET_TYPE_NONE,
    ) -> int:
        """Register one GUI activation rule.

        The clauses of a rule are ANDed. Pass clauses as an iterable of
        mappings or (condition, condition_key[, condition_index
        [, condition_float_value[, condition_string_value]]]) sequences. The
        numeric operand is rounded for INT_* conditions and compared directly
        for FLOAT_* conditions; the string operand is compared with the
        serialized option value for STRING_EQUALS. When clauses is omitted, one
        clause is built from condition_key and the single-condition keyword
        arguments.

        Every rule is one activation reason: the target is enabled when at
        least one rule registered for it is true. scope restricts the rule to
        one preset kind (RAW_PRESET_TYPE_NONE applies everywhere).

        Returns 1 when the rule was added and 0 when an identical rule was
        already registered. Raises ValueError when the host rejects the
        descriptor (unknown enum value, invalid index, non-finite float
        compare, ...) and RuntimeError on an internal host failure.
        """
        if clauses is None:
            if condition_key is None:
                raise ValueError("add_gui_rule needs condition_key or clauses.")
            clause_inputs = [{
                "condition": condition,
                "condition_key": condition_key,
                "condition_index": condition_index,
                "condition_float_value": condition_float_value,
                "condition_string_value": condition_string_value,
            }]
        else:
            clause_inputs = list(clauses)
            if not clause_inputs:
                raise ValueError("add_gui_rule needs at least one clause.")

        # Keep the clause instances and their contiguous array alive until the
        # host has copied the complete rule.
        clause_instances = [_gui_rule_clause(clause) for clause in clause_inputs]
        clause_array = (RawGuiRuleClause * len(clause_instances))()
        for index, instance in enumerate(clause_instances):
            clause_array[index] = instance

        rule = RawGuiRule()
        rule.struct_size = ctypes.sizeof(RawGuiRule)
        rule.clause_struct_size = ctypes.sizeof(RawGuiRuleClause)
        rule.target_key = _as_bytes(target_key)
        rule.target_index = int(target_index)
        rule.scope = int(scope)
        rule.clauses = ctypes.cast(clause_array, ctypes.POINTER(RawGuiRuleClause))
        rule.clause_count = len(clause_instances)
        result = int(self.host.orchestrator_add_gui_rule(self.orchestrator, ctypes.byref(rule)))
        if result == -1:
            raise ValueError(
                f"The host rejected the GUI rule descriptor for target '{target_key}'."
            )
        if result < 0:
            raise RuntimeError(
                f"The host failed to register the GUI rule for target '{target_key}' (code {result})."
            )
        return result

    @abi_requires_headers("slic3r_config_def.h", "slic3r_orchestrator.h")
    def create_option_def(self, **kwargs) -> int:
        defn = RawConfigOptionDef()
        defn.height = -1
        defn.width = -1
        defn.label_width = -1
        defn.sidetext_width = -1
        defn.invalidates_step = STEP_ANY

        byte_refs = []
        for key, value in kwargs.items():
            if isinstance(value, str):
                encoded = _as_bytes(value)
                byte_refs.append(encoded)
                setattr(defn, key, encoded)
            elif key == "graph_settings" and isinstance(value, RawGraphSettings):
                # ctypes keeps nested pointer fields alive through the raw
                # structure; retain both it and its pointer until registration
                # has copied the complete graph definition.
                graph_settings_pointer = ctypes.pointer(value)
                byte_refs.extend((value, graph_settings_pointer))
                setattr(defn, key, graph_settings_pointer)
            else:
                setattr(defn, key, value)

        result = self.host.orchestrator_create_option_def(self.orchestrator, ctypes.byref(defn))
        if result != OPTION_DEF_ERROR_OK:
            option_key = kwargs.get("opt_key", "<unknown>")
            raise RuntimeError(f"Cannot register config option '{option_key}': error {result}")
        return result

    def register_generic_facets_annotation(
        self,
        *,
        key: str,
        label: str,
        enforce_label: str,
        block_label: str,
        icon_svg: str,
        translation_domain: str = "",
    ) -> int:
        defn = RawGenericFacetsAnnotationDef(
            _as_bytes(key),
            _as_bytes(label),
            _as_bytes(enforce_label),
            _as_bytes(block_label),
            _as_bytes(translation_domain),
            _as_bytes(icon_svg),
        )
        return int(self.host.orchestrator_register_generic_facets_annotation(self.orchestrator, ctypes.byref(defn)))

    def storage_new_polygon(self, storage_address: int) -> int:
        return int(self.host.storage_new_polygon(ctypes.c_void_p(storage_address)) or 0)

    def multipoint(self, handle: int) -> MultiPoint:
        return MultiPoint(self, handle)

    def polygon(self, handle: int) -> Polygon:
        return Polygon(self, handle)

    def mutable_polygon(self, handle: int) -> MutablePolygon:
        return MutablePolygon(self, handle)

    def polyline(self, handle: int) -> Polyline:
        return Polyline(self, handle)

    def mutable_polyline(self, handle: int) -> MutablePolyline:
        return MutablePolyline(self, handle)

    def polygon_collection(self, handle: int) -> PolygonCollection:
        return PolygonCollection(self, handle)

    def polyline_collection(self, handle: int) -> PolylineCollection:
        return PolylineCollection(self, handle)

    def expolygon(self, handle: int) -> ExPolygon:
        return ExPolygon(self, handle)

    def expolygon_collection(self, handle: int) -> ExPolygonCollection:
        return ExPolygonCollection(self, handle)

    def new_polygon(self, storage_address: int) -> StoredPolygon:
        return StoredPolygon(self, storage_address)

    def new_polyline(self, storage_address: int) -> StoredPolyline:
        return StoredPolyline(self, storage_address)

    def new_polygon_collection(self, storage_address: int) -> StoredPolygonCollection:
        return StoredPolygonCollection(self, storage_address)

    def new_polyline_collection(self, storage_address: int) -> StoredPolylineCollection:
        return StoredPolylineCollection(self, storage_address)

    def new_expolygon(self, storage_address: int) -> StoredExPolygon:
        return StoredExPolygon(self, storage_address)

    def new_expolygon_collection(self, storage_address: int) -> StoredExPolygonCollection:
        return StoredExPolygonCollection(self, storage_address)

    @abi_requires_headers("slic3r_geometry.h", "slic3r_utils.h")
    def convex_hull(self, storage_address: int, polygon: Polygon) -> StoredPolygon:
        """Return a storage-owned hull without modifying the borrowed polygon."""
        if polygon.api is not self:
            raise ValueError("polygon must use this API instance")
        return convex_hull(storage_address, polygon)

    @abi_requires_headers("slic3r_geometry.h", "slic3r_utils.h")
    def expolygons_to_polylines(
        self, storage_address: int, src: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Copy borrowed boundaries into a storage-owned polyline collection."""
        if src.api is not self:
            raise ValueError("source collection must use this API instance")
        return expolygons_to_polylines(storage_address, src)

    def extrusion(self, handle: int) -> ExtrusionEntity:
        return ExtrusionEntity(self, handle)

    def mutable_extrusion(self, handle: int) -> MutableExtrusionEntity:
        return MutableExtrusionEntity(self, handle)

    def new_extrusion(self, storage_address: int, src=None) -> StoredExtrusionEntity:
        return StoredExtrusionEntity(self, storage_address, src)

    def medial_axis_extrusion(self, role: int, flow) -> MedialAxisExtrusionFactory:
        return MedialAxisExtrusionFactory(self, role, flow)

    def medial_axis_thin_wall(self, flow) -> MedialAxisExtrusionFactory:
        return medial_axis_thin_wall(self, flow)

    def medial_axis_gap_fill(self, flow) -> MedialAxisExtrusionFactory:
        return medial_axis_gap_fill(self, flow)

    def register_extrusion_property_type(self, namespaced_name: str, payload_cls) -> int:
        return register_extrusion_property_type(self, namespaced_name, payload_cls)

    @abi_requires_headers("slic3r_orchestrator.h")
    def register_property_key(self, namespaced_name: str, payload_cls) -> PluginPropertyKey:
        """
        Register a typed dynamic property for this orchestrator.

        The returned key may be used with both extrusion entities and generic
        data-tree property containers. Keep the key rather than serializing its
        numeric id: dynamic ids are local to this API/orchestrator instance.
        """
        return PluginPropertyKey.register_dynamic(self, namespaced_name, payload_cls)

    def clipper(self, storage_address: int) -> ClipperContext:
        return ClipperContext(self, storage_address)

    def clipper_operand(self, storage_address: int, source=None) -> ClipperOperand:
        return ClipperOperand(self, storage_address, source)

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_diff_polyline_expolygons(
        self,
        storage_address: int,
        subject: Polyline,
        clip: ExPolygonCollection,
    ) -> StoredPolylineCollection:
        """Return storage-owned open pieces outside the borrowed clip areas."""
        if subject.api is not self or clip.api is not self:
            raise ValueError("subject and clip must use this API instance")
        return clipper_diff_polyline_expolygons(storage_address, subject, clip)

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_intersection_polyline_expolygons(
        self,
        storage_address: int,
        subject: Polyline,
        clip: ExPolygonCollection,
    ) -> StoredPolylineCollection:
        """Return storage-owned open pieces inside the borrowed clip areas."""
        if subject.api is not self or clip.api is not self:
            raise ValueError("subject and clip must use this API instance")
        return clipper_intersection_polyline_expolygons(
            storage_address, subject, clip
        )

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_diff_polylines_expolygons(
        self, storage_address: int, subject: PolylineCollection, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Return a storage-owned batch result; inputs must belong to this API instance."""
        if subject.api is not self or clip.api is not self:
            raise ValueError("subject and clip must use this API instance")
        return clipper_diff_polylines_expolygons(storage_address, subject, clip)

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_intersection_polylines_expolygons(
        self, storage_address: int, subject: PolylineCollection, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Return a storage-owned batch result; inputs must belong to this API instance."""
        if subject.api is not self or clip.api is not self:
            raise ValueError("subject and clip must use this API instance")
        return clipper_intersection_polylines_expolygons(storage_address, subject, clip)

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_split_polyline_expolygons(
        self,
        storage_address: int,
        subject: Polyline,
        clip: ExPolygonCollection,
    ) -> tuple[StoredPolylineCollection, StoredPolylineCollection]:
        """Return storage-owned ``(inside, outside)`` pieces cut at the same points."""
        if subject.api is not self or clip.api is not self:
            raise ValueError("subject and clip must use this API instance")
        return clipper_split_polyline_expolygons(storage_address, subject, clip)

    @abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def clipper_clip_expolygons_with_subject_bbox(
        self,
        storage_address: int,
        src: ExPolygonCollection,
        bbox: CBoundingBox,
    ) -> StoredExPolygonCollection:
        """Copy borrowed ExPolygons into a storage-owned, bbox-clipped collection."""
        if src.api is not self:
            raise ValueError("source collection must use this API instance")
        return clipper_clip_expolygons_with_subject_bbox(storage_address, src, bbox)

    def config(self, handle: int) -> Config:
        return Config(self, handle)

    def mutable_config(self, handle: int) -> MutableConfig:
        return MutableConfig(self, handle)

    def new_config(self, storage_address: int) -> StoredConfig:
        return StoredConfig(self, storage_address)

    def config_option(self, handle: int) -> ConfigOption:
        return ConfigOption(self, handle)

    def mutable_config_option(self, handle: int) -> MutableConfigOption:
        return MutableConfigOption(self, handle)

    @abi_requires_headers("slic3r_data_tree.h")
    def surface(self, handle: int) -> Surface:
        return Surface(self, handle)

    @abi_requires_headers("slic3r_data_tree.h")
    def mutable_surface(self, handle: int, properties: int) -> MutableSurface:
        """Borrow a mutable Surface view explicitly supplied by the host."""
        return MutableSurface(self, handle, properties)

    @abi_requires_headers("slic3r_data_tree.h")
    def surface_collection(self, handle: int) -> SurfaceCollection:
        return SurfaceCollection(self, handle)

    @abi_requires_headers("slic3r_data_tree.h", "slic3r_geometry.h")
    def new_surface_collection(self, storage_address: int) -> StoredSurfaceCollection:
        """Create a temporary mutable Surface collection owned by storage."""
        return StoredSurfaceCollection(self, storage_address)

    @abi_requires_headers("slic3r_data_tree.h", "slic3r_geometry.h")
    def new_surface(
        self, expolygon: StoredExPolygon, surface_type: int
    ) -> StoredSurface:
        """Transfer owned geometry into a temporary standalone Surface value."""
        return StoredSurface(expolygon, surface_type)

    def print(self, handle: int) -> Print:
        return Print(self, handle)

    def mutable_print(self, handle: int) -> MutablePrint:
        return MutablePrint(self, handle)

    def object(self, handle: int) -> Object:
        return Object(self, handle)

    def mutable_object(self, handle: int) -> MutableObject:
        return MutableObject(self, handle)

    @abi_requires_headers("slic3r_volume.h")
    def project_painting_to_polygons(
        self,
        storage_address: int,
        obj: Object,
        paint_key: str,
        painting_value: int,
    ) -> list[StoredPolygonCollection]:
        """
        Project borrowed object painting into storage-owned per-layer polygons.

        Every returned collection must be released with ``free_from_storage()``
        or a ``with`` block before the storage is cleared. The object and its
        facet annotations remain owned by the host and are not modified.
        """
        if obj.api is not self:
            raise ValueError("object must use this API instance")
        return project_painting_to_polygons(
            storage_address, obj, paint_key, painting_value
        )

    def layer(self, handle: int) -> Layer:
        return Layer(self, handle)

    def mutable_layer(self, handle: int) -> MutableLayer:
        return MutableLayer(self, handle)

    def layer_region(self, handle: int) -> LayerRegion:
        return LayerRegion(self, handle)

    def mutable_layer_region(self, handle: int) -> MutableLayerRegion:
        return MutableLayerRegion(self, handle)

    def layer_island(self, handle: int) -> LayerIsland:
        return LayerIsland(self, handle)

    def mutable_layer_island(self, handle: int) -> MutableLayerIsland:
        return MutableLayerIsland(self, handle)

    def layer_region_island(self, handle: int) -> LayerRegionIsland:
        return LayerRegionIsland(self, handle)

    def mutable_layer_region_island(self, handle: int) -> MutableLayerRegionIsland:
        return MutableLayerRegionIsland(self, handle)

    def print_region(self, handle: int) -> PrintRegion:
        return PrintRegion(self, handle)


    def printing_plan(self, handle: int) -> PrintingPlan:
        """Borrow a read-only plan; neither this view nor its children are freed."""
        return PrintingPlan(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def mutable_printing_plan(self, handle: int) -> MutablePrintingPlan:
        """Borrow a mutable plan only when the host payload grants mutation."""
        return MutablePrintingPlan(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def printing_group(self, handle: int) -> PrintingGroup:
        return PrintingGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def mutable_printing_group(self, handle: int) -> MutablePrintingGroup:
        return MutablePrintingGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def printing_layer_group(self, handle: int) -> PrintingLayerGroup:
        return PrintingLayerGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def mutable_printing_layer_group(self, handle: int) -> MutablePrintingLayerGroup:
        return MutablePrintingLayerGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def printing_tool_group(self, handle: int) -> PrintingToolGroup:
        return PrintingToolGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def mutable_printing_tool_group(self, handle: int) -> MutablePrintingToolGroup:
        return MutablePrintingToolGroup(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def printing_extrusion(self, handle: int) -> PrintingExtrusion:
        return PrintingExtrusion(self, handle)

    @abi_requires_headers("slic3r_printing_plan.h")
    def mutable_printing_extrusion(self, handle: int) -> MutablePrintingExtrusion:
        return MutablePrintingExtrusion(self, handle)

    def volume(self, handle: int) -> Volume:
        return Volume(self, handle)

    def triangle_mesh(self, handle: int) -> TriangleMesh:
        return TriangleMesh(self, handle)

    @abi_requires_headers("steps/slic3r_step_layer_height.h")
    def layer_height_context(self, run_ctx_address: int) -> LayerHeightContext | None:
        return LayerHeightContext.from_layer_height_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_slicing.h")
    def slicing_context(self, run_ctx_address: int) -> SlicingContext | None:
        return SlicingContext.from_slicing_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_post_slicing.h")
    def post_slicing_context(self, run_ctx_address: int) -> PostSlicingContext | None:
        return PostSlicingContext.from_post_slicing_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_post_perimeter.h")
    def post_perimeter_context(self, run_ctx_address: int) -> PostPerimeterContext | None:
        return PostPerimeterContext.from_post_perimeter_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_post_infill.h")
    def post_infill_context(self, run_ctx_address: int) -> PostInfillContext | None:
        return PostInfillContext.from_post_infill_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_perimeter.h")
    def perimeter_context(self, run_ctx_address: int) -> PerimeterContext | None:
        return PerimeterContext.from_perimeter_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_perimeter.h")
    def perimeter_module_context(self, run_ctx_address: int) -> PerimeterModuleContext | None:
        return PerimeterModuleContext.from_perimeter_module_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_seam_placer.h")
    def seam_placer_context(self, run_ctx_address: int) -> SeamPlacerContext | None:
        """Borrow the service payload used to publish a Python seam session."""
        return SeamPlacerContext.from_seam_placer_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_gcode_firmware.h")
    def gcode_firmware_context(self, run_ctx_address: int) -> GCodeFirmwareContext | None:
        """Borrow the service payload used to publish a Python firmware session."""
        return GCodeFirmwareContext.from_gcode_firmware_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_gcode_thumbnail.h")
    def gcode_thumbnail_context(self, run_ctx_address: int) -> GCodeThumbnailContext | None:
        """Borrow the service payload used to publish a Python thumbnail encoder."""
        return GCodeThumbnailContext.from_gcode_thumbnail_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_surface_generation.h")
    def surface_generation_context(self, run_ctx_address: int) -> SurfaceGenerationContext | None:
        return SurfaceGenerationContext.from_surface_generation_run_context(self, run_ctx_address)

    @abi_requires_headers("steps/slic3r_step_pre_perimeter.h")
    def pre_perimeter_context(self, address: int) -> PrePerimeterContext | None:
        return PrePerimeterContext.from_pre_perimeter_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_pre_infill.h")
    def pre_infill_context(self, address: int) -> PreInfillContext | None:
        return PreInfillContext.from_pre_infill_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_infill_group.h")
    def infill_grouping_context(self, address: int) -> InfillGroupingContext | None:
        return InfillGroupingContext.from_infill_grouping_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_infill.h")
    def infill_context(self, address: int) -> InfillContext | None:
        return InfillContext.from_infill_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_infill.h")
    def infill_pattern_context(self, address: int) -> InfillPatternContext | None:
        return InfillPatternContext.from_infill_pattern_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_infill.h")
    def infill_surface_recipe_modifier_context(
        self, address: int
    ) -> InfillSurfaceRecipeModifierContext | None:
        return InfillSurfaceRecipeModifierContext.from_infill_surface_recipe_modifier_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_support.h")
    def support_context(self, address: int) -> SupportContext | None:
        return SupportContext.from_support_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_support_demand.h")
    def support_demand_context(self, address: int) -> SupportDemandContext | None:
        return SupportDemandContext.from_support_demand_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_support_toolpath.h")
    def support_toolpath_context(self, address: int) -> SupportToolpathContext | None:
        """Support toolpath context, or None when the step or payload does not match."""
        return SupportToolpathContext.from_support_toolpath_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_support_spot.h")
    def support_spot_context(self, address: int) -> SupportSpotContext | None:
        return SupportSpotContext.from_support_spot_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_skirt_brim.h")
    def skirt_brim_context(self, address: int) -> SkirtBrimContext | None:
        return SkirtBrimContext.from_skirt_brim_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_wipetower.h")
    def wipe_tower_context(self, address: int) -> WipeTowerContext | None:
        """Borrow the wipe manager's read-only Print and mutable whole plan."""
        return WipeTowerContext.from_wipe_tower_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_ordering.h")
    def ordering_context(self, address: int) -> OrderingContext | None:
        return OrderingContext.from_ordering_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_finalize_slicing.h")
    def finalize_slicing_context(self, address: int) -> FinalizeSlicingContext | None:
        return FinalizeSlicingContext.from_finalize_slicing_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_gcode.h")
    def gcode_context(self, address: int) -> GCodeContext | None:
        return GCodeContext.from_gcode_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_extrusion_edit.h")
    def extrusion_edit_context(self, address: int) -> ExtrusionEditContext | None:
        return ExtrusionEditContext.from_extrusion_edit_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_layer_extrusion_edit.h")
    def layer_extrusion_edit_context(self, address: int) -> LayerExtrusionEditContext | None:
        return LayerExtrusionEditContext.from_layer_extrusion_edit_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_extrusion_simplification.h")
    def extrusion_simplification_context(self, address: int) -> ExtrusionSimplificationContext | None:
        return ExtrusionSimplificationContext.from_extrusion_simplification_run_context(self, address)

    @abi_requires_headers("steps/slic3r_step_layer_stiching.h")
    def layer_stitching_context(self, address: int) -> LayerStitchingContext | None:
        return LayerStitchingContext.from_layer_stitching_run_context(self, address)

    def storage_clear(self, storage_address: int) -> None:
        self.host.storage_clear(ctypes.c_void_p(storage_address))

    def storage_size(self, storage_address: int) -> int:
        return int(self.host.storage_size(ctypes.c_void_p(storage_address)))

    def config_get(self, config_address: int, key: str) -> int:
        return int(self.host.config_get(ctypes.c_void_p(config_address), _as_bytes(key)) or 0)

    def _config_enabled_numeric_option(self, config_address: int, key: str, idx: int) -> int:
        """Return an option whose legacy raw numeric index is usable and enabled."""
        option = self.config_get(config_address, key)
        if not option or idx < 0:
            return 0
        handle = ctypes.c_void_p(option)
        size = int(self.host.config_option_size(handle))
        if size == 0:
            return 0

        option_type = int(self.host.config_option_type_get(handle))
        if option_type == SLIC3R_CONFIG_OPTION_POINT:
            if idx >= 2:
                return 0
            enabled_idx = 0
        elif option_type == SLIC3R_CONFIG_OPTION_POINTS:
            if idx >= size * 2:
                return 0
            enabled_idx = idx // 2
        elif self.host.config_option_is_vector(handle):
            if idx >= size:
                return 0
            enabled_idx = idx
        else:
            enabled_idx = 0

        return option if self.host.config_option_is_enabled(handle, enabled_idx) else 0

    def config_bool(self, config_address: int, key: str, idx: int = 0) -> bool:
        opt = self._config_enabled_numeric_option(config_address, key, idx)
        return bool(opt and self.host.config_option_get_bool(ctypes.c_void_p(opt), int(idx)))

    def config_int(self, config_address: int, key: str, idx: int = 0) -> int:
        opt = self._config_enabled_numeric_option(config_address, key, idx)
        return 0 if not opt else int(self.host.config_option_get_int(ctypes.c_void_p(opt), int(idx)))

    def config_float(self, config_address: int, key: str, idx: int = 0) -> float:
        opt = self._config_enabled_numeric_option(config_address, key, idx)
        return 0.0 if not opt else float(self.host.config_option_get_float(ctypes.c_void_p(opt), int(idx)))

    def config_float_or_percent_effective(self, config_address: int, key: str, ratio: float, idx: int = 0) -> float:
        opt = self._config_enabled_numeric_option(config_address, key, idx)
        if not opt:
            return 0.0
        value = self.host.config_option_get_float_or_percent(ctypes.c_void_p(opt), int(idx))
        return float(self.host.c_float_or_percent_get_effective_value(ctypes.byref(value), float(ratio)))

    def polygon_points(self, polygon_address: int) -> list[CPoint]:
        multipoint = self.host.polygon_as_multipoint_const(ctypes.c_void_p(polygon_address))
        count = int(self.host.multipoint_size(multipoint))
        return [self.host.multipoint_get(multipoint, idx) for idx in range(count)]

    def polygon_point_count(self, polygon_address: int) -> int:
        multipoint = self.host.polygon_as_multipoint_const(ctypes.c_void_p(polygon_address))
        return int(self.host.multipoint_size(multipoint))

    def polygon_convex_point_count(self, polygon_address: int, min_angle: float, max_angle: float) -> int:
        count = self.polygon_point_count(polygon_address)
        array = (ctypes.c_uint32 * count)()
        return int(self.host.polygon_convex_points_idx(
            ctypes.c_void_p(polygon_address),
            float(min_angle),
            float(max_angle),
            array,
            count,
        ))

    def polygon_centroid(self, polygon_address: int) -> CPoint:
        return self.host.polygon_centroid(ctypes.c_void_p(polygon_address))

    def polygon_replace_points(self, polygon_address: int, replacement_polygon_address: int) -> None:
        dst = self.host.polygon_as_multipoint(ctypes.c_void_p(polygon_address))
        src = self.host.polygon_as_multipoint_const(ctypes.c_void_p(replacement_polygon_address))
        self.host.multipoint_copy(dst, src)

    def polygon_push_back(self, polygon_address: int, point: CPoint) -> None:
        multipoint = self.host.polygon_as_multipoint(ctypes.c_void_p(polygon_address))
        self.host.multipoint_push_back(multipoint, point)

    def polygon_make_clockwise(self, polygon_address: int) -> None:
        self.host.polygon_make_clockwise(ctypes.c_void_p(polygon_address))

    def expolygon_holes(self, expolygon_address: int, mutable: bool = False) -> list[int]:
        count = int(self.host.expolygon_hole_size(ctypes.c_void_p(expolygon_address)))
        if mutable:
            return [
                int(self.host.expolygon_hole_at(ctypes.c_void_p(expolygon_address), idx) or 0)
                for idx in range(count)
            ]
        return [
            int(self.host.expolygon_hole_at_const(ctypes.c_void_p(expolygon_address), idx) or 0)
            for idx in range(count)
        ]

    def expolygons(self, collection_address: int, mutable: bool = False) -> list[int]:
        count = int(self.host.expolygons_size(ctypes.c_void_p(collection_address)))
        if mutable:
            return [
                int(self.host.expolygons_at(ctypes.c_void_p(collection_address), idx) or 0)
                for idx in range(count)
            ]
        return [
            int(self.host.expolygons_at_const(ctypes.c_void_p(collection_address), idx) or 0)
            for idx in range(count)
        ]
