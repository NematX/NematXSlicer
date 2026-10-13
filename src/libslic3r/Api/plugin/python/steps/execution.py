#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""High-level Python views for the remaining execution-step payloads.

Each class validates both the step id and the payload before exposing borrowed
host objects. Data-tree and printing-plan handles are wrapped in their normal
Python views. A firmware handle stays an integer because that service still has
no generic Python ownership model.

All returned handles are borrowed and valid only during the current
``PluginBase.setup_run()`` or ``PluginBase.run()`` callback. Collection setters
document the few operations that move plugin-owned content into the host.
Nothing in this module frees a payload or a host-owned data-tree object.
"""

from __future__ import annotations

import ctypes

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    INFILL_PATTERN,
    INFILL_SURFACE_RECIPE_MODIFIER,
    PluginRunContext,
    RawInfillPatternParams,
    RunCtxDetectSupportSpots,
    RunCtxExtrusionEdition,
    RunCtxExtrusionOrdering,
    RunCtxExtrusionSimplification,
    RunCtxFinalizeSlicing,
    RunCtxGenerateGcode,
    RunCtxGenerateInfill,
    RunCtxGenerateSupport,
    RunCtxGenerateWipeTower,
    RunCtxGroupInfillRegions,
    RunCtxInfillPattern,
    RunCtxInfillSurfaceRecipeModifier,
    RunCtxLayerExtrusionEdition,
    RunCtxLayerStiching,
    RunCtxPrepareForPerimeters,
    RunCtxPrepareInfill,
    RunCtxSkirtBrim,
    RunCtxSupportDemand,
    RunCtxSupportToolpath,
    STEP_EXTRUSION_EDIT,
    STEP_EXTRUSION_SIMPLIFICATION,
    STEP_FINALIZE_SLICING,
    STEP_GCODE,
    STEP_INFILL,
    STEP_INFILL_GROUP,
    STEP_LAYER_EXTRUSION_EDIT,
    STEP_LAYER_STICHING,
    STEP_ORDERING,
    STEP_PRE_INFILL,
    STEP_PRE_PERIMETER,
    STEP_SKIRT_BRIM,
    STEP_SUPPORT,
    STEP_SUPPORT_DEMAND,
    STEP_SUPPORT_SPOT,
    STEP_SUPPORT_TOOLPATH,
    STEP_WIPETOWER,
)
from slic3r_datatree_views import (
    Layer,
    LayerIsland,
    LayerRegion,
    LayerRegionIsland,
    MutablePrint,
    Object,
    Print,
)
from slic3r_extrusion_views import MutableExtrusionEntity
from slic3r_geometry_views import ExPolygonCollection, MutableExPolygonCollection
from slic3r_printing_plan_views import (
    LayerExtrusionEditPrintingLayerGroup,
    MutablePrintingPlan,
    PrintingGroup,
    PrintingPlan,
)
from slic3r_surface_views import Surface
from slic3r_run_context import PluginRunContextView


def _address(handle) -> int:
    """Return a numeric C handle without changing or taking its ownership."""
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


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


class _PayloadContext(PluginRunContextView):
    """Shared validated storage for one borrowed ctypes step payload."""

    def __init__(self, api, common: PluginRunContext, payload_ptr) -> None:
        super().__init__(api, common)
        self._payload_ptr = payload_ptr
        self.payload = payload_ptr.contents

    @classmethod
    def _from_run_context(cls, api, run_ctx_address: int, step: int, payload_type, required=()):
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != step or not common.data:
            return None
        payload_ptr = ctypes.cast(common.data, ctypes.POINTER(payload_type))
        payload = payload_ptr.contents
        if any(not getattr(payload, field) for field in required):
            return None
        return cls(api, common, payload_ptr)


class _PrintObjectContext(_PayloadContext):
    """Base for simple object steps with borrowed read-only Print and Object."""

    def _print(self) -> Print:
        return Print(self.api, self.payload.print)

    def _object(self) -> Object:
        return Object(self.api, self.payload.object)


@abi_requires_headers("steps/slic3r_step_pre_perimeter.h")
class PrePerimeterContext(_PrintObjectContext):
    """Read-only object context used immediately before perimeter generation."""

    @classmethod
    def from_pre_perimeter_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_PRE_PERIMETER, RunCtxPrepareForPerimeters, ("print", "object"))

    def pre_perimeter_print(self) -> Print:
        return self._print()

    def pre_perimeter_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_pre_infill.h")
class PreInfillContext(_PrintObjectContext):
    """Read-only object context used to prepare surfaces before infill grouping."""

    @classmethod
    def from_pre_infill_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_PRE_INFILL, RunCtxPrepareInfill, ("print", "object"))

    def pre_infill_print(self) -> Print:
        return self._print()

    def pre_infill_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_infill_group.h")
class InfillGroupingContext(_PrintObjectContext):
    """Read-only object context for plugins grouping compatible infill regions."""

    @classmethod
    def from_infill_grouping_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_INFILL_GROUP, RunCtxGroupInfillRegions, ("print", "object"))

    def infill_grouping_print(self) -> Print:
        return self._print()

    def infill_grouping_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_support.h")
class SupportContext(_PrintObjectContext):
    """Read-only object context for the support-generation step."""

    @classmethod
    def from_support_run_context(cls, api, address: int):
        """Validate the step, handles and pattern callbacks, returning None on mismatch."""
        fields = ("print", "object", "resolve_pattern_id", "generate_pattern")
        return cls._from_run_context(api, address, STEP_SUPPORT, RunCtxGenerateSupport, fields)

    def support_print(self) -> Print:
        return self._print()

    def support_object(self) -> Object:
        return self._object()

    def support_resolve_pattern_id(self, plugin_id: str) -> int:
        """Runtime id of an INFILL_PATTERN plugin, valid only during this step run."""
        return int(self.payload.resolve_pattern_id(self._payload_ptr, plugin_id.encode("utf-8")))

    def support_generate_pattern(
        self, pattern_id: int, layer, island, region_island, primary_region,
        surface, no_overlap_areas, params, output: MutableExtrusionEntity,
    ) -> bool:
        """Fill ``surface`` into the caller-owned ``output``; the host never publishes it at this step.

        The caller only reads ``output`` (for example to sample points).
        ``layer``, ``island``, ``region_island``, ``primary_region`` and
        ``no_overlap_areas`` may be None at this step. ``params`` is a complete
        caller-built RawInfillPatternParams (see the C header for the fields to
        set). False means nothing was generated: an unknown id, a pattern
        refused by this step, or an error contained by the host (a pattern
        setup() or run failure, or a cancellation). In the last case ``output``
        is left as an empty collection, no exception is raised, and the step
        stops with the standard "was cancelled" error once the family returns;
        return from the family soon.
        """
        params_ptr = ctypes.byref(params) if isinstance(params, RawInfillPatternParams) else params
        return bool(self.payload.generate_pattern(
            self._payload_ptr, int(pattern_id), _void_p(layer), _void_p(island),
            _void_p(region_island), _void_p(primary_region), _void_p(surface),
            _void_p(no_overlap_areas), params_ptr, _void_p(output),
        ))


@abi_requires_headers("steps/slic3r_step_support_toolpath.h")
class SupportToolpathContext(_PrintObjectContext):
    """One object's support auxiliary surfaces, ready for toolpath algorithms."""

    @classmethod
    def from_support_toolpath_run_context(cls, api, address: int):
        """Validate the step, handles and pattern callbacks, returning None on mismatch."""
        fields = ("print", "object", "resolve_pattern_id", "generate_pattern")
        return cls._from_run_context(api, address, STEP_SUPPORT_TOOLPATH, RunCtxSupportToolpath, fields)

    def support_toolpath_print(self) -> Print:
        """The print being processed; use it only during this callback."""
        return self._print()

    def support_toolpath_object(self) -> Object:
        """The object whose support auxiliary layers the algorithm processes."""
        return self._object()

    def support_toolpath_resolve_pattern_id(self, plugin_id: str) -> int:
        """Runtime id of an INFILL_PATTERN plugin, valid only during this step run."""
        return int(self.payload.resolve_pattern_id(self._payload_ptr, plugin_id.encode("utf-8")))

    def support_toolpath_generate_pattern(
        self, pattern_id: int, layer, island, region_island, primary_region,
        surface, no_overlap_areas, params, output: MutableExtrusionEntity,
    ) -> bool:
        """Fill ``surface`` into borrowed/plugin-owned ``output``; the caller publishes it.

        ``params`` is a complete caller-built RawInfillPatternParams (see the C
        header for the fields to set). False means nothing was generated
        because of an unknown id or a pattern refused by this step.
        """
        params_ptr = ctypes.byref(params) if isinstance(params, RawInfillPatternParams) else params
        return bool(self.payload.generate_pattern(
            self._payload_ptr, int(pattern_id), _void_p(layer), _void_p(island),
            _void_p(region_island), _void_p(primary_region), _void_p(surface),
            _void_p(no_overlap_areas), params_ptr, _void_p(output),
        ))


@abi_requires_headers("steps/slic3r_step_support_spot.h")
class SupportSpotContext(_PrintObjectContext):
    """Read-only object context for detecting local support spots."""

    @classmethod
    def from_support_spot_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_SUPPORT_SPOT, RunCtxDetectSupportSpots, ("print", "object"))

    def support_spot_print(self) -> Print:
        return self._print()

    def support_spot_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_wipetower.h")
class WipeTowerContext(_PayloadContext):
    """Whole-plan wipe generation with a borrowed Print and mutable plan."""

    @classmethod
    def from_wipe_tower_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_WIPETOWER, RunCtxGenerateWipeTower, ("print", "plan"))

    def wipe_tower_print(self) -> Print:
        """Borrow the read-only source print for this callback."""
        return Print(self.api, self.payload.print)

    def wipe_tower_plan(self) -> MutablePrintingPlan:
        """Borrow the working plan for whole-plan edits during this callback."""
        return MutablePrintingPlan(self.api, self.payload.plan)


@abi_requires_headers("steps/slic3r_step_finalize_slicing.h")
class FinalizeSlicingContext(_PrintObjectContext):
    """Read-only object context used before construction of the printing plan."""

    @classmethod
    def from_finalize_slicing_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_FINALIZE_SLICING, RunCtxFinalizeSlicing, ("print", "object"))

    def finalize_slicing_print(self) -> Print:
        return self._print()

    def finalize_slicing_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_extrusion_simplification.h")
class ExtrusionSimplificationContext(_PrintObjectContext):
    """Object context for simplifying the object's extrusion trees in place."""

    @classmethod
    def from_extrusion_simplification_run_context(cls, api, address: int):
        return cls._from_run_context(
            api, address, STEP_EXTRUSION_SIMPLIFICATION, RunCtxExtrusionSimplification, ("print", "object")
        )

    def extrusion_simplification_print(self) -> Print:
        return self._print()

    def extrusion_simplification_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_layer_stiching.h")
class LayerStitchingContext(_PrintObjectContext):
    """Object context for joining compatible paths after per-layer editing."""

    @classmethod
    def from_layer_stitching_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_LAYER_STICHING, RunCtxLayerStiching, ("print", "object"))

    def layer_stitching_print(self) -> Print:
        return self._print()

    def layer_stitching_object(self) -> Object:
        return self._object()


@abi_requires_headers("steps/slic3r_step_skirt_brim.h")
class SkirtBrimContext(_PayloadContext):
    """Expose the host-owned mutable Print used to publish skirt/brim layers."""

    @classmethod
    def from_skirt_brim_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_SKIRT_BRIM, RunCtxSkirtBrim, ("print",))

    def skirt_brim_mutable_print(self) -> MutablePrint:
        """Borrow the mutable Print; never free or retain it after the callback."""
        return MutablePrint(self.api, self.payload.print)


class _PrintingPlanContext(_PayloadContext):
    """Common read-only Print plus a host-owned printing-plan work copy."""

    def _print(self) -> Print:
        return Print(self.api, self.payload.print)

    def _plan_handle(self) -> int:
        return _address(self.payload.plan)

    def _plan(self) -> MutablePrintingPlan:
        return MutablePrintingPlan(self.api, self.payload.plan)


@abi_requires_headers("steps/slic3r_step_ordering.h")
class OrderingContext(_PrintingPlanContext):
    """Context for ordering structures within a mutable host printing plan."""

    @classmethod
    def from_ordering_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_ORDERING, RunCtxExtrusionOrdering, ("print", "plan"))

    def ordering_print(self) -> Print:
        return self._print()

    def ordering_plan_handle(self) -> int:
        """Return the low-level borrowed plan address for legacy plugin code."""
        return self._plan_handle()

    def ordering_plan(self) -> MutablePrintingPlan:
        """Borrow the mutable ordering plan; never free or retain it after the callback."""
        return self._plan()


@abi_requires_headers("steps/slic3r_step_extrusion_edit.h")
class ExtrusionEditContext(_PrintingPlanContext):
    """Whole-plan extrusion-edit context."""

    @classmethod
    def from_extrusion_edit_run_context(cls, api, address: int):
        return cls._from_run_context(api, address, STEP_EXTRUSION_EDIT, RunCtxExtrusionEdition, ("print", "plan"))

    def extrusion_edit_print(self) -> Print:
        return self._print()

    def extrusion_edit_plan_handle(self) -> int:
        """Return the low-level borrowed plan address for legacy plugin code."""
        return self._plan_handle()

    def extrusion_edit_plan(self) -> MutablePrintingPlan:
        """Borrow the mutable whole-plan work copy for this callback."""
        return self._plan()


@abi_requires_headers("steps/slic3r_step_layer_extrusion_edit.h")
class LayerExtrusionEditContext(_PrintingPlanContext):
    """Per-layer plan context whose layer-group extrusion roots may be edited."""

    @classmethod
    def from_layer_extrusion_edit_run_context(cls, api, address: int):
        return cls._from_run_context(
            api,
            address,
            STEP_LAYER_EXTRUSION_EDIT,
            RunCtxLayerExtrusionEdition,
            ("print", "plan", "group", "layer_group"),
        )

    def layer_extrusion_edit_print(self) -> Print:
        return self._print()

    def layer_extrusion_edit_plan_handle(self) -> int:
        return self._plan_handle()

    def layer_extrusion_edit_plan(self) -> PrintingPlan:
        """Borrow read-only structural context shared by parallel layer jobs."""
        return PrintingPlan(self.api, self.payload.plan)

    def layer_extrusion_edit_group_handle(self) -> int:
        """Return the borrowed read-only PrintingGroup containing this job."""
        return _address(self.payload.group)

    def layer_extrusion_edit_group(self) -> PrintingGroup:
        """Borrow the containing group without granting structural mutation."""
        return PrintingGroup(self.api, self.payload.group)

    def layer_extrusion_edit_layer_group_handle(self) -> int:
        """Return the borrowed mutable PrintingLayerGroup owned by this job."""
        return _address(self.payload.layer_group)

    def layer_extrusion_edit_layer_group(self) -> LayerExtrusionEditPrintingLayerGroup:
        """Borrow a restricted view exposing mutation only on cloned roots."""
        return LayerExtrusionEditPrintingLayerGroup(self.api, self.payload.layer_group)

    def layer_extrusion_edit_group_index(self) -> int:
        return int(self.payload.group_idx)

    def layer_extrusion_edit_layer_group_index(self) -> int:
        return int(self.payload.layer_group_idx)


@abi_requires_headers("steps/slic3r_step_gcode.h")
class GCodeContext(_PrintingPlanContext):
    """Final output context with a borrowed validated firmware service."""

    @classmethod
    def from_gcode_run_context(cls, api, address: int):
        return cls._from_run_context(
            api,
            address,
            STEP_GCODE,
            RunCtxGenerateGcode,
            ("print", "plan", "output_path", "publish_reader_source_path"),
        )

    def gcode_print(self) -> Print:
        return self._print()

    def gcode_plan_handle(self) -> int:
        return self._plan_handle()

    def gcode_plan(self) -> PrintingPlan:
        """Borrow the finalized read-only plan owned by this G-code run."""
        return PrintingPlan(self.api, self.payload.plan)

    def gcode_firmware_handle(self) -> int:
        """Return a borrowed firmware instance; invoke it only through its service adapter."""
        return _address(self.payload.firmware)

    @abi_requires_headers("slic3r_gcode_firmware.h")
    def gcode_firmware(self):
        """Borrow the host-owned firmware instance for this G-code run only."""
        if not self.payload.firmware:
            raise ValueError("the G-code context has no firmware instance")
        from .gcode_firmware import GCodeFirmwareView

        return GCodeFirmwareView(self.api, self.payload.firmware)

    def gcode_output_path(self) -> str:
        """Copy the borrowed UTF-8 output path into an independently owned Python string."""
        return self.payload.output_path.decode("utf-8")

    def publish_gcode_reader_source_path(self, path: str) -> None:
        """Publish the completed text file that the host Reader may consume.

        The host copies ``path`` during this call but does not own the file.
        The writer must keep it available while the resulting preview is used,
        and may publish at most one path during the current run.
        """
        if not isinstance(path, str) or not path:
            raise ValueError("the G-code Reader source path must be a non-empty string")
        encoded = path.encode("utf-8")
        if not self.payload.publish_reader_source_path(
            self.payload.reader_source_context, encoded
        ):
            raise RuntimeError("the host rejected the G-code Reader source path")


@abi_requires_headers("steps/slic3r_step_infill.h")
class InfillContext(_PrintObjectContext):
    """High-level dispatcher used by STEP_INFILL generators."""

    @classmethod
    def from_infill_run_context(cls, api, address: int):
        callbacks = (
            "print", "object", "resolve_pattern_id", "pattern_plugin_id",
            "generate_pattern", "modify_surface_recipe", "append_region_island_extrusion",
        )
        return cls._from_run_context(api, address, STEP_INFILL, RunCtxGenerateInfill, callbacks)

    def infill_print(self) -> Print:
        return self._print()

    def infill_object(self) -> Object:
        return self._object()

    def infill_resolve_pattern_id(self, plugin_id: str) -> int:
        """Resolve a configured pattern id for this run, using the host fallback when it is inactive.

        Returns 0 when the step is stopping or a host failure is contained. The
        failure is reported to the host, which stops STEP_INFILL after the
        generator returns; a normal inactive id may resolve to a fallback.
        """
        return int(self.payload.resolve_pattern_id(self._payload_ptr, plugin_id.encode("utf-8")))

    def infill_pattern_plugin_id(self, pattern_id: int) -> str | None:
        """Return the stable pattern id.

        The host returns ``""`` (never NULL) for an unknown id or a stopping step;
        this wrapper maps that value to ``None``.
        """
        value = self.payload.pattern_plugin_id(self._payload_ptr, int(pattern_id))
        return None if not value else value.decode("utf-8")

    def infill_modify_surface_recipe(
        self, layer, island, region_island, primary_region, surface, no_overlap_areas, params
    ) -> bool:
        """Run recipe modifiers in priority order against the same mutable ``params``.

        If a modifier fails, later modifiers do not run and earlier edits remain.
        The call adds no diagnostic of its own; the failing modifier's error is
        the plugin message. Plugin cancellation is requested, the call returns
        ``False`` and the runner stops STEP_INFILL after ``setup_run()``
        returns. Print cancellation is rethrown by the runner instead of
        becoming a plugin error.
        """
        params_ptr = ctypes.byref(params) if isinstance(params, RawInfillPatternParams) else params
        return bool(self.payload.modify_surface_recipe(
            self._payload_ptr, _void_p(layer), _void_p(island), _void_p(region_island),
            _void_p(primary_region), _void_p(surface), _void_p(no_overlap_areas), params_ptr,
        ))

    def infill_generate_pattern(
        self, pattern_id: int, layer, island, region_island, primary_region,
        surface, no_overlap_areas, params, output: MutableExtrusionEntity,
    ) -> bool:
        """Generate one surface into caller-owned ``output`` without transferring it.

        An ordinary refusal returns ``False`` and leaves output unchanged. A
        contained failure returns ``False``, empties output and asks the runner
        to stop STEP_INFILL after the generator returns. Print cancellation is
        left for the runner to rethrow.
        """
        params_ptr = ctypes.byref(params) if isinstance(params, RawInfillPatternParams) else params
        return bool(self.payload.generate_pattern(
            self._payload_ptr, int(pattern_id), _void_p(layer), _void_p(island),
            _void_p(region_island), _void_p(primary_region), _void_p(surface),
            _void_p(no_overlap_areas), params_ptr, _void_p(output),
        ))

    def infill_append_region_island_extrusion(
        self, region_island: LayerRegionIsland, role: int, extrusion: MutableExtrusionEntity
    ) -> bool:
        """Append extrusion children to the host bucket selected by ``role``.

        On success the host owns the moved paths and ``extrusion`` remains a
        valid, emptied wrapper. If preparation fails, earlier destination paths
        stay published, the source may be partly consumed, and the host requests
        plugin cancellation; print cancellation is handled by the runner.
        """
        return bool(self.payload.append_region_island_extrusion(
            _void_p(region_island), int(role), _void_p(extrusion)
        ))


class _InfillSurfaceContext(_PayloadContext):
    """Shared borrowed geometry exposed to per-surface infill services."""

    def infill_surface_print(self) -> Print:
        return Print(self.api, self.payload.print)

    def infill_surface_object(self) -> Object:
        return Object(self.api, self.payload.object)

    def infill_surface_layer(self) -> Layer:
        return Layer(self.api, self.payload.layer)

    def infill_surface_island(self) -> LayerIsland:
        return LayerIsland(self.api, self.payload.island)

    def infill_surface_region_island(self) -> LayerRegionIsland:
        return LayerRegionIsland(self.api, self.payload.region_island)

    def infill_surface_primary_region(self) -> LayerRegion:
        return LayerRegion(self.api, self.payload.primary_region)

    def infill_surface(self) -> Surface:
        return Surface(self.api, self.payload.surface)

    def infill_surface_no_overlap_areas(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.payload.no_overlap_areas)


@abi_requires_headers("steps/slic3r_step_infill.h")
class InfillPatternContext(_InfillSurfaceContext):
    """Per-surface input/output for an INFILL_PATTERN provider."""

    @classmethod
    def from_infill_pattern_run_context(cls, api, address: int):
        fields = ("print", "object", "layer", "island", "region_island", "primary_region", "surface", "params", "output")
        return cls._from_run_context(api, address, INFILL_PATTERN, RunCtxInfillPattern, fields)

    def infill_pattern_params(self) -> RawInfillPatternParams:
        """Copy read-only recipe values; editing the copy cannot affect the host."""
        return RawInfillPatternParams.from_buffer_copy(self.payload.params.contents)

    def infill_pattern_output(self) -> MutableExtrusionEntity:
        """Borrow the empty host output; write it in this call and never free it."""
        return MutableExtrusionEntity(self.api, self.payload.output)


@abi_requires_headers("steps/slic3r_step_infill.h")
class InfillSurfaceRecipeModifierContext(_InfillSurfaceContext):
    """Per-surface recipe editor; geometry remains read-only."""

    @classmethod
    def from_infill_surface_recipe_modifier_run_context(cls, api, address: int):
        fields = (
            "print", "object", "layer", "island", "region_island", "primary_region",
            "surface", "generate_infill_ctx", "resolve_pattern_id", "pattern_plugin_id", "params",
        )
        return cls._from_run_context(api, address, INFILL_SURFACE_RECIPE_MODIFIER, RunCtxInfillSurfaceRecipeModifier, fields)

    def infill_recipe_params(self) -> RawInfillPatternParams:
        """Borrow mutable recipe fields; changes are consumed after this callback."""
        return self.payload.params.contents

    def infill_recipe_resolve_pattern_id(self, plugin_id: str) -> int:
        return int(self.payload.resolve_pattern_id(self.payload.generate_infill_ctx, plugin_id.encode("utf-8")))

    def infill_recipe_pattern_plugin_id(self, pattern_id: int) -> str | None:
        """Return the stable pattern id.

        The host returns ``""`` (never NULL) for an unknown id or a stopping step;
        this wrapper maps that value to ``None``.
        """
        value = self.payload.pattern_plugin_id(self.payload.generate_infill_ctx, int(pattern_id))
        return None if not value else value.decode("utf-8")


@abi_requires_headers("steps/slic3r_step_support_demand.h")
class SupportDemandContext(_PrintObjectContext):
    """
    STEP_SUPPORT_DEMAND payload: the print and the object whose islands get a
    support demand.

    Read an island's demand with LayerIsland.support_demand(); edit it in place
    through support_demand_get_island_mutable(), available on every run context
    from STEP_SUPPORT_DEMAND through STEP_SUPPORT.
    """

    @classmethod
    def from_support_demand_run_context(cls, api, address: int):
        fields = ("print", "object")
        return cls._from_run_context(api, address, STEP_SUPPORT_DEMAND, RunCtxSupportDemand, fields)

    def support_demand_print(self) -> Print:
        return self._print()

    def support_demand_object(self) -> Object:
        return self._object()


__all__ = [
    "ExtrusionEditContext",
    "ExtrusionSimplificationContext",
    "GCodeContext",
    "InfillContext",
    "InfillGroupingContext",
    "InfillPatternContext",
    "InfillSurfaceRecipeModifierContext",
    "LayerExtrusionEditContext",
    "LayerStitchingContext",
    "OrderingContext",
    "FinalizeSlicingContext",
    "PreInfillContext",
    "PrePerimeterContext",
    "SkirtBrimContext",
    "SupportContext",
    "SupportDemandContext",
    "SupportSpotContext",
    "SupportToolpathContext",
    "WipeTowerContext",
]
