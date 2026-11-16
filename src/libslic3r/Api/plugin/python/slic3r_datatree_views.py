#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Python views over the SuperSlicer print data tree.

The data tree is the host-owned hierarchy a plugin usually receives through a
step payload:

    Print -> Object -> Layer -> LayerIsland / LayerRegion -> LayerRegionIsland

The wrappers here keep raw C handles out of normal plugin code. Use read-only
views for borrowed const handles, and Mutable* views only when a step payload or
host callback explicitly gives a mutable handle.

Typical use
-----------

Read layers and islands:

    obj = api.object(ctx.object)
    for layer in obj.layers():
        for island in layer.islands():
            bbox = island.bounding_box()

Read configuration:

    config = obj.config()
    density = config.float("fill_density")
    enabled = config.bool("support_material")

Print.default_region_config() reads the preset's regional defaults without
creating an auxiliary object. Its borrowed result must be reacquired after apply().
Print.default_object_config() reads preset object settings, such as the global
wipe tower position. Object.default_region_config(print) uses the host's fallback
for objects without regions. These queries never create geometry or edit settings.

Inspect region-island extrusions:

    region_island = island.region_island(0)
    if region_island.has_extrusion(RAW_EXTRUSION_ROLE_PERIMETER):
        extrusion = region_island.extrusion(RAW_EXTRUSION_ROLE_PERIMETER)
        print(extrusion.length())

LayerIsland.perimeter_slices() borrows filled XY areas published by perimeter
processing for travel boundaries, not extrusion paths. The collection may be
empty. See layer_island_get_perimeter_slices in slic3r_data_tree.h for publication,
lifetime and thread rules.

Property guide: [Using Plugin Properties](/doc/plugins/properties.md)

Extrusion guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)

These wrappers do not keep the host objects alive. A view remains usable only
while its owner exists and its containing collection has not been rebuilt,
cleared or replaced. Reacquire child views after such a change; copying a
Python wrapper does not copy the geometry behind its handle.
"""

from __future__ import annotations

import ctypes
import math
from typing import Iterable, Iterator, NamedTuple, Sequence

from slic3r_api_generated import (
    CLayerAdhesionProperty,
    CLayerInstanceProperty,
    CLayerBrimProperty,
    CLayerSupportProperty,
    CCurledLine,
    CFlow,
    CFlowMm,
    CFloatOrPercent,
    CMatrix4d,
    CMeshSlicingParams,
    CTriangleIndices,
    RAW_MESH_SLICE_STATUS_CANCELLED,
    RAW_MESH_SLICE_STATUS_DONE,
    RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT,
    CVec2d,
    EXTRUDER_ID_UNKNOWN,
    PLUGIN_PROPERTY_TYPE_LAYER_ADHESION,
    PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE,
    PLUGIN_PROPERTY_TYPE_LAYER_BRIM,
    PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT,
    RAW_FACET_PAINTING_FDM_SUPPORT,
    RAW_FACET_PAINTING_MMU_SEGMENTATION,
    RAW_FACET_PAINTING_SEAM,
    RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY,
    RAW_LAYER_ADHESION_KIND_BRIM,
    RAW_LAYER_ADHESION_KIND_SKIRT,
    RAW_MESH_SLICING_MODE_EVEN_ODD,
    RAW_MESH_SLICING_MODE_POSITIVE,
    RAW_MESH_SLICING_MODE_REGULAR,
    SLIC3R_CONFIG_OPTION_POINT,
    SLIC3R_CONFIG_OPTION_POINTS,
    RAW_VOLUME_TYPE_BRIM_NEGATIVE,
    RAW_VOLUME_TYPE_BRIM_PATCH,
    RAW_VOLUME_TYPE_MODEL_PART,
    RAW_VOLUME_TYPE_NEGATIVE_VOLUME,
    RAW_VOLUME_TYPE_NO_SUPPORT_ZONE,
    RAW_VOLUME_TYPE_PARAMETER_MODIFIER,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER,
    RAW_VOLUME_TYPE_SUPPORT_BLOCKER,
    RAW_VOLUME_TYPE_SUPPORT_COLUMN,
    RAW_VOLUME_TYPE_SUPPORT_ENFORCER,
    CLIPPER_FILL_EVENODD,
    CLIPPER_FILL_NONZERO,
    CLIPPER_FILL_POSITIVE,
    CLIPPER_JOIN_MITER,
    RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR,
    EPSILON,
    SCALED_EPSILON,
    SCALING_FACTOR,
    UNSCALING_FACTOR,
)

from slic3r_clipper_views import (
    ClipperContext,
    clipper_offset2_ex,
    clipper_offset_ex,
    clipper_union_ex,
)
from slic3r_extrusion_views import ExtrusionEntity, MutableExtrusionEntity
from slic3r_geometry_views import (
    CPoint,
    ExPolygon,
    ExPolygonCollection,
    StoredExPolygonCollection,
    StoredPolygonCollection,
    _UniqueOwnerMixin,
)
from slic3r_surface_views import (
    MutableSurface,
    StoredSurface,
    StoredSurfaceCollection,
    Surface,
    SurfaceCollection,
    SurfaceTypeBuilder,
    srf_type,
)
from slic3r_abi import abi_requires_headers
from slic3r_run_context import PluginCancelled, _context_view, throw_if_cancelled
from slic3r_property_key import PluginPropertyKey


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


def _require_handle(handle, name: str = "handle") -> int:
    address = _address(handle)
    if not address:
        raise ValueError(f"{name} must not be null")
    return address


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


def _optional(cls, api, handle):
    return None if not _address(handle) else cls(api, handle)


def _decode_const_string(ptr) -> str:
    if not ptr:
        return ""
    return ctypes.cast(ptr, ctypes.c_char_p).value.decode("utf-8", errors="replace")


def _payload_type(payload_cls_or_type) -> int:
    if isinstance(payload_cls_or_type, int):
        return int(payload_cls_or_type)
    return int(payload_cls_or_type.property_type)


def _payload_class(payload_cls_or_key):
    return getattr(payload_cls_or_key, "payload_cls", payload_cls_or_key)


def _validate_property_key(owner, payload_cls_or_type) -> None:
    if isinstance(payload_cls_or_type, PluginPropertyKey):
        payload_cls_or_type.validate_owner(owner)


def _property_orchestrator(api, payload_cls_or_key, explicit_orchestrator=None):
    if isinstance(payload_cls_or_key, PluginPropertyKey):
        if payload_cls_or_key.api is not None and payload_cls_or_key.api is not api:
            raise ValueError("a dynamic property key belongs to a different API/orchestrator")
        if payload_cls_or_key.orchestrator is not None:
            if explicit_orchestrator is not None and (
                _address(explicit_orchestrator) != _address(payload_cls_or_key.orchestrator)
            ):
                raise ValueError("the explicit orchestrator does not own this dynamic property key")
            return payload_cls_or_key.orchestrator
    return api.orchestrator if explicit_orchestrator is None else _void_p(explicit_orchestrator)


def _payload_copy(payload_cls, ptr: int):
    if not ptr:
        return None
    return payload_cls.from_buffer_copy(ctypes.string_at(_void_p(ptr), ctypes.sizeof(payload_cls)))


def _payload_mutable(payload_cls, ptr: int):
    if not ptr:
        return None
    return ctypes.cast(_void_p(ptr), ctypes.POINTER(payload_cls)).contents


def unscaled(value: int | float) -> float:
    return float(value) * SCALING_FACTOR


CLayerSupportProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT
CLayerBrimProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_BRIM
CLayerAdhesionProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_ADHESION
CLayerInstanceProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE


@abi_requires_headers("slic3r_data_tree.h")
class LayerInstanceProperty(CLayerInstanceProperty):
    """Object-local auxiliary geometry restricted to one instance index."""


LayerInstanceProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_INSTANCE
LayerInstanceProperty.key = PluginPropertyKey.built_in(LayerInstanceProperty)


@abi_requires_headers("slic3r_data_tree.h")
class LayerSupportProperty(CLayerSupportProperty):
    """Value snapshot marking an auxiliary layer as generated support."""


@abi_requires_headers("slic3r_data_tree.h")
class LayerBrimProperty(CLayerBrimProperty):
    """Value snapshot of the legacy object-owned brim marker."""


@abi_requires_headers("slic3r_data_tree.h")
class LayerAdhesionProperty(CLayerAdhesionProperty):
    """Value snapshot describing generated brim or skirt geometry."""

    @abi_requires_headers("slic3r_data_tree.h")
    def has_kind(self, expected_kind: int) -> bool:
        """Return whether ``kind`` equals the requested ABI adhesion kind."""
        return int(self.kind) == int(expected_kind)

    @abi_requires_headers("slic3r_data_tree.h")
    def has_flag(self, flag: int) -> bool:
        """Return whether at least one bit requested by ``flag`` is present."""
        return (int(self.flags) & int(flag)) != 0

    @abi_requires_headers("slic3r_data_tree.h")
    def is_brim(self) -> bool:
        """Return whether this payload identifies brim geometry."""
        return self.has_kind(RAW_LAYER_ADHESION_KIND_BRIM)

    @abi_requires_headers("slic3r_data_tree.h")
    def is_skirt(self) -> bool:
        """Return whether this payload identifies skirt geometry."""
        return self.has_kind(RAW_LAYER_ADHESION_KIND_SKIRT)

    @abi_requires_headers("slic3r_data_tree.h")
    def is_first_layer_only(self) -> bool:
        """Return whether this adhesion geometry is limited to layer one."""
        return self.has_flag(RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY)


LayerSupportProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT
LayerBrimProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_BRIM
LayerAdhesionProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_ADHESION

LayerSupportProperty.key = PluginPropertyKey.built_in(LayerSupportProperty)
LayerBrimProperty.key = PluginPropertyKey.built_in(LayerBrimProperty)
LayerAdhesionProperty.key = PluginPropertyKey.built_in(LayerAdhesionProperty)


@abi_requires_headers("slic3r_data_tree.h")
def layer_adhesion_property(layer):
    """
    Copy the adhesion payload directly stored on ``layer``, if present.

    The layer and its property container remain host-owned. The returned ctypes
    value is an independent copy and needs no release; changing its fields does
    not update the layer. Use ``layer.mutable_properties().get_or_add()`` explicitly
    when the current step is allowed to annotate the layer.
    """
    return layer.properties().get(LayerAdhesionProperty.key)


@abi_requires_headers("slic3r_data_tree.h")
def layer_has_adhesion_kind(layer, kind: int) -> bool:
    """
    Test the effective brim/skirt kind, including the legacy brim marker.

    A modern adhesion payload is authoritative when present. Only when it is
    absent does ``LayerBrimProperty`` count as brim, which keeps older producer
    plugins compatible without treating that marker as skirt metadata.
    """
    adhesion = layer_adhesion_property(layer)
    if adhesion is not None:
        return adhesion.has_kind(kind)
    return int(kind) == int(RAW_LAYER_ADHESION_KIND_BRIM) and layer.properties().has(
        LayerBrimProperty.key
    )


@abi_requires_headers("slic3r_data_tree.h")
def layer_is_brim(layer) -> bool:
    """Return whether a borrowed layer effectively represents brim geometry."""
    return layer_has_adhesion_kind(layer, RAW_LAYER_ADHESION_KIND_BRIM)


@abi_requires_headers("slic3r_data_tree.h")
def layer_is_normal_skirt(layer) -> bool:
    """Return whether a layer is a recurring skirt, not first-layer-only skirt."""
    adhesion = layer_adhesion_property(layer)
    return adhesion is not None and adhesion.is_skirt() and not adhesion.is_first_layer_only()


@abi_requires_headers("slic3r_data_tree.h")
def layer_is_skirt_first_layer_only(layer) -> bool:
    """Return whether a layer carries the special first-layer-only skirt."""
    adhesion = layer_adhesion_property(layer)
    return adhesion is not None and adhesion.is_skirt() and adhesion.is_first_layer_only()


@abi_requires_headers("slic3r_config_option.h")
class ConfigPoint(NamedTuple):
    """Unscaled XY value stored by a point configuration option."""

    x: float
    y: float


# Base for borrowed data-tree views. It never owns or frees the handle.
@abi_requires_headers("slic3r_data_tree.h")
class DataTreeView:
    def __init__(self, api, handle) -> None:
        self.api = api
        self.address = _require_handle(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def same_handle(self, other: "DataTreeView") -> bool:
        return self.address == other.address

    def __bool__(self) -> bool:
        return self.address != 0


@abi_requires_headers("slic3r_data_tree.h")
class ConstPluginProperties:
    """Borrow metadata for reading; typed payload reads are independent copies."""

    def __init__(self, api, handle) -> None:
        self.api = api
        self.address = _address(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def valid(self) -> bool:
        return self.address != 0

    def count(self) -> int:
        return int(self.api.host.plugin_property_count(self.c_handle()))

    def type_at(self, idx: int) -> int:
        return int(self.api.host.plugin_property_type_at(self.c_handle(), int(idx)))

    def types(self) -> list[int]:
        return [self.type_at(idx) for idx in range(self.count())]

    def has(self, payload_cls_or_type) -> bool:
        _validate_property_key(self, payload_cls_or_type)
        return bool(self.api.host.plugin_property_has(self.c_handle(), _payload_type(payload_cls_or_type)))

    def data_size(self, payload_cls_or_type) -> int:
        _validate_property_key(self, payload_cls_or_type)
        return int(self.api.host.plugin_property_data_size(self.c_handle(), _payload_type(payload_cls_or_type)))

    def data_address(self, payload_cls_or_type) -> int:
        _validate_property_key(self, payload_cls_or_type)
        return _address(self.api.host.plugin_property_data(self.c_handle(), _payload_type(payload_cls_or_type)))

    def get(self, payload_cls_or_key):
        _validate_property_key(self, payload_cls_or_key)
        payload_cls = _payload_class(payload_cls_or_key)
        if self.data_size(payload_cls_or_key) != ctypes.sizeof(payload_cls):
            return None
        return _payload_copy(payload_cls, self.data_address(payload_cls_or_key))

@abi_requires_headers("slic3r_data_tree.h")
class PluginProperties(ConstPluginProperties):
    """Borrow writable metadata through an explicit editing capability."""

    def data_mutable_address(self, payload_cls_or_type) -> int:
        _validate_property_key(self, payload_cls_or_type)
        return _address(self.api.host.plugin_property_data_mutable(self.c_handle(), _payload_type(payload_cls_or_type)))

    def get_mutable(self, payload_cls_or_key):
        _validate_property_key(self, payload_cls_or_key)
        payload_cls = _payload_class(payload_cls_or_key)
        if self.data_size(payload_cls_or_key) != ctypes.sizeof(payload_cls):
            return None
        return _payload_mutable(payload_cls, self.data_mutable_address(payload_cls_or_key))

    def get_or_add(self, payload_cls_or_key, orchestrator=None):
        _validate_property_key(self, payload_cls_or_key)
        payload_cls = _payload_class(payload_cls_or_key)
        orch = _property_orchestrator(self.api, payload_cls_or_key, orchestrator)
        if not self.valid():
            raise RuntimeError("property creation requires a writable capability")
        ptr = self.api.host.plugin_property_get_or_add_data_mutable(
            orch, self.c_handle(), _payload_type(payload_cls_or_key)
        )
        if not _address(ptr):
            raise RuntimeError("native property creation was refused")
        return _payload_mutable(payload_cls, _address(ptr))

    def remove(self, payload_cls_or_type) -> bool:
        _validate_property_key(self, payload_cls_or_type)
        return bool(self.api.host.plugin_property_remove(self.c_handle(), _payload_type(payload_cls_or_type)))

    def clear(self) -> None:
        self.api.host.plugin_property_clear(self.c_handle())

    def copy_from(self, other: "PluginProperties") -> None:
        """
        Replace this container's properties with independent copies of ``other``.

        Both views are borrowed and remain owned by their data-tree objects.
        The payload bytes are copied by the host, so later changes to either
        container do not affect the other. An invalid source is treated as a
        no-op by the native API; this lets property-free ``c_surface`` snapshots
        serve as templates for newly appended Surfaces.
        """
        if not self.valid():
            raise ValueError("destination property container must not be null")
        self.api.host.plugin_property_copy_all(self.c_handle(), other.c_handle())


# Borrowed read-only view over one graph payload stored inside a ConfigOption.
@abi_requires_headers("slic3r_config_option.h")
class GraphData(DataTreeView):
    """
    Borrowed graph value owned by a scalar or vector configuration option.

    The handle is invalidated when its parent option is replaced, resized or
    destroyed. Do not free it or retain it across configuration mutations.
    Point access and interpolation only read the graph; they do not expose its
    mutable native storage. Serialize the parent ConfigOption when a stable text
    representation is needed.
    """

    def size(self) -> int:
        """Return the number of usable points in the graph."""
        return int(self.api.host.graph_data_size(self.c_handle()))

    def point(self, index: int) -> ConfigPoint:
        """
        Copy one usable, unscaled graph point.

        The returned value is independent from the native graph and remains
        valid after its parent option is changed or destroyed.
        """
        if index < 0:
            raise IndexError(f"graph point index {index} is outside [0, {self.size()})")

        point = CVec2d()
        if not self.api.host.graph_data_get_point(
            self.c_handle(), int(index), ctypes.byref(point)
        ):
            raise IndexError(f"graph point index {index} is outside [0, {self.size()})")
        return ConfigPoint(float(point.x), float(point.y))

    def points(self) -> tuple[ConfigPoint, ...]:
        """Copy all usable graph points in interpolation order."""
        return tuple(self.point(index) for index in range(self.size()))

    def type(self) -> int:
        """Return one generated ``RAW_GRAPH_TYPE_*`` interpolation mode."""
        return int(self.api.host.graph_data_type(self.c_handle()))

    def interpolate(self, x: float) -> float:
        """Evaluate the graph at unscaled coordinate ``x``."""
        return float(self.api.host.graph_interpolate(self.c_handle(), float(x)))

    def inverse_interpolate(self, y: float) -> float:
        """Evaluate the host's inverse interpolation at unscaled ``y``."""
        return float(
            self.api.host.graph_inverse_interpolate(self.c_handle(), float(y))
        )

    def validate(self) -> bool:
        """Return whether the native graph points and interpolation mode are valid."""
        return bool(self.api.host.graph_validate(self.c_handle()))

# Read-only view over one ConfigOption. It covers common scalar/vector access
# without exposing raw config_option_handle values.
@abi_requires_headers("slic3r_config_option.h")
class ConfigOption(DataTreeView):
    def type(self) -> int:
        return int(self.api.host.config_option_type_get(self.c_handle()))

    def flags(self) -> int:
        return int(self.api.host.config_option_flags_get(self.c_handle()))

    def size(self) -> int:
        return int(self.api.host.config_option_size(self.c_handle()))

    def is_scalar(self) -> bool:
        return bool(self.api.host.config_option_is_scalar(self.c_handle()))

    def is_vector(self) -> bool:
        return bool(self.api.host.config_option_is_vector(self.c_handle()))

    def is_enabled(self, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_is_enabled(self.c_handle(), int(idx)))

    def can_be_disabled(self) -> bool:
        return bool(self.api.host.config_option_can_be_disabled(self.c_handle()))

    def is_phony(self) -> bool:
        return bool(self.api.host.config_option_is_phony(self.c_handle()))

    def get_bool(self, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_get_bool(self.c_handle(), int(idx)))

    def get_int(self, idx: int = 0) -> int:
        return int(self.api.host.config_option_get_int(self.c_handle(), int(idx)))

    def get_float(self, idx: int = 0) -> float:
        return float(self.api.host.config_option_get_float(self.c_handle(), int(idx)))

    def get_float_or_percent(self, idx: int = 0) -> CFloatOrPercent:
        return self.api.host.config_option_get_float_or_percent(self.c_handle(), int(idx))

    def get_string(self, idx: int = 0) -> str:
        """
        Copy one typed string value from this option.

        Unlike serialize(), this returns the string payload itself without
        option syntax or escaping. A scalar string ignores ``idx``. For a
        vector string, an invalid index produces an empty string at the C
        boundary. Non-string options also produce an empty string; callers
        that need to distinguish these cases should inspect type(), is_vector()
        and size() first.
        """
        needed = int(
            self.api.host.config_option_get_string(
                self.c_handle(), int(idx), None, 0
            )
        )
        output = ctypes.create_string_buffer(needed + 1)
        written = int(
            self.api.host.config_option_get_string(
                self.c_handle(), int(idx), output, len(output)
            )
        )
        return output.value[:written].decode("utf-8")

    def get_point(self, idx: int = 0) -> ConfigPoint:
        """
        Copy one unscaled XY point from a point or points option.

        Point components occupy two consecutive numeric slots in the C ABI.
        For a vector option, ``idx`` selects the point, not a component.
        """
        if idx < 0 or idx >= self.size():
            raise IndexError(
                f"configuration point index {idx} is outside [0, {self.size()})"
            )
        component = int(idx) * 2
        return ConfigPoint(
            float(self.api.host.config_option_get_float(self.c_handle(), component)),
            float(
                self.api.host.config_option_get_float(
                    self.c_handle(), component + 1
                )
            ),
        )

    def graph(self, idx: int = 0) -> GraphData | None:
        """
        Borrow one graph payload, or return None for a wrong type/index.

        A scalar graph ignores ``idx``. For a vector graph, an invalid index
        returns None.

        The returned view is owned by this option and is invalidated by any
        mutation that replaces or resizes the option. It must not be freed.
        """
        return _optional(
            GraphData,
            self.api,
            self.api.host.config_option_get_graph(self.c_handle(), int(idx)),
        )

    def is_percent(self, idx: int = 0) -> bool:
        return bool(self.get_float_or_percent(idx).percent)

    def get_effective_value(self, ratio_over: float, idx: int = 0) -> float:
        value = self.get_float_or_percent(idx)
        return float(self.api.host.c_float_or_percent_get_effective_value(ctypes.byref(value), float(ratio_over)))

    def serialize(self) -> str:
        """Return serialized text; raise RuntimeError if serialization fails or its length changes between calls."""
        needed = int(self.api.host.config_option_serialize(self.c_handle(), None, 0))
        if needed == 0xFFFFFFFF:
            raise RuntimeError("The configuration option could not be serialized")
        out = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_option_serialize(self.c_handle(), out, len(out)))
        if written == 0xFFFFFFFF:
            raise RuntimeError("The configuration option could not be serialized")
        if written != needed:
            raise RuntimeError("The configuration option changed while it was being serialized")
        return out.value[:written].decode("utf-8", errors="replace")

    def vector_empty(self) -> bool:
        vector = self.api.host.config_option_vector_cast(self.c_handle())
        return not vector or bool(self.api.host.config_option_vector_empty(vector))

    def vector_serialize_at(self, idx: int) -> str:
        """Return one vector item's text; raise RuntimeError if serialization fails or its length changes between calls."""
        vector = self.api.host.config_option_vector_cast(self.c_handle())
        if not vector:
            return ""
        needed = int(self.api.host.config_option_vector_serialize_at(vector, int(idx), None, 0))
        if needed == 0xFFFFFFFF:
            raise RuntimeError("The configuration option could not be serialized")
        out = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_option_vector_serialize_at(vector, int(idx), out, len(out)))
        if written == 0xFFFFFFFF:
            raise RuntimeError("The configuration option could not be serialized")
        if written != needed:
            raise RuntimeError("The configuration option changed while it was being serialized")
        return out.value[:written].decode("utf-8", errors="replace")


# Mutable config option. Use only on a MutableConfig result; changing options may
# require the host step to invalidate/recompute dependent data.
class MutableConfigOption(ConfigOption):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def set_enabled(self, enabled: bool, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_set_enabled(self.mutable_c_handle(), int(enabled), int(idx)))

    def set_can_be_disabled(self, enabled: bool) -> bool:
        return bool(self.api.host.config_option_set_can_be_disabled(self.mutable_c_handle(), int(enabled)))

    def set_phony(self, phony: bool) -> bool:
        return bool(self.api.host.config_option_set_phony(self.mutable_c_handle(), int(phony)))

    def copy_from(self, src: ConfigOption) -> None:
        self.api.host.config_option_copy(self.mutable_c_handle(), src.c_handle())

    def deserialize(self, text: str, append: bool = False) -> bool:
        return bool(self.api.host.config_option_deserialize(self.mutable_c_handle(), _as_bytes(text), int(append)))

    def vector_deserialize_at(self, idx: int, text: str) -> bool:
        """
        Replace one existing vector item from its serialized representation.

        Invalid text or indices leave the vector unchanged. This method never
        resizes the option; call ``resize()`` explicitly when that is intended.
        """
        vector = self.api.host.config_option_vector_cast_mutable(self.mutable_c_handle())
        return bool(
            vector
            and self.api.host.config_option_vector_deserialize_at(
                vector, int(idx), _as_bytes(text)
            )
        )

    def set_int(self, value: int, idx: int = 0) -> None:
        self.api.host.config_option_set_int(self.mutable_c_handle(), int(value), int(idx))

    def set_float(self, value: float, idx: int = 0) -> None:
        self.api.host.config_option_set_float(self.mutable_c_handle(), float(value), int(idx))

    def set_float_or_percent(self, value: CFloatOrPercent, idx: int = 0) -> None:
        self.api.host.config_option_set_float_or_percent(self.mutable_c_handle(), value, int(idx))

    def set_bool(self, value: bool, idx: int = 0) -> None:
        self.api.host.config_option_set_bool(self.mutable_c_handle(), int(value), int(idx))

    def set_string(self, value: str, idx: int = 0) -> None:
        self.api.host.config_option_set_string(self.mutable_c_handle(), _as_bytes(value), int(idx))

    def resize(self, size: int) -> None:
        vector = self.api.host.config_option_vector_cast_mutable(self.mutable_c_handle())
        if not vector:
            raise TypeError("Cannot resize a scalar configuration option")
        self.api.host.config_option_vector_resize(vector, int(size), None)

    def clear_vector(self) -> None:
        vector = self.api.host.config_option_vector_cast_mutable(self.mutable_c_handle())
        if not vector:
            raise TypeError("Cannot clear a scalar configuration option as a vector")
        self.api.host.config_option_vector_clear(vector)


# Borrowed read-only config view. get() returns None for an unknown key.
@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h")
class Config(DataTreeView):
    """
    Read-only access to one host-owned configuration.

    ``option(key).get_*()`` performs a direct typed read and is appropriate
    when the option is required. The ``*_or_default`` helpers return their
    fallback when a key is absent, the option or selected item is disabled, or
    a vector index is outside the current option size. Scalar helpers read item
    zero; vector helpers take an explicit item index.
    """

    def keys(self) -> list[str]:
        raw = self.api.host.config_keys(self.c_handle())
        out: list[str] = []
        for idx in range(raw.size):
            out.append(_decode_const_string(raw.items[idx]))
        return out

    def get(self, key: str) -> ConfigOption | None:
        return _optional(ConfigOption, self.api, self.api.host.config_get(self.c_handle(), _as_bytes(key)))

    def has(self, key: str) -> bool:
        """Return whether this configuration currently contains ``key``."""
        return self.get(key) is not None

    def option(self, key: str) -> ConfigOption:
        option = self.get(key)
        if option is None:
            raise KeyError(key)
        return option

    def bool(self, key: str, idx: int = 0, default: bool = False) -> bool:
        """Read legacy numeric slot ``idx``; point slots are flattened X/Y components."""
        option = self._legacy_numeric_option_if_enabled(key, idx)
        return default if option is None else option.get_bool(idx)

    def int(self, key: str, idx: int = 0, default: int = 0) -> int:
        """Read legacy numeric slot ``idx``; point slots are flattened X/Y components."""
        option = self._legacy_numeric_option_if_enabled(key, idx)
        return default if option is None else option.get_int(idx)

    def float(self, key: str, idx: int = 0, default: float = 0.0) -> float:
        """Read legacy numeric slot ``idx``; point slots are flattened X/Y components."""
        option = self._legacy_numeric_option_if_enabled(key, idx)
        return default if option is None else option.get_float(idx)

    def float_or_percent(self, key: str, idx: int = 0) -> CFloatOrPercent | None:
        option = self.get(key)
        return None if option is None else option.get_float_or_percent(idx)

    def bool_or_default(self, key: str, fallback: bool) -> bool:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_bool()

    def vector_bool_or_default(self, key: str, idx: int, fallback: bool) -> bool:
        option = self._vector_option_if_enabled(key, idx)
        return fallback if option is None else option.get_bool(idx)

    def int_or_default(self, key: str, fallback: int) -> int:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_int()

    def vector_int_or_default(self, key: str, idx: int, fallback: int) -> int:
        option = self._vector_option_if_enabled(key, idx)
        return fallback if option is None else option.get_int(idx)

    def enum_or_default(self, key: str, fallback: int) -> int:
        return self.int_or_default(key, fallback)

    def vector_enum_or_default(self, key: str, idx: int, fallback: int) -> int:
        return self.vector_int_or_default(key, idx, fallback)

    def float_or_default(self, key: str, fallback: float) -> float:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_float()

    def vector_float_or_default(
        self, key: str, idx: int, fallback: float
    ) -> float:
        option = self._vector_option_if_enabled(key, idx)
        return fallback if option is None else option.get_float(idx)

    def percent_or_default(self, key: str, fallback: float) -> float:
        return self.float_or_default(key, fallback)

    def vector_percent_or_default(
        self, key: str, idx: int, fallback: float
    ) -> float:
        return self.vector_float_or_default(key, idx, fallback)

    def float_or_percent_or_default(
        self, key: str, fallback: CFloatOrPercent
    ) -> CFloatOrPercent:
        option = self._option_if_enabled(key)
        return (
            CFloatOrPercent(fallback.value, fallback.percent)
            if option is None
            else option.get_float_or_percent()
        )

    def vector_float_or_percent_or_default(
        self, key: str, idx: int, fallback: CFloatOrPercent
    ) -> CFloatOrPercent:
        option = self._vector_option_if_enabled(key, idx)
        return (
            CFloatOrPercent(fallback.value, fallback.percent)
            if option is None
            else option.get_float_or_percent(idx)
        )

    def effective_float_or_percent_or_default(
        self, key: str, ratio: float, fallback: float
    ) -> float:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_effective_value(ratio)

    def vector_effective_float_or_percent_or_default(
        self, key: str, idx: int, ratio: float, fallback: float
    ) -> float:
        option = self._vector_option_if_enabled(key, idx)
        return (
            fallback
            if option is None
            else option.get_effective_value(ratio, idx)
        )

    def computed_float_or_default(
        self, key: str, extruder_id: int, fallback: float
    ) -> float:
        """
        Resolve a numeric setting through the host's ratio dependency chain.

        ``fallback`` is returned when the key or selected vector item is
        absent, disabled, non-numeric, non-finite or otherwise unresolved.
        """
        if not isinstance(extruder_id, int):
            raise TypeError("extruder id must be an integer")
        if extruder_id < EXTRUDER_ID_UNKNOWN or extruder_id > 0x7FFF:
            raise ValueError("extruder id must be UNKNOWN or fit in extruder_index_t")
        output = ctypes.c_double(float(fallback))
        resolved = self.api.host.config_get_computed_value(
            self.c_handle(), _as_bytes(key), int(extruder_id), ctypes.byref(output)
        )
        return float(output.value) if resolved else float(fallback)

    def string_or_default(self, key: str, fallback: str) -> str:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_string()

    def vector_string_or_default(
        self, key: str, idx: int, fallback: str
    ) -> str:
        option = self._vector_option_if_enabled(key, idx)
        return fallback if option is None else option.get_string(idx)

    def point_or_default(
        self, key: str, fallback: ConfigPoint
    ) -> ConfigPoint:
        option = self._option_if_enabled(key)
        return fallback if option is None else option.get_point()

    def vector_point_or_default(
        self, key: str, idx: int, fallback: ConfigPoint
    ) -> ConfigPoint:
        option = self._vector_option_if_enabled(key, idx)
        return fallback if option is None else option.get_point(idx)

    def graph_or_default(
        self, key: str, fallback: GraphData | None
    ) -> GraphData | None:
        option = self._option_if_enabled(key)
        if option is None:
            return fallback
        return option.graph()

    def vector_graph_or_default(
        self, key: str, idx: int, fallback: GraphData | None
    ) -> GraphData | None:
        option = self._vector_option_if_enabled(key, idx)
        if option is None:
            return fallback
        return option.graph(idx)

    def _option_if_enabled(self, key: str) -> ConfigOption | None:
        option = self.get(key)
        if option is None or option.size() == 0 or not option.is_enabled(0):
            return None
        return option

    def _vector_option_if_enabled(
        self, key: str, idx: int
    ) -> ConfigOption | None:
        option = self.get(key)
        if (
            option is None
            or idx < 0
            or idx >= option.size()
            or not option.is_enabled(idx)
        ):
            return None
        return option

    def _legacy_numeric_option_if_enabled(
        self, key: str, idx: int
    ) -> ConfigOption | None:
        """
        Validate the raw numeric index used by bool(), int() and float().

        Those pre-existing shortcuts expose point options as flattened numeric
        components. ConfigOption.size(), however, counts whole points, and the
        enabled state of a points vector is also stored per point. Other vector
        options use the numeric index directly. Other scalar numeric options
        retain their native behavior of ignoring the index after checking item
        zero.
        """
        option = self.get(key)
        if option is None or idx < 0 or option.size() == 0:
            return None

        option_type = option.type()
        if option_type == SLIC3R_CONFIG_OPTION_POINT:
            if idx >= 2:
                return None
            enabled_idx = 0
        elif option_type == SLIC3R_CONFIG_OPTION_POINTS:
            if idx >= option.size() * 2:
                return None
            enabled_idx = idx // 2
        elif option.is_vector():
            if idx >= option.size():
                return None
            enabled_idx = idx
        else:
            enabled_idx = 0

        return option if option.is_enabled(enabled_idx) else None

    def serialize_all(self) -> str:
        """Serialize every option into the host's versioned SCFG format."""
        needed = int(self.api.host.config_serialize_all(self.c_handle(), None, 0))
        if needed == 0:
            raise RuntimeError("The configuration could not be serialized")
        output = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_serialize_all(self.c_handle(), output, len(output)))
        if written != needed:
            raise RuntimeError("The configuration changed while it was being serialized")
        return output.value[:written].decode("utf-8")


# Mutable config view. Only use it when the host explicitly exposes a mutable
# config handle to this plugin step.
class MutableConfig(Config):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def get_mutable(self, key: str) -> MutableConfigOption | None:
        return _optional(MutableConfigOption, self.api, self.api.host.config_get_mutable(self.mutable_c_handle(), _as_bytes(key)))

    def mutable_option(self, key: str) -> MutableConfigOption:
        option = self.get_mutable(key)
        if option is None:
            raise KeyError(key)
        return option

    def get_or_add(self, key: str, option_type: int) -> MutableConfigOption:
        handle = self.api.host.config_get_or_add_mutable(
            self.mutable_c_handle(), _as_bytes(key), int(option_type))
        option = _optional(MutableConfigOption, self.api, handle)
        if option is None:
            raise ValueError(f"Cannot create configuration option {key!r} with type {option_type}")
        return option

    def clear(self) -> None:
        if not self.api.host.config_clear(self.mutable_c_handle()):
            raise RuntimeError("The configuration cannot be cleared")

    def deserialize_all(self, serialized: str) -> None:
        """Atomically merge one SCFG document into this dynamic config."""
        if not self.api.host.config_deserialize_all(self.mutable_c_handle(), _as_bytes(serialized)):
            raise ValueError("The serialized configuration is invalid")


# Owned mutable DynamicConfig allocated in a storage_handle. The storage must
# outlive this object and must not be cleared until close() has released it.
# Assignment shares this wrapper; copy.copy() and copy.deepcopy() are forbidden.
class StoredConfig(_UniqueOwnerMixin, MutableConfig):
    def __init__(self, api, storage) -> None:
        self._storage = _require_handle(storage, "storage")
        handle = api.host.storage_new_config(_void_p(self._storage))
        super().__init__(api, handle)
        self._owns_handle = True

    def close(self) -> bool:
        if not self._owns_handle or not self.address:
            return False
        freed = bool(self.api.host.storage_free(_void_p(self._storage), self.c_handle()))
        if freed:
            self.address = 0
            self._owns_handle = False
        return freed

    def release(self) -> int:
        handle = self.address
        self.address = 0
        self._owns_handle = False
        return handle

    def __enter__(self) -> "StoredConfig":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


def _validated_mesh_slice_z_values(z_mm_by_layer: Iterable[float]) -> list[float]:
    """Materialize finite float-compatible heights in native slicing order."""
    z_values: list[float] = []
    for idx, value in enumerate(z_mm_by_layer):
        try:
            z_value = float(value)
            native_z = ctypes.c_float(z_value).value
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"mesh slice Z value at index {idx} is not a finite number") from error
        if not math.isfinite(z_value) or not math.isfinite(native_z):
            raise ValueError(f"mesh slice Z value at index {idx} must be finite")
        if z_values and z_value < z_values[-1]:
            raise ValueError(
                "mesh slice Z values must be sorted in non-decreasing order "
                f"(index {idx}: {z_value} follows {z_values[-1]})"
            )
        z_values.append(z_value)
    return z_values


def _truncated_scaled(value: float) -> int:
    """
    Convert millimeters to scaled units by truncation toward zero, like the native
    ``scale_i()``. The Python ``scale_i()`` rounds, which is not the same for a distance
    that is not a whole number of scaled units.
    """
    return int(float(value) * UNSCALING_FACTOR)


def _squared_distance_to_segment(ax: int, ay: int, bx: int, by: int, px: int, py: int) -> float:
    """
    Squared distance from the point ``p`` to the segment ``[a, b]``, with the floating
    point operations of the native ``Line::distance_to_squared_abp(a, b, p)``: the
    differences are made on integers then converted, ``x * x`` is a product (never ``**``
    or ``hypot``), and ``s = t / l2`` is applied to ``v`` before the subtraction.
    """
    vx = float(bx - ax)
    vy = float(by - ay)
    vax = float(px - ax)
    vay = float(py - ay)
    l2 = vx * vx + vy * vy
    if l2 == 0.0:
        return vax * vax + vay * vay
    t = vax * vx + vay * vy
    if t <= 0.0:
        return vax * vax + vay * vay
    if t >= l2:
        dx = float(px - bx)
        dy = float(py - by)
        return dx * dx + dy * dy
    s = t / l2
    wx = s * vx - vax
    wy = s * vy - vay
    return wx * wx + wy * wy


def _remove_collinear_points(points: Sequence[tuple[int, int]], max_offset: int) -> list[tuple[int, int]]:
    """
    Port of the native ``Polygon::remove_collinear(max_offset)``: the points of a closed ring
    that are within ``max_offset`` (scaled units) of the segment joining their neighbours are
    deleted. The result is the same points in the same order as the native function, because
    its three passes are reproduced as they are:

    1. while more than 2 points remain, the first point goes if it is closer than
       ``max_offset`` to the segment (last point, second point);
    2. one pass over the remaining points: a point goes if it is closer than ``max_offset``
       to the segment (last point kept, next ORIGINAL point). The last point kept is not
       examined again after a deletion, and the first and last points are never examined;
    3. while more than 2 points remain, the last point goes if it is closer than
       ``max_offset`` to the segment (point before it, first point).

    ``points`` is a sequence of ``(x, y)`` integers; a new list is returned and ``points`` is
    not changed. Rings of fewer than 3 points are returned as they are.
    """
    count = len(points)
    if count < 3:
        return list(points)
    threshold = float(max_offset) * float(max_offset)
    last_x, last_y = points[count - 1]

    first = 0
    while count - first > 2 and _squared_distance_to_segment(
        last_x, last_y, points[first + 1][0], points[first + 1][1], points[first][0], points[first][1]
    ) < threshold:
        first += 1

    kept = [points[first]]
    for idx in range(first + 1, count - 1):
        previous = kept[-1]
        following = points[idx + 1]
        candidate = points[idx]
        if not _squared_distance_to_segment(
            previous[0], previous[1], following[0], following[1], candidate[0], candidate[1]
        ) < threshold:
            kept.append(candidate)
    kept.append(points[count - 1])

    while len(kept) > 2 and _squared_distance_to_segment(
        kept[-2][0], kept[-2][1], kept[0][0], kept[0][1], kept[-1][0], kept[-1][1]
    ) < threshold:
        kept.pop()
    return kept


@abi_requires_headers("slic3r_geometry.h")
def _remove_collinear_rings(api, rings: StoredPolygonCollection, max_offset: int) -> int:
    """Apply ``_remove_collinear_points`` to every ring of the collection; return the deleted point count."""
    removed_total = 0
    for idx in range(rings.size()):
        polygon = api.host.polygons_at(rings.mutable_c_handle(), idx)
        multipoint = _void_p(api.host.polygon_as_multipoint(_void_p(polygon)))
        view = api.host.multipoint_view_const(multipoint)
        count = int(view.size)
        if count < 3:
            continue
        array = view.array
        points = [(int(array[point].x), int(array[point].y)) for point in range(count)]
        kept = _remove_collinear_points(points, max_offset)
        if len(kept) == count:
            continue
        # The ring is rewritten only when a point was deleted.
        api.host.multipoint_clear(multipoint)
        kept_array = (CPoint * len(kept))(*[CPoint(x, y) for x, y in kept])
        api.host.multipoint_insert_array(multipoint, 0, kept_array, len(kept))
        removed_total += count - len(kept)
    return removed_total


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def _finish_sliced_layer(
    api,
    storage_address: int,
    params: CMeshSlicingParams,
    layer: int,
    rings: StoredPolygonCollection,
) -> StoredExPolygonCollection:
    """
    Turn the raw rings of one layer into its final ExPolygons, like the native
    ``slice_mesh_ex()`` followed by the final ``ensure_valid()`` of the host
    (``TriangleMeshSlicer.cpp``): the same steps, in the same order, through the plugin API.
    ``rings`` is modified; intermediate collections are freed, also when an error is raised.
    """
    # Distances in scaled units, truncated like the native scale_i(); every branch tests these integers.
    closing_radius = _truncated_scaled(params.closing_radius)
    extra_offset = _truncated_scaled(params.extra_offset)
    model_resolution = _truncated_scaled(params.model_resolution)
    resolution = _truncated_scaled(params.resolution)
    # The mode of the layer decides the fill rule and the largest-contour filter.
    mode = params.mode_below if layer < params.slicing_mode_normal_below_layer else params.mode

    rings.ensure_valid(SCALED_EPSILON)
    if model_resolution > 0:
        _remove_collinear_rings(api, rings, model_resolution)

    if mode == RAW_MESH_SLICING_MODE_EVEN_ODD:
        fill_rule = CLIPPER_FILL_EVENODD
    elif mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR:
        fill_rule = CLIPPER_FILL_POSITIVE
    else:
        fill_rule = CLIPPER_FILL_NONZERO
    slices = clipper_union_ex(storage_address, rings, fill_rule)
    try:
        # Closing (grow then shrink) and extra offset, in one or two offsets as the native make_expolygons() does.
        if closing_radius >= extra_offset:
            offset_out = float(closing_radius)
            offset_in = -float(closing_radius - extra_offset)
        else:
            offset_out = float(extra_offset)
            offset_in = 0.0
        if offset_out > 0 and offset_in < 0:
            slices = _replace_expolygons(
                slices, clipper_offset2_ex(storage_address, slices, offset_out, offset_in, CLIPPER_JOIN_MITER, 3.0)
            )
        elif offset_out > 0:
            slices = _replace_expolygons(
                slices, clipper_offset_ex(storage_address, slices, offset_out, CLIPPER_JOIN_MITER, 3.0)
            )
        elif offset_in < 0:
            slices = _replace_expolygons(
                slices, clipper_offset_ex(storage_address, slices, offset_in, CLIPPER_JOIN_MITER, 3.0)
            )

        # keep_largest_contour_only(): the island whose contour has the largest signed area, the first on a tie.
        # With no positive contour the islands are left unchanged, as in the native code.
        if mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR and slices.size() > 1:
            max_area = 0.0
            largest = -1
            for idx in range(slices.size()):
                area = slices.at(idx).contour().area()
                if area > max_area:
                    max_area = area
                    largest = idx
            if largest >= 0:
                with slices.extract(largest) as kept:
                    slices.clear()
                    slices.insert_move(kept, 0)

        if resolution != 0:
            # The safety offset (native union_safety_offset_ex) merges the islands that touch.
            slices = _replace_expolygons(
                slices, clipper_offset_ex(storage_address, slices, 10.0, CLIPPER_JOIN_MITER, 3.0)
            )
            slices.ensure_valid(resolution)
        else:
            slices.ensure_valid(SCALED_EPSILON)
        # The host applied one more simplification to the result, with a resolution that can be 0.
        slices.ensure_valid(resolution)
    except BaseException:
        slices.free_from_storage()
        raise
    return slices


def _replace_expolygons(old: StoredExPolygonCollection, new: StoredExPolygonCollection) -> StoredExPolygonCollection:
    """Free the collection that was just replaced by its offset and return the new one."""
    old.free_from_storage()
    return new


@abi_requires_headers("slic3r_volume.h", "slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def _slice_mesh_to_expolygons(
    mesh: "TriangleMesh",
    storage_address: int,
    params: CMeshSlicingParams,
    z_values: list[float],
    context,
) -> list[StoredExPolygonCollection]:
    """
    Slice a mesh to one ExPolygonCollection per Z with the result of the native object slicing.
    The host gives the raw loops, cancellable through the run context (``PositiveLargestContour``
    replaced by ``Positive``, as the native slicer does) and the rest of the post-processing runs
    here, layer by layer, through the API. The context is checked before each layer and after the
    last one.
    """
    view = _context_view(context)
    raw_params = CMeshSlicingParams.from_buffer_copy(params)
    if raw_params.mode == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR:
        raw_params.mode = RAW_MESH_SLICING_MODE_POSITIVE
    if raw_params.mode_below == RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR:
        raw_params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE
    raw = mesh.slice_to_polygons(storage_address, params.transform, z_values, raw_params, view)
    out: list[StoredExPolygonCollection] = []
    try:
        for layer, rings in enumerate(raw):
            throw_if_cancelled(view)
            out.append(_finish_sliced_layer(mesh.api, storage_address, params, layer, rings))
            rings.free_from_storage()
        throw_if_cancelled(view)
    except BaseException:
        for collection in (*raw, *out):
            collection.free_from_storage()
        raise
    return out


@abi_requires_headers("slic3r_volume.h")
class TriangleMesh(DataTreeView):
    """
    Borrowed read-only triangle mesh view.

    Mesh vertices use unscaled millimeters, unlike most 2D geometry in the
    slicing data tree. Slicing plugins usually get this through Volume.mesh().
    """

    def vertex_count(self) -> int:
        return int(self.api.host.triangle_mesh_vertex_count(self.c_handle()))

    def triangle_count(self) -> int:
        return int(self.api.host.triangle_mesh_triangle_count(self.c_handle()))

    def empty(self) -> bool:
        """
        Return whether the borrowed mesh contains no triangles.

        Vertices without triangles do not form printable mesh geometry and are
        therefore considered empty. This check does not validate manifoldness
        or repair the mesh, and it never modifies or frees the host-owned mesh.
        """
        return self.triangle_count() == 0

    def project_to_polygons(
        self, storage_address: int, transform
    ) -> StoredPolygonCollection:
        """Project mesh triangles onto XY into a storage-owned collection."""
        handle = self.api.host.triangle_mesh_project_to_polygons(
            _void_p(storage_address), self.c_handle(), transform
        )
        if not handle:
            raise ValueError("The mesh projection could not be created")
        return StoredPolygonCollection.adopt_owned(
            self.api, storage_address, handle
        )

    def vertex_at(self, idx: int):
        return self.api.host.triangle_mesh_vertex_at(self.c_handle(), int(idx))

    def triangle_at(self, idx: int) -> CTriangleIndices:
        return self.api.host.triangle_mesh_triangle_at(self.c_handle(), int(idx))

    def slice_to_polygons(
        self,
        storage_address: int,
        transform: CMatrix4d,
        z_mm_by_layer: Iterable[float],
        params: CMeshSlicingParams | None = None,
        context=None,
    ) -> list[StoredPolygonCollection]:
        """
        Slice this mesh into one flat PolygonCollection per requested Z.

        ``transform`` and every Z use unscaled model-space millimeters. Convert
        a scaled ``Layer.slice_z()`` with ``unscaled()`` before passing it here.
        Heights must be sorted in non-decreasing order because the native mesh
        slicer performs binary searches over them. Repeated heights are allowed.
        With no ``params``, the host uses regular slicing and the supplied
        transform. When ``params`` is supplied, its transform and slicing modes
        are authoritative; the separate ``transform`` argument is ignored.

        Each returned collection owns a handle allocated in ``storage_address``.
        Keep that storage alive, and call ``free_from_storage()`` on every
        collection, or manage each one with a ``with`` block, before clearing
        the storage. Polygons borrowed from a returned collection become invalid
        when that collection is freed or modified. The method reads the mesh
        only and never changes host model data.

        ``context`` is the run context of the running step: the address of its
        ``PluginRunContext``, a ``PluginRunContextView`` or a ``PluginRunContext``
        structure, or ``None``. The host polls its cancellation callback during
        the slicing, at a coarse granularity and also when there is no layer: a
        cancelled context raises ``PluginCancelled``, an invalid argument (for example a negative
        ``closing_radius`` or a non-finite transform) raises ``ValueError`` and an
        internal host error raises ``RuntimeError``. Every allocation is freed
        before the exception is raised.
        """
        z_values = _validated_mesh_slice_z_values(z_mm_by_layer)
        # Everything that can fail on a bad argument is built before the first allocation.
        if params is None:
            params = CMeshSlicingParams()
            params.transform = transform
        params_pointer = ctypes.byref(params)
        view = _context_view(context)
        context_pointer = None if view is None else ctypes.byref(view.common)
        z_array = (ctypes.c_float * len(z_values))(*z_values)

        output: list[StoredPolygonCollection] = []
        try:
            for _unused in z_values:
                output.append(StoredPolygonCollection(self.api, storage_address))
            output_array = (ctypes.c_void_p * len(output))(
                *[collection.c_handle() for collection in output]
            )
            status = int(
                self.api.host.triangle_mesh_slice_to_polygons(
                    self.c_handle(), params_pointer, z_array, output_array, len(output), context_pointer
                )
            )
        except BaseException:
            for collection in output:
                collection.free_from_storage()
            raise
        if status == RAW_MESH_SLICE_STATUS_DONE:
            return output
        for collection in output:
            collection.free_from_storage()
        if status == RAW_MESH_SLICE_STATUS_CANCELLED:
            raise PluginCancelled()
        if status == RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT:
            raise ValueError("The TriangleMesh slicing arguments are invalid.")
        raise RuntimeError("The TriangleMesh slicing failed inside the host.")

    def slice_to_expolygons(
        self,
        storage_address: int,
        transform: CMatrix4d,
        z_mm_by_layer: Iterable[float],
        params: CMeshSlicingParams | None = None,
        context=None,
    ) -> list[StoredExPolygonCollection]:
        """
        Slice this mesh into topology-aware ExPolygonCollections.

        Coordinate, parameter and ownership rules are identical to
        ``slice_to_polygons()``. Prefer this form when contours and holes must
        remain associated. Every returned collection is allocated in the
        caller's ``storage_address`` and must be released with
        ``free_from_storage()``, or managed with a ``with`` block, before that
        storage is cleared. This read-only operation never changes the borrowed
        mesh.

        The result is the one of the native object slicing: the host gives the
        raw loops and the closing radius, extra offset, model resolution,
        resolution and slicing modes of ``params`` are applied here, through the
        API, layer by layer (the collinear point removal is a port of the native ``Polygon::remove_collinear``). Without
        ``params``, the slicing is regular with ``transform`` and no cleaning
        distance.

        ``context`` and the exceptions are the ones of ``slice_to_polygons()``; the
        context is also checked between two layers.
        """
        z_values = _validated_mesh_slice_z_values(z_mm_by_layer)
        if params is None:
            params = CMeshSlicingParams()
            params.transform = transform
        return _slice_mesh_to_expolygons(self, storage_address, params, z_values, context)


@abi_requires_headers("slic3r_volume.h")
class Volume(DataTreeView):
    """
    Borrowed read-only model volume view.

    Volumes are the raw model inputs used by STEP_SLICING. They expose the mesh,
    transform and volume-level config needed to decide how a slicing plugin
    should write layer-region polygons.
    """

    def type(self) -> int:
        return int(self.api.host.volume_get_type(self.c_handle()))

    def is_model_part(self) -> bool:
        """Return whether this volume contributes normal printable material."""
        return self.type() == RAW_VOLUME_TYPE_MODEL_PART

    def is_negative(self) -> bool:
        """Return whether this volume subtracts material from model parts."""
        return self.type() == RAW_VOLUME_TYPE_NEGATIVE_VOLUME

    def is_modifier(self) -> bool:
        """Return whether this volume changes settings without adding material."""
        return self.type() == RAW_VOLUME_TYPE_PARAMETER_MODIFIER

    def is_support_enforcer(self) -> bool:
        """Return whether this volume marks areas where support is enforced."""
        return self.type() == RAW_VOLUME_TYPE_SUPPORT_ENFORCER

    def is_support_blocker(self) -> bool:
        """Return whether this volume marks areas where support is blocked."""
        return self.type() == RAW_VOLUME_TYPE_SUPPORT_BLOCKER

    def is_support_modifier(self) -> bool:
        """Return whether this volume adds or removes object surfaces that ask for support."""
        return self.is_support_enforcer() or self.is_support_blocker()

    def is_support_column(self) -> bool:
        """Return whether this volume asks for sparse support inside it."""
        return self.type() == RAW_VOLUME_TYPE_SUPPORT_COLUMN

    def is_no_support_zone(self) -> bool:
        """Return whether this volume forbids support inside it."""
        return self.type() == RAW_VOLUME_TYPE_NO_SUPPORT_ZONE

    def is_support_geometry_modifier(self) -> bool:
        """Return whether this volume adds to or removes from the support itself, not from its demand."""
        return self.is_support_column() or self.is_no_support_zone()

    def is_seam_position(self) -> bool:
        """
        Return whether this volume is one of the seam-position helper types.

        This classification only describes the volume's immutable model role;
        it does not calculate a seam or inspect painted seam facets.
        """
        return self.type() in {
            RAW_VOLUME_TYPE_SEAM_POSITION_CENTER,
            RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z,
            RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER,
            RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE,
        }

    def is_brim(self) -> bool:
        """Return whether this is a positive or negative brim-control volume."""
        return self.type() in {RAW_VOLUME_TYPE_BRIM_PATCH, RAW_VOLUME_TYPE_BRIM_NEGATIVE}

    def id(self) -> int:
        return int(self.api.host.volume_get_id(self.c_handle()))

    def config(self) -> Config:
        return Config(self.api, self.api.host.volume_get_config(self.c_handle()))

    def extruder_id(self) -> int:
        return int(self.api.host.volume_get_extruder_id(self.c_handle()))

    def matrix(self) -> CMatrix4d:
        return self.api.host.volume_get_matrix(self.c_handle())

    def matrix_no_offset(self) -> CMatrix4d:
        return self.api.host.volume_get_matrix_no_offset(self.c_handle())

    def has_painting(self, paint_key: str) -> bool:
        """
        Return whether this borrowed volume has facets for ``paint_key``.

        The key must be a built-in painting key or a generic annotation key
        registered with the orchestrator. This method reads host-owned facet
        metadata and neither copies nor modifies it.
        """
        return bool(self.api.host.volume_has_painting(self.c_handle(), _as_bytes(paint_key)))

    def is_fdm_support_painted(self) -> bool:
        """Return whether the volume contains FDM support painting."""
        return self.has_painting(RAW_FACET_PAINTING_FDM_SUPPORT)

    def is_seam_painted(self) -> bool:
        """Return whether the volume contains seam painting."""
        return self.has_painting(RAW_FACET_PAINTING_SEAM)

    def is_mm_painted(self) -> bool:
        """Return whether the volume contains multi-material painting."""
        return self.has_painting(RAW_FACET_PAINTING_MMU_SEGMENTATION)

    def mesh(self) -> TriangleMesh:
        """
        Return a borrowed, read-only view of this volume's triangle mesh.

        The mesh remains owned by the host and is valid only while this volume
        and its model object remain alive. Do not free the returned handle.
        """
        return TriangleMesh(self.api, self.api.host.volume_get_mesh(self.c_handle()))


# Borrowed print-region view.
class PrintRegion(DataTreeView):
    def config(self) -> Config:
        return Config(self.api, self.api.host.print_region_get_config(self.c_handle()))


# Borrowed layer-region view.
class LayerRegion(DataTreeView):
    @abi_requires_headers("slic3r_data_tree.h")
    def infill_bridge_flow_mm(self, role: int) -> CFlowMm:
        """Return bridge dimensions in mm before coordinate quantization.

        The role selects the nozzle; flow-based mode uses solid-infill area.
        A host resolution failure returns zeroed fields without changing the region.
        """
        return self.api.host.layer_region_get_infill_bridge_flow_mm(self.c_handle(), int(role))

    @abi_requires_headers("slic3r_data_tree.h")
    def infill_bridge_flow(self, role: int) -> CFlow:
        """Return native area and scaled recipe dimensions, including layer-grid height.

        The role follows infill_bridge_flow_mm(); host resolution failure returns zeros.
        """
        return self.api.host.layer_region_get_infill_bridge_flow(self.c_handle(), int(role))

    @abi_requires_headers("slic3r_data_tree.h")
    def role_extruder(self, role: int) -> int:
        """Return this role's zero-based tool index, or EXTRUDER_ID_UNKNOWN on failure."""
        return int(self.api.host.layer_region_get_role_extruder(self.c_handle(), int(role)))

    def flow(self, role: int) -> CFlow:
        return self.api.host.layer_region_get_flow(self.c_handle(), int(role))

    @abi_requires_headers("slic3r_data_tree.h")
    def bridging_flow(self, role: int) -> CFlow:
        """
        Return the bridge/overhang flow for ``role`` on this layer region.

        The native region remains host-owned and unmodified. The returned
        ``CFlow`` is copied by value and therefore requires no explicit free.
        """
        return self.api.host.layer_region_get_bridging_flow(
            self.c_handle(), int(role)
        )

    def properties(self) -> ConstPluginProperties:
        return ConstPluginProperties(self.api, self.api.host.layer_region_get_properties(self.c_handle()))

    def mutable_properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_region_get_properties_mutable(self.c_handle()))

    def slices(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_region_get_slices(self.c_handle()))

    def bounding_box(self):
        return self.api.host.layer_region_get_bounding_box(self.c_handle())

    def print_region(self) -> PrintRegion:
        return PrintRegion(self.api, self.api.host.layer_region_get_print_region(self.c_handle()))

    def layer(self) -> "Layer":
        return Layer(self.api, self.api.host.layer_region_get_layer(self.c_handle()))


class MutableLayerRegion(LayerRegion):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()


# Borrowed layer-region island view. It is the usual entry point for extrusion
# trees attached to one island/region pair.
class LayerRegionIsland(DataTreeView):
    def extruder_id(self) -> int:
        return int(self.api.host.layer_region_island_extruder_id(self.c_handle()))

    def properties(self) -> ConstPluginProperties:
        return ConstPluginProperties(self.api, self.api.host.layer_region_island_get_properties(self.c_handle()))

    def mutable_properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_region_island_get_properties_mutable(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def region_count(self) -> int:
        """Return the number of borrowed LayerRegion views in this group."""
        return int(self.api.host.layer_region_island_count_region(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def region(self, idx: int) -> LayerRegion:
        """
        Borrow one region by its current group-local index.

        The returned view is owned by the host data tree. Do not free it or
        retain it after the current tree mutation invalidates region handles.
        """
        count = self.region_count()
        if idx < 0 or idx >= count:
            raise IndexError(f"layer-region index {idx} is outside [0, {count})")
        return LayerRegion(
            self.api,
            self.api.host.layer_region_island_get_region(self.c_handle(), int(idx)),
        )

    @abi_requires_headers("slic3r_data_tree.h")
    def regions(self) -> Iterator[LayerRegion]:
        """Iterate borrowed regions in their current host-defined order."""
        for idx in range(self.region_count()):
            yield self.region(idx)

    def has_extrusions(self) -> bool:
        return bool(self.api.host.layer_region_island_has_extrusions(self.c_handle()))

    def has_extrusion(self, role: int) -> bool:
        return bool(self.api.host.layer_region_island_has_extrusion(self.c_handle(), int(role)))

    def extrusion(self, role: int) -> ExtrusionEntity | None:
        return _optional(ExtrusionEntity, self.api, self.api.host.layer_region_island_get_extrusion(self.c_handle(), int(role)))

    def fill_surfaces_collection(self) -> SurfaceCollection:
        return SurfaceCollection(self.api, self.api.host.layer_region_island_get_fill_surfaces(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def set_fill_surfaces(self, surfaces: StoredSurfaceCollection | None = None) -> bool:
        """Move temporary surfaces into an auxiliary layer, or clear with None; rejected edits return False."""
        return bool(self.api.host.layer_region_island_set_fill_surfaces(
            self.c_handle(), None if surfaces is None else surfaces.mutable_c_handle()))

    def fill_surface_count(self) -> int:
        return int(self.api.host.layer_region_island_count_fill_surface(self.c_handle()))

    def fill_surface(self, idx: int) -> Surface:
        return Surface(self.api, self.api.host.layer_region_island_get_fill_surface(self.c_handle(), int(idx)))

    def fill_surfaces(self) -> list[Surface]:
        return [self.fill_surface(idx) for idx in range(self.fill_surface_count())]


class MutableLayerRegionIsland(LayerRegionIsland):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def extrusion_mutable(self, role: int) -> MutableExtrusionEntity | None:
        return _optional(
            MutableExtrusionEntity,
            self.api,
            self.api.host.layer_region_island_get_mutable_extrusion(self.mutable_c_handle(), int(role)),
        )


# Borrowed layer island view.
class LayerIsland(DataTreeView):
    def slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_slice(self.c_handle()))

    def properties(self) -> ConstPluginProperties:
        return ConstPluginProperties(self.api, self.api.host.layer_island_get_properties(self.c_handle()))

    def mutable_properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_island_get_properties_mutable(self.c_handle()))

    def bounding_box(self):
        return self.api.host.layer_island_get_bounding_box(self.c_handle())

    def infill_slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_infill_slice(self.c_handle()))

    def infill_areas(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_infill_areas(self.c_handle()))

    def perimeter_slices(self) -> ExPolygonCollection:
        """Borrow filled XY areas published by perimeter processing for travel boundaries.

        These are not extrusion paths and may be empty. See the C function
        layer_island_get_perimeter_slices for publication, lifetime and thread rules.
        """
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_perimeter_slices(self.c_handle()))

    def infill_bounding_box(self):
        return self.api.host.layer_island_get_infill_bounding_box(self.c_handle())

    def infill_no_overlap_slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_infill_no_overlap_slice(self.c_handle()))

    def infill_no_overlap_areas(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_infill_no_overlap_areas(self.c_handle()))

    def support_demand(self) -> ExPolygonCollection:
        """Borrow the areas of this island that ask for support (empty before STEP_SUPPORT_DEMAND)."""
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_support_demand(self.c_handle()))

    def region_count(self) -> int:
        return int(self.api.host.layer_island_count_region(self.c_handle()))

    def region(self, idx: int) -> LayerRegion:
        return LayerRegion(self.api, self.api.host.layer_island_get_region(self.c_handle(), int(idx)))

    def regions(self) -> Iterator[LayerRegion]:
        for idx in range(self.region_count()):
            yield self.region(idx)

    def region_island_count(self) -> int:
        return int(self.api.host.layer_island_count_region_island(self.c_handle()))

    def region_island(self, idx: int) -> LayerRegionIsland:
        return LayerRegionIsland(self.api, self.api.host.layer_island_get_region_island(self.c_handle(), int(idx)))

    def region_islands(self) -> Iterator[LayerRegionIsland]:
        for idx in range(self.region_island_count()):
            yield self.region_island(idx)

    def get_or_create_region_island(
        self,
        regions: Sequence[LayerRegion],
        extruder_id: int = -1,
    ) -> LayerRegionIsland | None:
        """
        Return the LayerRegionIsland for this island, region list, and extruder.

        The caller must already know which extruder should own the output.
        Passing an empty region list means "all regions of this island", just
        like get_or_create_full_region_island().
        """
        if not isinstance(extruder_id, int):
            raise TypeError("extruder id must be an integer")
        if extruder_id < EXTRUDER_ID_UNKNOWN or extruder_id > 0x7FFF:
            raise ValueError("extruder id must be UNKNOWN or fit in extruder_index_t")
        region_addresses = [_address(region.c_handle()) for region in regions]
        region_array = None
        if region_addresses:
            region_array = (ctypes.c_void_p * len(region_addresses))(*region_addresses)

        handle = self.api.host.layer_island_get_or_create_region_island(
            self.c_handle(),
            region_array,
            len(region_addresses),
            int(extruder_id),
        )
        return _optional(LayerRegionIsland, self.api, handle)

    def get_or_create_full_region_island(self, extruder_id: int = -1) -> LayerRegionIsland | None:
        """
        Return the LayerRegionIsland that represents every region on this island.

        Use this when the plugin output is not split by region settings. The
        extruder is still explicit, because the data-tree API does not infer it
        from an extrusion role.
        """
        if not isinstance(extruder_id, int):
            raise TypeError("extruder id must be an integer")
        if extruder_id < EXTRUDER_ID_UNKNOWN or extruder_id > 0x7FFF:
            raise ValueError("extruder id must be UNKNOWN or fit in extruder_index_t")
        handle = self.api.host.layer_island_get_or_create_region_island(
            self.c_handle(),
            None,
            0,
            int(extruder_id),
        )
        return _optional(LayerRegionIsland, self.api, handle)

    def layer(self) -> "Layer":
        return Layer(self.api, self.api.host.layer_island_get_layer(self.c_handle()))

    def lower_island_count(self) -> int:
        return int(self.api.host.layer_island_count_lower_island(self.c_handle()))

    def lower_island(self, idx: int) -> "LayerIsland":
        return LayerIsland(self.api, self.api.host.layer_island_get_lower_island(self.c_handle(), int(idx)))

    def lower_islands(self) -> Iterator["LayerIsland"]:
        for idx in range(self.lower_island_count()):
            yield self.lower_island(idx)

    def upper_island_count(self) -> int:
        return int(self.api.host.layer_island_count_upper_island(self.c_handle()))

    def upper_island(self, idx: int) -> "LayerIsland":
        return LayerIsland(self.api, self.api.host.layer_island_get_upper_island(self.c_handle(), int(idx)))

    def upper_islands(self) -> Iterator["LayerIsland"]:
        for idx in range(self.upper_island_count()):
            yield self.upper_island(idx)


class MutableLayerIsland(LayerIsland):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def slice_mutable(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_slice_mutable(self.mutable_c_handle()))

    def region_mutable(self, idx: int) -> MutableLayerRegion:
        return MutableLayerRegion(self.api, self.api.host.layer_island_get_region_mutable(self.mutable_c_handle(), int(idx)))

    def region_island_mutable(self, idx: int) -> MutableLayerRegionIsland:
        return MutableLayerRegionIsland(
            self.api,
            self.api.host.layer_island_get_region_island_mutable(self.mutable_c_handle(), int(idx)),
        )


# Borrowed layer view.
class Layer(DataTreeView):
    def height(self) -> int:
        return int(self.api.host.layer_get_height(self.c_handle()))

    def print_z(self) -> int:
        return int(self.api.host.layer_get_print_z(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def bottom_z(self) -> int:
        """Return ``print_z - height`` in scaled coordinates."""
        return self.print_z() - self.height()

    def slice_z(self) -> int:
        return int(self.api.host.layer_get_slice_z(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def object(self) -> "Object":
        """
        Borrow the Object that owns this normal or auxiliary layer.

        The Object remains host-owned. Do not free it, and do not retain the
        view after its Print or owning object has been destroyed.
        """
        return Object(self.api, self.api.host.layer_get_object(self.c_handle()))

    def properties(self) -> ConstPluginProperties:
        return ConstPluginProperties(self.api, self.api.host.layer_get_properties(self.c_handle()))

    def mutable_properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_get_properties_mutable(self.c_handle()))

    def upper_layer(self) -> "Layer | None":
        return _optional(Layer, self.api, self.api.host.layer_get_upper_layer(self.c_handle()))

    def lower_layer(self) -> "Layer | None":
        return _optional(Layer, self.api, self.api.host.layer_get_lower_layer(self.c_handle()))

    def region_count(self) -> int:
        return int(self.api.host.layer_count_region(self.c_handle()))

    def island_count(self) -> int:
        return int(self.api.host.layer_count_island(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def curled_line_count(self) -> int:
        """Return the number of host curl-estimation segments on this layer."""
        return int(self.api.host.layer_count_curled_line(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def curled_line(self, idx: int) -> CCurledLine:
        """
        Copy one curled-line estimate by index.

        The returned ctypes structure owns no native allocation and needs no
        explicit free. Its points use scaled XY coordinates.
        """
        count = self.curled_line_count()
        if idx < 0 or idx >= count:
            raise IndexError(f"curled-line index {idx} is outside [0, {count})")
        return self.api.host.layer_get_curled_line(self.c_handle(), int(idx))

    @abi_requires_headers("slic3r_data_tree.h")
    def curled_lines(self) -> list[CCurledLine]:
        """Return independent value snapshots of all curled-line estimates."""
        return [self.curled_line(idx) for idx in range(self.curled_line_count())]

    def slices(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_get_slices(self.c_handle()))

    def region(self, idx: int) -> LayerRegion:
        return LayerRegion(self.api, self.api.host.layer_get_region(self.c_handle(), int(idx)))

    def regions(self) -> Iterator[LayerRegion]:
        for idx in range(self.region_count()):
            yield self.region(idx)

    def island(self, idx: int) -> LayerIsland:
        return LayerIsland(self.api, self.api.host.layer_get_island(self.c_handle(), int(idx)))

    def islands(self) -> Iterator[LayerIsland]:
        for idx in range(self.island_count()):
            yield self.island(idx)


class MutableLayer(Layer):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def upper_layer_mutable(self) -> "MutableLayer | None":
        return _optional(MutableLayer, self.api, self.api.host.layer_get_upper_layer_mutable(self.mutable_c_handle()))

    def lower_layer_mutable(self) -> "MutableLayer | None":
        return _optional(MutableLayer, self.api, self.api.host.layer_get_lower_layer_mutable(self.mutable_c_handle()))

    def region_mutable(self, idx: int) -> MutableLayerRegion:
        return MutableLayerRegion(self.api, self.api.host.layer_get_region_mutable(self.mutable_c_handle(), int(idx)))

    def island_mutable(self, idx: int) -> MutableLayerIsland:
        return MutableLayerIsland(self.api, self.api.host.layer_get_island_mutable(self.mutable_c_handle(), int(idx)))


# Borrowed print object view.
class Object(DataTreeView):
    def name(self) -> str:
        """Return the UTF-8 model-object name copied from the host."""
        required = int(self.api.host.object_get_name(self.c_handle(), None, 0))
        if required == 0:
            return ""
        output = ctypes.create_string_buffer(required + 1)
        self.api.host.object_get_name(self.c_handle(), output, len(output))
        return output.raw[:required].decode("utf-8", errors="replace")

    def properties(self) -> ConstPluginProperties:
        return ConstPluginProperties(self.api, self.api.host.object_get_properties(self.c_handle()))

    def mutable_properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.object_get_properties_mutable(self.c_handle()))

    def config(self) -> Config:
        return Config(self.api, self.api.host.object_get_config(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def default_region_config(self, print_view: "Print") -> Config:
        """Borrow this object's regional defaults using the host's fallback rule.

        print_view must own this object. Without regions, return its Print's
        defaults. Read only while apply()/clear() are stopped; reacquire after apply().
        """
        return Config(self.api, self.api.host.print_resolve_default_region_config(
            print_view.c_handle(), self.c_handle()))

    def max_z(self) -> int:
        return int(self.api.host.object_get_max_z(self.c_handle()))

    def transform(self) -> CMatrix4d:
        return self.api.host.object_get_transform(self.c_handle())

    def center_offset(self) -> CPoint:
        return self.api.host.object_get_center_offset(self.c_handle())

    def transform_centered(self) -> CMatrix4d:
        offset = self.center_offset()
        translation = self.api.host.matrix4d_translation(-unscaled(offset.x), -unscaled(offset.y), 0.0)
        return self.api.host.matrix4d_mul(translation, self.transform())

    @abi_requires_headers("slic3r_data_tree.h")
    def instance_count(self) -> int:
        """Return the number of physical instances of this printable object."""
        return int(self.api.host.object_count_instance(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def instance_shift(self, idx: int) -> CPoint:
        """
        Copy the scaled XY translation of one object instance.

        Extrusion trees are object-local; add this shift when cloning them for
        a physical instance. The returned point owns no native memory. Instance
        indices are stable only for the lifetime of the current Print object.
        """
        count = self.instance_count()
        if idx < 0 or idx >= count:
            raise IndexError(f"instance index {idx} is outside [0, {count})")
        return self.api.host.object_get_instance_shift(self.c_handle(), int(idx))

    def layer_count(self) -> int:
        return int(self.api.host.object_count_layer(self.c_handle()))

    def layer(self, idx: int) -> Layer:
        return Layer(self.api, self.api.host.object_get_layer(self.c_handle(), int(idx)))

    def layers(self) -> Iterator[Layer]:
        for idx in range(self.layer_count()):
            yield self.layer(idx)

    def auxiliary_layer_count(self) -> int:
        return int(self.api.host.object_count_auxiliary_layer(self.c_handle()))

    def auxiliary_layer(self, idx: int) -> Layer:
        return Layer(self.api, self.api.host.object_get_auxiliary_layer(self.c_handle(), int(idx)))

    def auxiliary_layers(self) -> Iterator[Layer]:
        for idx in range(self.auxiliary_layer_count()):
            yield self.auxiliary_layer(idx)

    def add_auxiliary_layer(self, height: int, print_z: int, slice_z: int) -> "MutableLayer":
        """
        Create an empty plugin-mutable auxiliary layer.

        The returned layer has the object's LayerRegion list but no raw slices.
        Write region slices, then call the relevant recompute callbacks from
        the step/helper that owns the geometry.
        """
        return MutableLayer(
            self.api,
            self.api.host.object_add_auxiliary_layer(
                self.c_handle(),
                int(height),
                int(print_z),
                int(slice_z),
            ),
        )

    def remove_auxiliary_layer(self, layer: Layer) -> bool:
        """
        Remove an auxiliary layer previously obtained from this Object.

        Normal object layers are not accepted by the host. The method returns
        False if the handle is null or does not belong to the auxiliary layer
        vector.
        """
        return bool(self.api.host.object_remove_auxiliary_layer(self.c_handle(), layer.c_handle()))

    def volume_count(self) -> int:
        return int(self.api.host.object_volume_count(self.c_handle()))

    def volume(self, idx: int) -> Volume:
        return Volume(self.api, self.api.host.object_volume_at(self.c_handle(), int(idx)))

    def volumes(self) -> Iterator[Volume]:
        for idx in range(self.volume_count()):
            yield self.volume(idx)

    def print_region_count(self) -> int:
        return int(self.api.host.object_count_region(self.c_handle()))

    def print_region(self, idx: int) -> PrintRegion:
        return PrintRegion(self.api, self.api.host.object_get_print_region(self.c_handle(), int(idx)))

    def print_regions(self) -> Iterator[PrintRegion]:
        for idx in range(self.print_region_count()):
            yield self.print_region(idx)

    @abi_requires_headers("slic3r_data_tree.h")
    def support_flow(
        self, role: int, flow_height_mm: float, print_height_mm: float = 0.0, flags: int = 0
    ) -> CFlow:
        """
        Return a support flow of this object at any height, computed by the host.

        ``flags`` combines the ``OBJECT_SUPPORT_FLOW_*`` constants; see
        ``object_get_support_flow()`` in the C header for the rules. A flow
        the host cannot compute has all fields zero. The ``CFlow`` is a copy.
        """
        return self.api.host.object_get_support_flow(
            self.c_handle(), int(role), float(flow_height_mm), float(print_height_mm), int(flags)
        )

    @abi_requires_headers("slic3r_data_tree.h")
    def region_flow(self, region_idx: int, role: int, height_mm: float, layer_id: int) -> CFlow:
        """
        Return the perimeter or infill flow of one print region at any height.

        ``layer_id`` selects the per-layer width rules of the native region
        flow (0 first layer, odd ids the odd-layer change). Invalid input
        gives a zero flow. The ``CFlow`` is a copy.
        """
        return self.api.host.object_get_region_flow(
            self.c_handle(), int(region_idx), int(role), float(height_mm), int(layer_id)
        )
    @abi_requires_headers("slic3r_data_tree.h")
    def support_flow_mm(
        self, role: int, flow_height_mm: float, print_height_mm: float = 0.0, flags: int = 0
    ) -> CFlowMm:
        """
        Return the same flow as ``support_flow()`` in the host's millimeters.

        Use it when a computation must match the host exactly; the scaled
        integers of ``CFlow`` are truncated. A flow the host cannot compute
        has all fields zero. The ``CFlowMm`` is a copy.
        """
        return self.api.host.object_get_support_flow_mm(
            self.c_handle(), int(role), float(flow_height_mm), float(print_height_mm), int(flags)
        )

    @abi_requires_headers("slic3r_data_tree.h")
    def region_flow_mm(self, region_idx: int, role: int, height_mm: float, layer_id: int) -> CFlowMm:
        """Return the same flow as ``region_flow()`` in the host's millimeters."""
        return self.api.host.object_get_region_flow_mm(
            self.c_handle(), int(region_idx), int(role), float(height_mm), int(layer_id)
        )


@abi_requires_headers("slic3r_volume.h")
def project_painting_to_polygons(
    storage_address: int,
    obj: Object,
    paint_key: str,
    painting_value: int,
) -> list[StoredPolygonCollection]:
    """
    Project one facet-painting value from an object onto all of its layers.

    ``paint_key`` may name a built-in painting channel or a generic facets
    annotation registered with the orchestrator. For support and seam channels,
    ``painting_value`` is normally an enforcer or blocker value; for MMU
    painting it is a one-based extruder index. This low-level helper performs
    geometric projection only and does not apply later strategy-specific
    refinements such as full MMU segmentation.

    The object and its model volumes are borrowed, read-only host data. Each
    returned collection is newly allocated in ``storage_address`` and belongs
    to the caller: keep the storage alive and release each collection with
    ``free_from_storage()``, or manage it with a ``with`` block. Borrowed
    polygons obtained from a collection become invalid if that collection is
    modified or freed. The result contains one collection per current object
    layer, including empty collections for layers without matching painted
    facets.
    """
    layer_count = obj.layer_count()
    output = [
        StoredPolygonCollection(obj.api, storage_address)
        for _unused in range(layer_count)
    ]
    if not output:
        return []

    output_array = (ctypes.c_void_p * layer_count)(
        *[collection.c_handle() for collection in output]
    )
    obj.api.host.object_project_painting_to_polygons(
        obj.c_handle(),
        _as_bytes(paint_key),
        int(painting_value),
        output_array,
        layer_count,
    )
    return output


class MutableObject(Object):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def config_mutable(self) -> MutableConfig:
        return MutableConfig(self.api, self.api.host.object_get_config_mutable(self.mutable_c_handle()))

    def layer_mutable(self, idx: int) -> MutableLayer:
        return MutableLayer(self.api, self.api.host.object_get_layer_mutable(self.mutable_c_handle(), int(idx)))

# Borrowed print view.
class Print(DataTreeView):
    def config(self) -> Config:
        return Config(self.api, self.api.host.print_get_config(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def default_region_config(self) -> Config:
        """Borrow preset regional defaults, without creating an auxiliary object.

        Read only while apply()/clear() are stopped; reacquire after apply().
        The Print retains ownership and neither geometry nor settings are changed.
        """
        return Config(self.api, self.api.host.print_get_default_region_config(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def default_object_config(self) -> Config:
        """Borrow preset object settings, including global wipe tower settings.

        This query creates nothing and changes no settings. Read only while
        apply()/clear() are stopped; reacquire the borrowed view after apply().
        """
        return Config(self.api, self.api.host.print_get_default_object_config(self.c_handle()))

    @abi_requires_headers("slic3r_data_tree.h")
    def register_print_region_config(self, config: Config) -> PrintRegion:
        """Copy regional overrides onto this Print's current regional defaults.

        Return a canonical immutable borrowed region, valid across apply() and
        invalidation until Print.clear()/destruction. Never free it. Later source
        or default changes do not affect the copy. Does not attach any geometry.
        Concurrent registration is supported while defaults are stable; workers
        must stop before apply()/clear(). Invalid keys/types raise ValueError.
        """
        handle = self.api.host.print_register_print_region_config(self.c_handle(), config.c_handle())
        if not handle:
            raise ValueError("Cannot register the regional configuration")
        return PrintRegion(self.api, handle)

    def records(self) -> "PrintRecords":
        return PrintRecords(self.api, self.c_handle())

    def object_count(self) -> int:
        return int(self.api.host.print_count_object(self.c_handle()))

    def object(self, idx: int) -> Object:
        return Object(self.api, self.api.host.print_get_object(self.c_handle(), int(idx)))

    def objects(self) -> Iterator[Object]:
        for idx in range(self.object_count()):
            yield self.object(idx)

    def auxiliary_object(self) -> Object:
        """Hidden object that owns print-level auxiliary layers."""
        return Object(self.api, self.api.host.print_get_auxiliary_object(self.c_handle()))


class MutablePrint(Print):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def config_mutable(self) -> MutableConfig:
        return MutableConfig(self.api, self.api.host.print_get_config_mutable(self.mutable_c_handle()))

    def object_mutable(self, idx: int) -> MutableObject:
        return MutableObject(self.api, self.api.host.print_get_object_mutable(self.mutable_c_handle(), int(idx)))


class PrintRecords(DataTreeView):
    """
    Borrowed access to variable Config channels owned by one Print.

    Mutations are only valid from sequential pipeline callbacks. Configs may be
    read concurrently once their producer has finished publishing the channel.
    """

    def channels(self) -> list[str]:
        names = self.api.host.print_records_channels(self.c_handle())
        return [_decode_const_string(names.items[idx]) for idx in range(int(names.size))]

    def get(self, channel: str) -> Config | None:
        handle = self.api.host.print_records_get(self.c_handle(), _as_bytes(channel))
        return _optional(Config, self.api, handle)

    def get_or_add(self, channel: str) -> MutableConfig:
        handle = self.api.host.print_records_get_or_add(self.c_handle(), _as_bytes(channel))
        if not _address(handle):
            raise RuntimeError("The Print record channel could not be created.")
        return MutableConfig(self.api, handle)

    def remove(self, channel: str) -> bool:
        return bool(self.api.host.print_records_remove(self.c_handle(), _as_bytes(channel)))

    def allocate_id(self) -> int:
        record_id = int(self.api.host.print_records_allocate_id(self.c_handle()))
        if record_id == 0:
            raise RuntimeError("The Print record identifier space is exhausted.")
        return record_id


class AuxiliaryLayerBuildResult(NamedTuple):
    """Result of building one host-owned auxiliary layer."""

    layer: Layer | None
    created: bool


# Ported from src/libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.cpp
# at commit cca788f1cecab755cf623cb53d5e6298b4f6b6c4 (05/09/2026).
# This is an algorithm snapshot. Compare it with the cited source whenever the
# C++ region-classification rules change.
def _auxiliary_layer_range_for_z(obj: Object, slice_z: int):
    """Select the active range; a shared boundary belongs to the upper range."""
    selected = None
    range_count = int(obj.api.host.object_count_slicing_layer_range(obj.c_handle()))
    for range_idx in range(range_count):
        layer_range = obj.api.host.object_get_slicing_layer_range(obj.c_handle(), range_idx)
        if not layer_range:
            continue
        z_min = int(obj.api.host.slicing_layer_range_get_z_min(layer_range))
        z_max = int(obj.api.host.slicing_layer_range_get_z_max(layer_range))
        if z_min <= slice_z <= z_max:
            selected = layer_range
            if slice_z < z_max:
                break
    return selected


def _auxiliary_mask_slicing_params(print_view: Print, obj: Object) -> CMeshSlicingParams:
    """Build native-equivalent parameters for settings masks without XY growth."""
    print_config = print_view.config()
    object_config = obj.config()
    slicing_mode = object_config.enum_or_default("slicing_mode", 0)

    params = CMeshSlicingParams()
    if slicing_mode == 1:
        params.mode = RAW_MESH_SLICING_MODE_EVEN_ODD
    elif slicing_mode == 2:
        params.mode = RAW_MESH_SLICING_MODE_POSITIVE
    else:
        params.mode = RAW_MESH_SLICING_MODE_REGULAR
    params.mode_below = params.mode
    params.transform = obj.transform_centered()
    params.closing_radius = float(object_config.float_or_default("slice_closing_radius", 0.0))
    params.extra_offset = 0.0
    params.resolution = float(print_config.float_or_default("resolution", EPSILON))
    params.model_resolution = float(object_config.float_or_default("model_precision", 0.0))
    return params


def _auxiliary_slice_volume_mask(
    storage_address: int,
    print_view: Print,
    obj: Object,
    volume_region,
    subject: ExPolygonCollection,
    slice_z: int,
) -> StoredExPolygonCollection:
    """Return one storage-owned model-part/modifier mask clipped to the subject."""
    empty = StoredExPolygonCollection(obj.api, storage_address)
    if subject.empty():
        return empty

    z_mm = unscaled(slice_z)
    bbox = obj.api.host.slicing_volume_region_get_bbox(volume_region)
    if bbox.min.z > z_mm + EPSILON or bbox.max.z < z_mm - EPSILON:
        return empty

    volume_handle = obj.api.host.slicing_volume_region_get_volume(volume_region)
    if not volume_handle:
        return empty
    volume = Volume(obj.api, volume_handle)
    if not volume.is_model_part() and not volume.is_modifier():
        return empty

    params = _auxiliary_mask_slicing_params(print_view, obj)
    # No run context here: the helper slices one Z of one volume, so it has nothing worth cancelling.
    slices = volume.mesh().slice_to_expolygons(
        storage_address,
        params.transform,
        [z_mm],
        params,
    )
    if not slices or slices[0].empty():
        return empty

    clipper = ClipperContext(obj.api, storage_address)
    with clipper(slices[0]) as mask_operand, clipper(subject) as subject_operand:
        with clipper.intersection(mask_operand, subject_operand) as clipped:
            return clipped.to_expolygon_collection()


def _auxiliary_assign_regions_from_masks(
    storage_address: int,
    print_view: Print,
    obj: Object,
    layer: MutableLayer,
    subject: ExPolygonCollection,
    slice_z: int,
) -> None:
    """Publish disjoint region slices and rebuild one new layer's derived islands."""
    region_count = obj.print_region_count()
    if region_count == 0:
        return

    region_slices = [
        StoredExPolygonCollection(obj.api, storage_address)
        for _unused in range(region_count)
    ]
    region_slices[0].copy_from(subject)

    layer_range = _auxiliary_layer_range_for_z(obj, slice_z)
    volume_region_count = int(
        obj.api.host.slicing_layer_range_count_volume_region(layer_range)
    )
    effective_masks = [
        StoredExPolygonCollection(obj.api, storage_address)
        for _unused in range(volume_region_count)
    ]
    clipper = ClipperContext(obj.api, storage_address)

    for volume_region_idx in range(volume_region_count):
        volume_region = obj.api.host.slicing_layer_range_get_volume_region(
            layer_range, volume_region_idx
        )
        if not volume_region:
            continue
        target_region_idx = int(
            obj.api.host.slicing_volume_region_get_layer_region_idx(volume_region)
        )
        if target_region_idx < 0 or target_region_idx >= len(region_slices):
            continue

        mask = _auxiliary_slice_volume_mask(
            storage_address, print_view, obj, volume_region, subject, slice_z
        )
        if mask.empty():
            mask.free_from_storage()
            continue

        parent_idx = int(obj.api.host.slicing_volume_region_get_parent(volume_region))
        if 0 <= parent_idx < len(effective_masks):
            with clipper(mask) as mask_operand, clipper(effective_masks[parent_idx]) as parent_operand:
                with clipper.intersection(mask_operand, parent_operand) as clipped:
                    parent_clipped = clipped.to_expolygon_collection()
            mask.free_from_storage()
            mask = parent_clipped
            if mask.empty():
                mask.free_from_storage()
                continue

        effective_masks[volume_region_idx].copy_from(mask)
        for region_slice in region_slices:
            with clipper(region_slice) as region_operand, clipper(mask) as mask_operand:
                with clipper.diff(region_operand, mask_operand) as difference:
                    remaining = difference.to_expolygon_collection()
            region_slice.move_from(remaining)

        region_slices[target_region_idx].append_move_from(mask)
        with clipper(region_slices[target_region_idx]) as target_operand:
            with clipper.union(target_operand) as united:
                merged = united.to_expolygon_collection()
        region_slices[target_region_idx].move_from(merged)

    for region_idx, region_slice in enumerate(region_slices):
        region = obj.api.host.layer_get_region_mutable(layer.mutable_c_handle(), region_idx)
        destination = obj.api.host.layer_region_borrow_mutable_slices(region)
        if destination:
            obj.api.host.expolygons_move(destination, region_slice.mutable_c_handle())

    obj.api.host.layer_recompute_slices_and_islands_from_layer_regions(layer.mutable_c_handle())
    obj.api.host.layer_add_regions_to_islands(layer.mutable_c_handle())


@abi_requires_headers(
    "slic3r_clipper.h",
    "slic3r_config.h",
    "slic3r_config_option.h",
    "slic3r_data_tree.h",
    "slic3r_geometry.h",
    "slic3r_volume.h",
)
def build_auxiliary_layer_regions_from_subject(
    storage_address: int,
    print_view: Print,
    obj: Object,
    subject: ExPolygonCollection,
    height: int,
    print_z: int,
    slice_z: int,
) -> AuxiliaryLayerBuildResult:
    """
    Create and region-classify an auxiliary layer from an existing 2D subject.

    ``height``, ``print_z`` and ``slice_z`` are scaled coordinates. ``subject``
    is borrowed and read-only: the helper copies or clips it but never mutates
    or frees it. ``print_view`` and ``obj`` must belong to the same live print,
    and the call must run in a sequential phase where adding a layer is allowed.

    Region zero initially owns the complete subject. Model-part and parameter-
    modifier volumes active at ``slice_z`` are sliced to choose alternate
    PrintRegion settings; negative and special-purpose volumes are ignored.
    The helper then rebuilds layer slices, islands and region-island membership.
    It does not generate perimeters, infill, support, or extrusion paths.

    Intermediate polygons are allocated in ``storage_address`` and released by
    their ``Stored*`` wrappers before return. The successful ``Layer`` result is
    a borrowed view owned by ``obj`` and must never be freed by the plugin. A
    false result means no layer was created. If an exception occurs after host
    creation, there is no rollback: the object may retain a partially populated
    auxiliary layer, so callers should validate inputs before invoking it.
    """
    storage = _require_handle(storage_address, "storage")
    if obj.print_region_count() == 0 or int(height) <= 0 or int(print_z) <= 0:
        return AuxiliaryLayerBuildResult(None, False)

    layer_handle = obj.api.host.object_add_auxiliary_layer(
        obj.c_handle(), int(height), int(print_z), int(slice_z)
    )
    if not layer_handle:
        return AuxiliaryLayerBuildResult(None, False)

    layer = MutableLayer(obj.api, layer_handle)
    _auxiliary_assign_regions_from_masks(
        storage, print_view, obj, layer, subject, int(slice_z)
    )
    return AuxiliaryLayerBuildResult(layer, True)


__all__ = [
    "AuxiliaryLayerBuildResult",
    "ConstPluginProperties",
    "Config",
    "ConfigOption",
    "ConfigPoint",
    "DataTreeView",
    "GraphData",
    "Layer",
    "LayerAdhesionProperty",
    "LayerInstanceProperty",
    "LayerBrimProperty",
    "LayerIsland",
    "LayerRegion",
    "LayerRegionIsland",
    "LayerSupportProperty",
    "MutableConfig",
    "MutableConfigOption",
    "MutableLayer",
    "MutableLayerIsland",
    "MutableLayerRegion",
    "MutableLayerRegionIsland",
    "MutableObject",
    "MutablePrint",
    "MutableSurface",
    "Object",
    "Print",
    "PrintRecords",
    "PrintRegion",
    "project_painting_to_polygons",
    "StoredConfig",
    "StoredSurface",
    "StoredSurfaceCollection",
    "Surface",
    "SurfaceCollection",
    "SurfaceTypeBuilder",
    "TriangleMesh",
    "Volume",
    "build_auxiliary_layer_regions_from_subject",
    "layer_adhesion_property",
    "layer_has_adhesion_kind",
    "layer_is_brim",
    "layer_is_normal_skirt",
    "layer_is_skirt_first_layer_only",
    "srf_type",
    "unscaled",
]
