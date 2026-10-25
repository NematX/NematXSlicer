#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for STEP_SLICING plugins.

Slicing plugins create or edit raw LayerRegion slices for one object. The
payload exposes the object's layer ranges and their volume-region assignments,
so a plugin can decide which volumes contribute geometry to which LayerRegion.

Context contents
----------------

SlicingContext exposes:

* read-only slicing_print() and slicing_object() views for the object being
  sliced;
* layer_ranges(), each with scaled Z bounds, effective config, and
  volume_regions();
* each volume-region entry exposes the model Volume, parent relation, target
  LayerRegion index, and bounding box;
* slicing_borrow_layer_region_slices(), the mutable destination polygons for one
  LayerRegion;
* slice_volume_to_expolygons(), a convenience wrapper around the host mesh
  slicer for plugins that do not need a custom triangle slicer;
* plugin_storage(), cancellation, progress, warning, and error helpers.

Typical use
-----------

    ctx = api.slicing_context(run_ctx_address)
    if ctx is None:
        return

    for layer_range in ctx.layer_ranges():
        for volume_region in layer_range.volume_regions():
            if volume_region.layer_region_idx() < 0:
                continue
            volume = volume_region.volume()
            # Slice volume.mesh(), then move/intersect the result into the
            # matching layer-region slices.

The helper intentionally keeps slicing policy explicit. It exposes the raw
range/volume-region information and a small mesh-slicing convenience wrapper,
but it does not try to reproduce native PrintObjectSlice.cpp by itself.
"""

from __future__ import annotations

import ctypes
from typing import Iterable, Iterator

from slic3r_api_generated import (
    CMeshSlicingParams,
    PluginRunContext,
    RunCtxSlicing,
    STEP_SLICING,
)
from slic3r_datatree_views import Config, MutableLayerRegion, Object, Print, Volume
from slic3r_geometry_views import MutableExPolygonCollection, StoredExPolygonCollection
from slic3r_abi import abi_requires_headers
from slic3r_run_context import PluginRunContextView


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


class SlicingLayerRange:
    """
    Borrowed view over one object layer-config range.

    A range owns a list of volume-region entries. Those entries tell the plugin
    which model volumes are relevant inside this Z interval and which
    LayerRegion index should receive their sliced polygons.
    """

    def __init__(self, ctx: "SlicingContext", handle) -> None:
        self.ctx = ctx
        self.api = ctx.api
        self.address = _address(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def z_min(self) -> int:
        return int(self.ctx.payload.layer_range_z_min(self.c_handle()))

    def z_max(self) -> int:
        return int(self.ctx.payload.layer_range_z_max(self.c_handle()))

    def config(self) -> Config:
        return Config(self.api, self.ctx.payload.layer_range_config(self.c_handle()))

    def volume_region_count(self) -> int:
        return int(self.ctx.payload.layer_range_volume_region_count(self.c_handle()))

    def volume_region(self, idx: int) -> "SlicingVolumeRegion":
        if idx < 0 or idx >= self.volume_region_count():
            raise IndexError(idx)
        handle = self.ctx.payload.layer_range_volume_region_at(self.c_handle(), int(idx))
        return SlicingVolumeRegion(self.ctx, handle)

    def volume_regions(self) -> Iterator["SlicingVolumeRegion"]:
        for idx in range(self.volume_region_count()):
            yield self.volume_region(idx)


class SlicingVolumeRegion:
    """
    Borrowed view over one volume-region assignment.

    layer_region_idx() returns -1 for helper/negative entries that do not write
    printable material directly.
    """

    def __init__(self, ctx: "SlicingContext", handle) -> None:
        self.ctx = ctx
        self.api = ctx.api
        self.address = _address(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def volume(self) -> Volume:
        return Volume(self.api, self.ctx.payload.volume_region_volume(self.c_handle()))

    def parent(self) -> int:
        return int(self.ctx.payload.volume_region_parent(self.c_handle()))

    def layer_region_idx(self) -> int:
        return int(self.ctx.payload.volume_region_layer_region_idx(self.c_handle()))

    def bounding_box(self):
        return self.ctx.payload.volume_region_bbox(self.c_handle())


@abi_requires_headers("steps/slic3r_step_slicing.h")
class SlicingContext(PluginRunContextView):
    """
    High-level wrapper around the STEP_SLICING payload.

    The context is valid only for the current PluginBase.run() call. Mutable
    slices returned by borrow_layer_region_slices() are borrowed host storage;
    do not free them through plugin storage.
    """

    def __init__(self, api, common: PluginRunContext, payload: RunCtxSlicing) -> None:
        super().__init__(api, common)
        self.payload = payload

    @classmethod
    def from_slicing_run_context(cls, api, run_ctx_address: int) -> "SlicingContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != STEP_SLICING or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxSlicing)).contents
        if not payload.print or not payload.object or not payload.layer_region_borrow_mutable_slices:
            return None
        return cls(api, common, payload)

    def slicing_print(self) -> Print:
        return Print(self.api, self.payload.print)

    def slicing_object(self) -> Object:
        return Object(self.api, self.payload.object)

    def slicing_borrow_layer_region_slices(self, layer_region: MutableLayerRegion) -> MutableExPolygonCollection:
        handle = self.payload.layer_region_borrow_mutable_slices(layer_region.c_handle())
        return MutableExPolygonCollection(self.api, handle)

    def layer_range_count(self) -> int:
        return int(self.payload.layer_range_count(self.payload.object))

    def layer_range(self, idx: int) -> SlicingLayerRange:
        if idx < 0 or idx >= self.layer_range_count():
            raise IndexError(idx)
        return SlicingLayerRange(self, self.payload.layer_range_at(self.payload.object, int(idx)))

    def layer_ranges(self) -> Iterator[SlicingLayerRange]:
        for idx in range(self.layer_range_count()):
            yield self.layer_range(idx)

    @abi_requires_headers("slic3r_volume.h", "slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
    def slice_volume_to_expolygons(
        self,
        volume: Volume,
        z_mm_by_layer: Iterable[float],
        params: CMeshSlicingParams | None = None,
    ) -> list[StoredExPolygonCollection]:
        """
        Slice one volume mesh to storage-owned ExPolygonCollections.

        z_mm_by_layer uses unscaled object-local millimeters. If params is not
        provided, the volume matrix is used with a regular slicing and no cleaning
        distance. Passing CMeshSlicingParams lets a plugin reproduce the native object
        slicing: the closing radius, extra offset, model resolution, resolution and
        slicing modes are applied through the API (see TriangleMesh.slice_to_expolygons()).

        The heights are validated: a value that is not finite, or smaller than the one
        before it, raises ValueError before anything is allocated.
        """
        return volume.mesh().slice_to_expolygons(
            self.plugin_storage(), volume.matrix(), z_mm_by_layer, params
        )

__all__ = [
    "SlicingContext",
    "SlicingLayerRange",
    "SlicingVolumeRegion",
]
