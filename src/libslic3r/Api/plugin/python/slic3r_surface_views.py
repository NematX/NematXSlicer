#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python views for Surface objects and collections.

``Surface`` and ``SurfaceCollection`` borrow host-owned data. Their handles
must never be freed by a plugin and element views may be invalidated by a
structural change to their collection. ``StoredSurfaceCollection`` owns a
temporary collection allocated in plugin storage; use ``free_from_storage()``
or a ``with`` block before clearing that storage. Stored wrappers have unique
ownership: assignment shares a wrapper, while shallow and deep copies are
forbidden.

Copy operations preserve their source. Move operations consume the geometry
inside a ``StoredExPolygon`` container but leave that container valid and
empty. ``append_like*()`` first snapshots its Surface template, so the template
may safely come from the same collection that receives the new elements.
"""

from __future__ import annotations

import ctypes
from typing import Iterator, TYPE_CHECKING

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    CSurface,
    RAW_SURFACE_TYPE_DENS_SOLID,
    RAW_SURFACE_TYPE_DENS_SPARSE,
    RAW_SURFACE_TYPE_DENS_VOID,
    RAW_SURFACE_TYPE_MOD_BRIDGE,
    RAW_SURFACE_TYPE_MOD_OVERBRIDGE,
    RAW_SURFACE_TYPE_MOD_SUPPORT,
    RAW_SURFACE_TYPE_NONE,
    RAW_SURFACE_TYPE_POS_BOTTOM,
    RAW_SURFACE_TYPE_POS_INTERNAL,
    RAW_SURFACE_TYPE_POS_PERIMETER,
    RAW_SURFACE_TYPE_POS_TOP,
)
from slic3r_geometry_views import (
    ExPolygon,
    ExPolygonCollection,
    HandleView,
    MutableExPolygon,
    StoredExPolygon,
    StoredExPolygonCollection,
    _UniqueOwnerMixin,
)

if TYPE_CHECKING:
    from slic3r_datatree_views import PluginProperties


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


# Small builder for raw_surface_type. It mirrors the intended order:
# position -> density -> optional bridge, overbridge and support modifiers.
class SurfaceTypeBuilder:
    def __init__(self, value: int = RAW_SURFACE_TYPE_NONE) -> None:
        self.value = int(value)

    def top(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_TOP)

    def bottom(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_BOTTOM)

    def internal(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_INTERNAL)

    def perimeter(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_PERIMETER)

    def solid(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_SOLID)

    def sparse(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_SPARSE)

    def empty(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_VOID)

    def bridge(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_MOD_BRIDGE)

    def overbridge(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_MOD_OVERBRIDGE)

    def support(self) -> "SurfaceTypeBuilder":
        """Add support auxiliary-layer membership, preserving position and density."""
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_MOD_SUPPORT)

    def raw(self) -> int:
        return self.value

    def __int__(self) -> int:
        return self.value


def srf_type() -> SurfaceTypeBuilder:
    return SurfaceTypeBuilder()


def surface_type_is_support(surface_type: int) -> bool:
    """Test whether a type marks a surface in support auxiliary geometry."""
    return (int(surface_type) & RAW_SURFACE_TYPE_MOD_SUPPORT) != 0


# Borrowed Surface view over either a native handle or a c_surface snapshot.
# Geometry and properties belong to the source and are never freed here.
@abi_requires_headers("slic3r_data_tree.h")
class Surface:
    def __init__(self, api, handle=None, surface: CSurface | None = None) -> None:
        if handle is None and surface is None:
            raise ValueError("surface handle or c_surface is required")
        self.api = api
        self.address = _address(handle)
        self._surface = surface

    @classmethod
    def from_c_surface(cls, api, surface: CSurface) -> "Surface":
        return cls(api, surface=surface)

    def has_handle(self) -> bool:
        return self.address != 0

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def c_view(self) -> CSurface:
        if self.has_handle():
            return self.api.host.surface_c_view(self.c_handle())
        return self._surface

    def expolygon(self) -> ExPolygon:
        handle = (
            self.api.host.surface_get_expolygon(self.c_handle())
            if self.has_handle()
            else self._surface.expolygon
        )
        return ExPolygon(self.api, handle)

    def type(self) -> int:
        if self.has_handle():
            return int(self.api.host.surface_get_type(self.c_handle()))
        return int(self._surface.type)

    def id(self) -> int:
        if self.has_handle():
            return int(self.api.host.surface_get_id(self.c_handle()))
        return int(self._surface.id)

    def properties(self) -> "ConstPluginProperties":
        # Delayed import keeps Surface views independent from the much larger
        # data-tree module while preserving the existing public property type.
        from slic3r_datatree_views import ConstPluginProperties

        if not self.has_handle():
            return ConstPluginProperties(self.api, None)
        return ConstPluginProperties(
            self.api, self.api.host.surface_get_properties(self.c_handle())
        )

    def has_flag(self, flag: int) -> bool:
        if self.has_handle():
            return bool(self.api.host.surface_get_flag(self.c_handle(), int(flag)))
        return (self.type() & int(flag)) != 0

    def has_any_flag(self, flags: int) -> bool:
        """Return whether at least one bit from ``flags`` is set."""
        return (self.type() & int(flags)) != 0

    def has_all_flags(self, flags: int) -> bool:
        """Return whether every bit from ``flags`` is set."""
        flags = int(flags)
        return (self.type() & flags) == flags


# Borrowed mutable view over a Surface owned by a mutable collection. It edits
# plugin metadata only. Structural changes to the parent invalidate this view.
@abi_requires_headers("slic3r_data_tree.h")
class MutableSurface(Surface):
    def __init__(self, api, handle, properties) -> None:
        super().__init__(api, handle=handle)
        self._properties = properties

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def readonly(self) -> Surface:
        """Return another borrowed read-only view with the same lifetime."""
        return Surface(self.api, self.c_handle())

    def mutable_properties(self) -> "PluginProperties":
        """Borrow the writable metadata container of this Surface."""
        from slic3r_datatree_views import PluginProperties

        return PluginProperties(
            self.api,
            self._properties,
        )

    def copy_properties_from(self, other: Surface) -> None:
        """
        Replace this Surface's metadata with independent copies from ``other``.

        Geometry, type and runtime id are unchanged. An ABI snapshot has no
        property container and therefore leaves this Surface unchanged.
        """
        self.mutable_properties().copy_from(other.properties())


# Borrowed read-only collection. Element views are invalidated by structural
# changes to the owner and must never be released by the plugin.
@abi_requires_headers("slic3r_data_tree.h")
class SurfaceCollection(HandleView):
    def size(self) -> int:
        return int(self.api.host.surface_collection_size(self.c_handle()))

    def empty(self) -> bool:
        return self.size() == 0

    def at(self, idx: int) -> Surface:
        return Surface(
            self.api,
            self.api.host.surface_collection_at(self.c_handle(), int(idx)),
        )

    def front(self) -> Surface:
        if self.empty():
            raise IndexError("empty surface collection")
        return self.at(0)

    def back(self) -> Surface:
        if self.empty():
            raise IndexError("empty surface collection")
        return self.at(self.size() - 1)

    def __len__(self) -> int:
        return self.size()

    def __iter__(self) -> Iterator[Surface]:
        for idx in range(self.size()):
            yield self.at(idx)

    def __getitem__(self, idx: int) -> Surface:
        if idx < 0:
            idx += self.size()
        if idx < 0 or idx >= self.size():
            raise IndexError(idx)
        return self.at(idx)


# Storage-owned collection used to assemble Surfaces before a host callback
# moves them into the data tree. Existing element views die after any append.
@abi_requires_headers("slic3r_data_tree.h", "slic3r_geometry.h")
class StoredSurfaceCollection(_UniqueOwnerMixin, SurfaceCollection):
    def __init__(self, api, storage) -> None:
        self._storage = _require_handle(storage, "storage")
        handle = api.host.storage_new_surface_collection(_void_p(self._storage))
        super().__init__(api, handle)
        self._owns_handle = True

    @property
    def storage_address(self) -> int:
        return self._storage

    def storage(self) -> int:
        return self._storage

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def readonly(self) -> SurfaceCollection:
        """Borrow a read-only view valid until mutation or final release."""
        return SurfaceCollection(self.api, self.c_handle())

    def clear(self) -> None:
        """Remove every Surface and invalidate all element views."""
        self.api.host.surface_collection_clear(self.mutable_c_handle())

    def mutable_at(self, idx: int) -> MutableSurface:
        """Borrow one mutable metadata view until the next structural change."""
        if idx < 0:
            idx += self.size()
        if idx < 0 or idx >= self.size():
            raise IndexError(idx)
        handle = self.api.host.surface_collection_at_mutable(
            self.mutable_c_handle(), int(idx)
        )
        return MutableSurface(self.api, handle,
                              self.api.host.surface_collection_get_properties_mutable(self.mutable_c_handle(), int(idx)))

    def back_mutable(self) -> MutableSurface:
        if self.empty():
            raise IndexError("empty surface collection")
        return self.mutable_at(self.size() - 1)

    def append(self, areas, surface_type: int) -> None:
        """
        Append copied geometry while preserving ownership of ``areas``.

        ``areas`` may be an ExPolygon or ExPolygonCollection. Existing Surface
        views may be invalidated by native vector reallocation.
        """
        if isinstance(areas, ExPolygonCollection):
            if not areas.empty():
                self.api.host.surface_collection_append_expolygons_copy(
                    self.mutable_c_handle(), areas.c_handle(), int(surface_type)
                )
            return
        if isinstance(areas, ExPolygon):
            self.api.host.surface_collection_append_expolygon_copy(
                self.mutable_c_handle(), areas.c_handle(), int(surface_type)
            )
            return
        raise TypeError("areas must be an ExPolygon or ExPolygonCollection")

    def append_move(self, areas, surface_type: int) -> None:
        """
        Move owned geometry into new Surfaces without copying it.

        Stored geometry wrappers remain valid but empty after the move and may
        be reused or released. They must belong to the same host API.
        """
        if isinstance(areas, StoredExPolygonCollection):
            if not areas.empty():
                self.api.host.surface_collection_append_expolygons_move(
                    self.mutable_c_handle(), areas.mutable_c_handle(), int(surface_type)
                )
            return
        if isinstance(areas, StoredExPolygon):
            self.api.host.surface_collection_append_expolygon_move(
                self.mutable_c_handle(), areas.mutable_c_handle(), int(surface_type)
            )
            return
        raise TypeError(
            "areas must be a StoredExPolygon or StoredExPolygonCollection"
        )

    def append_like(self, areas, surface_template: Surface) -> None:
        """Copy geometry plus the template's type and plugin properties."""
        with self._snapshot_template(surface_template) as template_snapshot:
            first_new_idx = self.size()
            self.append(areas, template_snapshot.front().type())
            self._copy_properties_to_range(first_new_idx, template_snapshot.front())

    def append_like_move(self, areas, surface_template: Surface) -> None:
        """Move geometry and copy the template's type and plugin properties."""
        with self._snapshot_template(surface_template) as template_snapshot:
            first_new_idx = self.size()
            self.append_move(areas, template_snapshot.front().type())
            self._copy_properties_to_range(first_new_idx, template_snapshot.front())

    def _snapshot_template(
        self, surface_template: Surface
    ) -> "StoredSurfaceCollection":
        """
        Copy a template before destination mutation can invalidate its handle.

        A SurfaceCollection is used because the C ABI stores plugin properties
        on Surface objects and has no standalone owning property container.
        """
        snapshot = StoredSurfaceCollection(self.api, self._storage)
        try:
            snapshot.append(surface_template.expolygon(), surface_template.type())
            snapshot.back_mutable().copy_properties_from(surface_template)
            return snapshot
        except Exception:
            snapshot.free_from_storage()
            raise

    def _copy_properties_to_range(
        self, first_new_idx: int, surface_template: Surface
    ) -> None:
        for idx in range(first_new_idx, self.size()):
            self.mutable_at(idx).copy_properties_from(surface_template)

    def free_from_storage(self) -> bool:
        """Release this collection and invalidate every view borrowed from it."""
        if not self._owns_handle or not self.handle():
            return False
        freed = bool(
            self.api.host.storage_free(
                _void_p(self._storage), self.mutable_c_handle()
            )
        )
        if freed:
            self._handle = 0
            self._owns_handle = False
        return freed

    def __enter__(self) -> "StoredSurfaceCollection":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.free_from_storage()

    def __del__(self) -> None:
        try:
            self.free_from_storage()
        except Exception:
            pass


# Owned pairing of storage-backed geometry and a Surface type. Construction
# transfers ownership from the input StoredExPolygon, which becomes invalid.
@abi_requires_headers("slic3r_data_tree.h", "slic3r_geometry.h")
class StoredSurface(_UniqueOwnerMixin):
    def __init__(self, expolygon: StoredExPolygon, surface_type: int) -> None:
        if not isinstance(expolygon, StoredExPolygon):
            raise TypeError("expolygon must be a StoredExPolygon")
        if not expolygon:
            raise ValueError("expolygon must own a valid handle")
        self.api = expolygon.api
        self._expolygon = StoredExPolygon.adopt_owned(
            expolygon.api,
            expolygon.storage_address,
            expolygon.release(),
        )
        self._type = int(surface_type)

    def readonly(self) -> Surface:
        """Borrow a Surface snapshot valid while this object owns its geometry."""
        return Surface.from_c_surface(self.api, self.c_view())

    def expolygon(self) -> ExPolygon:
        """Borrow read-only geometry; do not free the returned view."""
        return ExPolygon(self.api, self._expolygon.c_handle())

    def expolygon_mutable(self) -> MutableExPolygon:
        """Borrow mutable geometry without exposing ownership operations."""
        return MutableExPolygon(self.api, self._expolygon.mutable_c_handle())

    def type(self) -> int:
        return self._type

    def c_view(self) -> CSurface:
        """Create a non-owning ABI snapshot whose runtime Surface id is zero."""
        return CSurface(self._expolygon.c_handle(), self._type, 0)

    def free_from_storage(self) -> bool:
        """Release the geometry; every view borrowed from it becomes invalid."""
        return self._expolygon.free_from_storage()

    def __enter__(self) -> "StoredSurface":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.free_from_storage()


__all__ = [
    "MutableSurface",
    "StoredSurface",
    "StoredSurfaceCollection",
    "Surface",
    "SurfaceCollection",
    "SurfaceTypeBuilder",
    "srf_type",
    "surface_type_is_support",
]
