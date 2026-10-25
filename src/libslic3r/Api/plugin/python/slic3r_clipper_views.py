#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python Clipper helpers over the SuperSlicer plugin C ABI.

Clipper operations work on a temporary ``ClipperOperand``. An operand is a
storage-owned adapter over a polygon, polyline, polygon collection, ExPolygon,
or ExPolygon collection. Creating an operand from existing geometry is cheap:
the source geometry is referenced, not copied, so keep that source alive while
the operand is used.

Typical use
-----------

Create a small clipping context for the current storage:

    clip = api.clipper(storage_handle)
    unsupported = clip.diff(clip(island.slice()), clip(lower_slices))
    unsupported_polygons = unsupported.to_expolygon_collection()

Build an accumulator, then union it when geometric merging is needed:

    acc = clip.empty()
    acc += clip(first_area)
    acc += clip(second_area)
    merged = clip.union(acc)

Write a result into an already-owned destination collection:

    result = clip.offset(clip(input_polygons), scale_i(0.2))
    result.write_expolygons_to(output_expolygons)

Lifetime model
--------------
- ClipperOperand objects returned by this module own their handle unless they
  were created with ``ClipperOperand.null()``.
- Owned handles are released with ``storage_free()`` when the object is closed,
  exits a ``with`` block, or is garbage-collected.
- Assignment shares one operand wrapper. Shallow and deep copies are forbidden
  because an owned operand has unique native ownership.
- Collections returned by ``to_polygon_collection()`` and
  ``to_expolygon_collection()`` are also storage-owned wrappers and follow the
  geometry helper lifetime rules.
"""

from __future__ import annotations

import ctypes
from typing import Callable

from slic3r_api_generated import (
    CBoundingBox,
    CLIPPER_END_CLOSED_LINE,
    CLIPPER_END_CLOSED_POLYGON,
    CLIPPER_END_OPEN_BUTT,
    CLIPPER_END_OPEN_ROUND,
    CLIPPER_END_OPEN_SQUARE,
    CLIPPER_JOIN_MITER,
    CLIPPER_JOIN_ROUND,
    CLIPPER_JOIN_SQUARE,
    CLIPPER_OPERATION_DIFFERENCE,
    CLIPPER_OPERATION_INTERSECTION,
    CLIPPER_OPERATION_UNION,
    CLIPPER_OPERATION_XOR,
)

from slic3r_geometry_views import (
    ExPolygon,
    ExPolygonCollection,
    Polygon,
    PolygonCollection,
    Polyline,
    PolylineCollection,
    StoredExPolygonCollection,
    StoredPolygonCollection,
    StoredPolylineCollection,
    _UniqueOwnerMixin,
)
from slic3r_abi import abi_requires_headers


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


def _same_storage(lhs: "ClipperOperand", rhs: "ClipperOperand") -> bool:
    return lhs.storage() == rhs.storage()


def _check_same_storage(lhs: "ClipperOperand", rhs: "ClipperOperand") -> None:
    if not _same_storage(lhs, rhs):
        raise ValueError("clipper operands must use the same storage")


def _binary_args(storage_or_subject, subject_or_clip, clip=None):
    if clip is None:
        subject = storage_or_subject
        other = subject_or_clip
        _check_same_storage(subject, other)
        return subject.api, subject.storage(), subject, other
    subject = subject_or_clip
    other = clip
    storage = _require_handle(storage_or_subject, "storage")
    if subject.storage() != storage or other.storage() != storage:
        raise ValueError("explicit storage must match both clipper operands")
    return subject.api, storage, subject, other


def _unary_args(storage_or_subject, subject=None):
    if subject is None:
        operand = storage_or_subject
        return operand.api, operand.storage(), operand
    storage = _require_handle(storage_or_subject, "storage")
    if subject.storage() != storage:
        raise ValueError("explicit storage must match the clipper operand")
    return subject.api, storage, subject


@abi_requires_headers("slic3r_clipper.h")
class ClipperOperand(_UniqueOwnerMixin):
    """
    Storage-owned Clipper input or result.

    Use ``api.clipper(storage)`` to build operands from geometry and chain
    operations. A borrowed geometry view may be wrapped as an operand, but the
    operand does not copy it. If the source geometry is modified or freed while
    the operand exists, the operand becomes invalid too.
    """

    def __init__(self, api, storage, source=None) -> None:
        self.api = api
        self._storage = _require_handle(storage, "storage")
        self._handle = 0
        self._owns_handle = False

        if source is None:
            self._handle = _address(self.api.host.clipper_shapes_create_empty(self.c_storage()))
            self._owns_handle = True
        else:
            self._handle = self._create_from_source(source)
            self._owns_handle = True

    @classmethod
    def null(cls, api, storage) -> "ClipperOperand":
        """Create an empty view with no storage handle allocated."""
        out = cls.__new__(cls)
        out.api = api
        out._storage = _require_handle(storage, "storage")
        out._handle = 0
        out._owns_handle = False
        return out

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "ClipperOperand":
        """Take ownership of a Clipper handle allocated in storage."""
        out = cls.__new__(cls)
        out.api = api
        out._storage = _require_handle(storage, "storage")
        out._handle = _require_handle(handle)
        out._owns_handle = True
        return out

    @classmethod
    def create_empty(cls, api, storage) -> "ClipperOperand":
        """Create a storage-owned operand that contains no paths."""
        return cls(api, storage)

    def _create_from_source(self, source) -> int:
        if isinstance(source, Polygon):
            return _address(self.api.host.clipper_shapes_from_polygon(self.c_storage(), source.c_handle()))
        if isinstance(source, Polyline):
            return _address(self.api.host.clipper_shapes_from_polyline(self.c_storage(), source.c_handle()))
        if isinstance(source, PolygonCollection):
            return _address(self.api.host.clipper_shapes_from_polygons(self.c_storage(), source.c_handle()))
        if isinstance(source, ExPolygon):
            return _address(self.api.host.clipper_shapes_from_expolygon(self.c_storage(), source.c_handle()))
        if isinstance(source, ExPolygonCollection):
            return _address(self.api.host.clipper_shapes_from_expolygons(self.c_storage(), source.c_handle()))
        raise TypeError(f"unsupported Clipper source type: {type(source)!r}")

    @property
    def address(self) -> int:
        return self._handle

    @property
    def storage_address(self) -> int:
        return self._storage

    def storage(self) -> int:
        return self._storage

    def c_storage(self) -> ctypes.c_void_p:
        return _void_p(self._storage)

    def handle(self) -> int:
        return self._handle

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self._handle)

    def is_valid_handle(self) -> bool:
        return self._handle != 0

    def empty(self) -> bool:
        """Return whether this operand contains no non-empty Clipper path."""
        return bool(self.api.host.clipper_shapes_empty(self.c_handle()))

    def path_count(self) -> int:
        """
        Count the non-empty raw paths held by this operand.

        This is not an ExPolygon count: an outer contour and each non-empty
        hole count as separate paths. The method only inspects the operand and
        does not materialize geometry or transfer ownership. A null operand,
        such as ``ClipperOperand.null()``, contains zero paths.
        """
        return int(self.api.host.clipper_shapes_path_count(self.c_handle()))

    def bounding_box(self) -> CBoundingBox:
        """
        Return the bounds of all raw Clipper points in scaled XY coordinates.

        The returned ``CBoundingBox`` is a value copy and remains valid after
        the operand is freed. No polygon collection is allocated. An empty or
        null operand returns the host's empty, zero-initialized bounding box;
        callers should use ``empty()`` when they need to distinguish that case
        from geometry whose coordinates happen to include the origin.
        """
        return self.api.host.clipper_shapes_bounding_box(self.c_handle())

    def same_handle(self, other: "ClipperOperand") -> bool:
        return self._handle == other.handle()

    def release(self) -> int:
        handle = self._handle
        self._handle = 0
        self._owns_handle = False
        return handle

    def free_from_storage(self) -> bool:
        if not self._owns_handle or not self._handle:
            return False
        freed = bool(self.api.host.storage_free(self.c_storage(), self.c_handle()))
        if freed:
            self._handle = 0
            self._owns_handle = False
        return freed

    def replace(self, other: "ClipperOperand") -> "ClipperOperand":
        """Release this operand and take ownership of ``other``'s handle."""
        _check_same_storage(self, other)
        if self is other:
            return self
        self.free_from_storage()
        self._handle = other.release()
        self._owns_handle = self._handle != 0
        return self

    def concat(self, other: "ClipperOperand") -> "ClipperOperand":
        """Return a new flat path accumulator without running a geometric union."""
        _check_same_storage(self, other)
        handle = self.api.host.clipper_concat(self.c_storage(), self.c_handle(), other.c_handle())
        return ClipperOperand.adopt_owned(self.api, self.storage(), handle)

    def concat_replace(self, other: "ClipperOperand") -> "ClipperOperand":
        """
        Append raw paths into this operand when the host can do so efficiently.

        This is not a geometric union. Use ``clipper_union()`` or
        ``ClipperContext.union()`` afterwards if overlapping paths must be
        merged.
        """
        _check_same_storage(self, other)
        handle = self.api.host.clipper_concat_replace(self.c_storage(), self.c_handle(), other.c_handle())
        self._handle = _address(handle)
        self._owns_handle = self._handle != 0
        return self

    def to_polygon_collection(self) -> StoredPolygonCollection:
        handle = self.api.host.clipper_shapes_to_polygons(self.c_storage(), self.c_handle())
        return StoredPolygonCollection.adopt_owned(self.api, self.storage(), handle)

    def to_expolygon_collection(self) -> StoredExPolygonCollection:
        handle = self.api.host.clipper_shapes_to_expolygons(self.c_storage(), self.c_handle())
        return StoredExPolygonCollection.adopt_owned(self.api, self.storage(), handle)

    def write_polygons_to(self, dst: StoredPolygonCollection) -> None:
        self.api.host.clipper_shapes_replace_polygons(dst.mutable_c_handle(), self.c_handle())

    def write_expolygons_to(self, dst: StoredExPolygonCollection) -> None:
        self.api.host.clipper_shapes_replace_expolygons(dst.mutable_c_handle(), self.c_handle())

    def for_each_expolygon(self, fn: Callable[[ExPolygon], object]) -> None:
        with self.to_expolygon_collection() as expolygons:
            for expolygon in expolygons:
                fn(expolygon)

    def __add__(self, other: "ClipperOperand") -> "ClipperOperand":
        return self.concat(other)

    def __iadd__(self, other: "ClipperOperand") -> "ClipperOperand":
        return self.concat_replace(other)

    def __bool__(self) -> bool:
        return self.is_valid_handle()

    def __enter__(self) -> "ClipperOperand":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.free_from_storage()

    def __del__(self) -> None:
        try:
            self.free_from_storage()
        except Exception:
            pass


def clipper_execute(storage_or_subject, operation: int, subject=None, clip=None) -> ClipperOperand:
    if isinstance(storage_or_subject, ClipperOperand):
        subject_operand = storage_or_subject
        storage = subject_operand.storage()
        api = subject_operand.api
        clip_operand = subject
    else:
        subject_operand = subject
        storage = _require_handle(storage_or_subject, "storage")
        api = subject_operand.api
        clip_operand = clip
        if subject_operand.storage() != storage:
            raise ValueError("explicit storage must match the clipper operand")
    if clip_operand is not None:
        _check_same_storage(subject_operand, clip_operand)
    clip_handle = ctypes.c_void_p(0) if clip_operand is None else clip_operand.c_handle()
    handle = api.host.clipper_execute(
        _void_p(storage), int(operation), subject_operand.c_handle(), clip_handle
    )
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_diff(storage_or_subject, subject_or_clip, clip=None) -> ClipperOperand:
    api, storage, subject, other = _binary_args(storage_or_subject, subject_or_clip, clip)
    handle = api.host.clipper_diff(_void_p(storage), subject.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_intersection(storage_or_subject, subject_or_clip, clip=None) -> ClipperOperand:
    api, storage, subject, other = _binary_args(storage_or_subject, subject_or_clip, clip)
    handle = api.host.clipper_intersection(_void_p(storage), subject.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_split(storage_or_subject, subject_or_clip, clip=None) -> tuple[ClipperOperand, ClipperOperand]:
    """
    Return ``(inside, outside)``: ``subject`` intersection ``clip`` and
    ``subject`` - ``clip``, computed from the same inputs so that they
    share their boundary along ``clip``. Both are new operands owned by the
    storage. Accepts ``(storage, subject, clip)`` or ``(subject, clip)``.
    """
    api, storage, subject, other = _binary_args(storage_or_subject, subject_or_clip, clip)
    inside = ctypes.c_void_p()
    outside = ctypes.c_void_p()
    if not api.host.clipper_split(
        _void_p(storage), subject.c_handle(), other.c_handle(), ctypes.byref(inside), ctypes.byref(outside)
    ):
        raise ValueError("clipper_split failed")
    return (
        ClipperOperand.adopt_owned(api, storage, _require_handle(inside.value, "inside clipper result")),
        ClipperOperand.adopt_owned(api, storage, _require_handle(outside.value, "outside clipper result")),
    )


def clipper_diff_with_safety_offset(storage_or_subject, subject_or_clip, clip=None) -> ClipperOperand:
    api, storage, subject, other = _binary_args(storage_or_subject, subject_or_clip, clip)
    handle = api.host.clipper_diff_with_safety_offset(_void_p(storage), subject.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_intersection_with_safety_offset(storage_or_subject, subject_or_clip, clip=None) -> ClipperOperand:
    api, storage, subject, other = _binary_args(storage_or_subject, subject_or_clip, clip)
    handle = api.host.clipper_intersection_with_safety_offset(_void_p(storage), subject.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_union(storage_or_subject, subject=None) -> ClipperOperand:
    api, storage, operand = _unary_args(storage_or_subject, subject)
    handle = api.host.clipper_union(_void_p(storage), operand.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_union_with_safety_offset(storage_or_subject, subject=None) -> ClipperOperand:
    api, storage, operand = _unary_args(storage_or_subject, subject)
    handle = api.host.clipper_union_with_safety_offset(_void_p(storage), operand.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_union2(storage_or_subject, subject_or_second, second=None) -> ClipperOperand:
    api, storage, first, other = _binary_args(storage_or_subject, subject_or_second, second)
    handle = api.host.clipper_union2(_void_p(storage), first.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_concat(storage_or_first, first_or_second, second=None) -> ClipperOperand:
    api, storage, first, other = _binary_args(storage_or_first, first_or_second, second)
    handle = api.host.clipper_concat(_void_p(storage), first.c_handle(), other.c_handle())
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_offset(storage_or_subject,
                   subject_or_delta,
                   delta=None,
                   join_type: int = CLIPPER_JOIN_MITER,
                   miter_limit: float = 3.0,
                   end_type: int = CLIPPER_END_CLOSED_POLYGON) -> ClipperOperand:
    if isinstance(storage_or_subject, ClipperOperand):
        subject = storage_or_subject
        storage = subject.storage()
        api = subject.api
        offset_delta = float(subject_or_delta)
        if delta is not None:
            join_type = int(delta)
    else:
        storage = _require_handle(storage_or_subject, "storage")
        subject = subject_or_delta
        api = subject.api
        if subject.storage() != storage:
            raise ValueError("explicit storage must match the clipper operand")
        offset_delta = float(delta)
    handle = api.host.clipper_offset(
        _void_p(storage), subject.c_handle(), offset_delta, int(join_type), float(miter_limit), int(end_type)
    )
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_offset2(storage_or_subject,
                    subject_or_delta1,
                    delta1_or_delta2,
                    delta2=None,
                    join_type: int = CLIPPER_JOIN_MITER,
                    miter_limit: float = 3.0,
                    end_type: int = CLIPPER_END_CLOSED_POLYGON) -> ClipperOperand:
    if isinstance(storage_or_subject, ClipperOperand):
        subject = storage_or_subject
        storage = subject.storage()
        api = subject.api
        offset_delta1 = float(subject_or_delta1)
        offset_delta2 = float(delta1_or_delta2)
        if delta2 is not None:
            join_type = int(delta2)
    else:
        storage = _require_handle(storage_or_subject, "storage")
        subject = subject_or_delta1
        api = subject.api
        if subject.storage() != storage:
            raise ValueError("explicit storage must match the clipper operand")
        offset_delta1 = float(delta1_or_delta2)
        offset_delta2 = float(delta2)
    handle = api.host.clipper_offset2(
        _void_p(storage),
        subject.c_handle(),
        offset_delta1,
        offset_delta2,
        int(join_type),
        float(miter_limit),
        int(end_type),
    )
    return ClipperOperand.adopt_owned(api, storage, handle)


def clipper_clip_shapes_with_subject_bbox(storage_or_subject, subject_or_bbox=None, bbox=None) -> ClipperOperand:
    """
    Clip a ClipperOperand to a local subject bounding box without materializing ExPolygons.

    This is a pre-filter for later boolean operations. The returned operand is a
    flat path-list shape, so use it to reduce the amount of geometry passed to
    ``clipper_intersection``/``clipper_diff`` and materialize only after the final
    operation that rebuilds contour/hole topology.
    """
    if isinstance(storage_or_subject, ClipperOperand):
        subject = storage_or_subject
        storage = subject.storage()
        api = subject.api
        subject_bbox = subject_or_bbox
    else:
        storage = _require_handle(storage_or_subject, "storage")
        subject = subject_or_bbox
        api = subject.api
        if subject.storage() != storage:
            raise ValueError("explicit storage must match the clipper operand")
        subject_bbox = bbox
    if subject.empty():
        return ClipperOperand.create_empty(api, storage)
    handle = api.host.clipper_clip_shapes_with_subject_bbox(
        _void_p(storage), subject.c_handle(), subject_bbox
    )
    return ClipperOperand.adopt_owned(api, storage, handle)


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_diff_polyline_expolygons(
    storage, subject: Polyline, clip: ExPolygonCollection
) -> StoredPolylineCollection:
    """
    Return the open pieces of ``subject`` that remain outside ``clip``.

    Both geometry arguments are borrowed, read-only views from the same API
    instance. The result is a new collection owned by ``storage``; release it
    with ``free_from_storage()`` or a ``with`` block. Its element views are
    borrowed from that collection. An empty subject returns an empty collection
    without invoking Clipper. Coordinates use scaled integer units.
    """
    storage_address = _require_handle(storage, "storage")
    if subject.api is not clip.api:
        raise ValueError("subject and clip must use the same API instance")
    if subject.empty():
        return StoredPolylineCollection(subject.api, storage_address)
    result = subject.api.host.clipper_diff_polyline_expolygons(
        _void_p(storage_address), subject.c_handle(), clip.c_handle()
    )
    return StoredPolylineCollection.adopt_owned(
        subject.api,
        storage_address,
        _require_handle(result, "clipped polyline collection result"),
    )


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_intersection_polyline_expolygons(
    storage, subject: Polyline, clip: ExPolygonCollection
) -> StoredPolylineCollection:
    """
    Return the open pieces of ``subject`` that lie inside ``clip``.

    The source views are borrowed and unchanged. A null or empty clip produces
    an empty storage-owned result. Release the returned collection with
    ``free_from_storage()`` or a ``with`` block before its storage is cleared;
    do not free element views obtained from the collection individually.
    """
    storage_address = _require_handle(storage, "storage")
    if subject.api is not clip.api:
        raise ValueError("subject and clip must use the same API instance")
    result = subject.api.host.clipper_intersection_polyline_expolygons(
        _void_p(storage_address), subject.c_handle(), clip.c_handle()
    )
    return StoredPolylineCollection.adopt_owned(
        subject.api,
        storage_address,
        _require_handle(result, "clipped polyline collection result"),
    )


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_offset_polylines(storage,
                             polylines: PolylineCollection,
                             delta: float,
                             join_type: int = CLIPPER_JOIN_SQUARE,
                             miter_limit: float = 0.0,
                             end_type: int = CLIPPER_END_OPEN_BUTT) -> ClipperOperand:
    """
    Offset open polylines into areas exactly like the native offset(Polylines).

    The defaults are the native line defaults (square joins, butt ends).
    ``delta`` is the scaled half width and must be positive. The polylines are
    read during the call only; the returned operand is owned by ``storage``.
    """
    storage_address = _require_handle(storage, "storage")
    if float(delta) <= 0.0:
        raise ValueError("polyline offset delta must be positive")
    handle = polylines.api.host.clipper_offset_polylines(
        _void_p(storage_address), polylines.c_handle(), float(delta), int(join_type), float(miter_limit), int(end_type)
    )
    return ClipperOperand.adopt_owned(polylines.api, storage_address, handle)


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_diff_polylines_expolygons(
    storage, subject: PolylineCollection, clip: ExPolygonCollection
) -> StoredPolylineCollection:
    """Return storage-owned batch fragments outside clip without modifying either input; result order is unspecified."""
    storage_address = _require_handle(storage, "storage")
    if subject.api is not clip.api:
        raise ValueError("subject and clip must use the same API instance")
    if subject.empty():
        return StoredPolylineCollection(subject.api, storage_address)
    result = subject.api.host.clipper_diff_polylines_expolygons(
        _void_p(storage_address), subject.c_handle(), clip.c_handle()
    )
    return StoredPolylineCollection.adopt_owned(
        subject.api, storage_address, _require_handle(result, "clipped polyline collection result")
    )


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_intersection_polylines_expolygons(
    storage, subject: PolylineCollection, clip: ExPolygonCollection
) -> StoredPolylineCollection:
    """Return storage-owned batch fragments inside clip without modifying either input; result order is unspecified."""
    storage_address = _require_handle(storage, "storage")
    if subject.api is not clip.api:
        raise ValueError("subject and clip must use the same API instance")
    if subject.empty():
        return StoredPolylineCollection(subject.api, storage_address)
    result = subject.api.host.clipper_intersection_polylines_expolygons(
        _void_p(storage_address), subject.c_handle(), clip.c_handle()
    )
    return StoredPolylineCollection.adopt_owned(
        subject.api, storage_address, _require_handle(result, "clipped polyline collection result")
    )


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_split_polyline_expolygons(
    storage, subject: Polyline, clip: ExPolygonCollection
) -> tuple[StoredPolylineCollection, StoredPolylineCollection]:
    """
    Return ``(inside, outside)``: the open pieces of ``subject`` inside and
    outside ``clip``, as the intersection and diff helpers would.

    Both sides come from the same prepared Clipper input, so they are cut at
    the same points: every end of a piece that is not an end of ``subject`` is
    also an end of a piece on the other side. When two cuts are less than
    ``SCALED_EPSILON`` apart, the tiny piece between them is dropped and the
    pieces around it are glued back. Piece order is unspecified. The
    sources are borrowed; both results are new collections owned by
    ``storage``. An empty subject returns two empty collections without
    invoking Clipper.
    """
    storage_address = _require_handle(storage, "storage")
    if subject.api is not clip.api:
        raise ValueError("subject and clip must use the same API instance")
    if subject.empty():
        return (
            StoredPolylineCollection(subject.api, storage_address),
            StoredPolylineCollection(subject.api, storage_address),
        )
    inside = ctypes.c_void_p()
    outside = ctypes.c_void_p()
    if not subject.api.host.clipper_split_polyline_expolygons(
        _void_p(storage_address),
        subject.c_handle(),
        clip.c_handle(),
        ctypes.byref(inside),
        ctypes.byref(outside),
    ):
        raise ValueError("clipper_split_polyline_expolygons failed")
    return (
        StoredPolylineCollection.adopt_owned(
            subject.api, storage_address, _require_handle(inside.value, "inside polyline collection result")
        ),
        StoredPolylineCollection.adopt_owned(
            subject.api, storage_address, _require_handle(outside.value, "outside polyline collection result")
        ),
    )


@abi_requires_headers("slic3r_clipper.h", "slic3r_geometry.h", "slic3r_utils.h")
def clipper_clip_expolygons_with_subject_bbox(
    storage, src: ExPolygonCollection, bbox: CBoundingBox
) -> StoredExPolygonCollection:
    """
    Copy and clip ExPolygons to a scaled subject bounding box.

    This is a spatial pre-filter for later boolean operations. ``src`` is a
    borrowed read-only collection and remains unchanged. The returned topology
    is an independent collection owned by ``storage`` and must be released with
    ``free_from_storage()`` or a ``with`` block. ``bbox`` is copied by value.
    """
    storage_address = _require_handle(storage, "storage")
    result = src.api.host.clipper_clip_expolygons_with_subject_bbox(
        _void_p(storage_address), src.c_handle(), bbox
    )
    return StoredExPolygonCollection.adopt_owned(
        src.api,
        storage_address,
        _require_handle(result, "clipped ExPolygon collection result"),
    )


@abi_requires_headers("slic3r_clipper.h")
class ClipperContext:
    """
    Convenience object bound to one storage_handle.

    It keeps call sites compact and prevents accidentally mixing operands from
    different storages:

        clip = api.clipper(storage)
        result = clip.diff(clip(subject), clip(mask))
    """

    def __init__(self, api, storage) -> None:
        self.api = api
        self._storage = _require_handle(storage, "storage")

    @property
    def storage_address(self) -> int:
        return self._storage

    def storage(self) -> int:
        return self._storage

    def __call__(self, source=None) -> ClipperOperand:
        if source is None:
            return ClipperOperand.null(self.api, self.storage())
        return ClipperOperand(self.api, self.storage(), source)

    def empty(self) -> ClipperOperand:
        return ClipperOperand.create_empty(self.api, self.storage())

    def diff(self, subject: ClipperOperand, clip: ClipperOperand) -> ClipperOperand:
        return clipper_diff(self.storage(), subject, clip)

    def intersection(self, subject: ClipperOperand, clip: ClipperOperand) -> ClipperOperand:
        return clipper_intersection(self.storage(), subject, clip)

    def split(self, subject: ClipperOperand, clip: ClipperOperand) -> tuple[ClipperOperand, ClipperOperand]:
        """Return ``(inside, outside)`` operands sharing their boundary along clip."""
        return clipper_split(self.storage(), subject, clip)

    def diff_with_safety_offset(self, subject: ClipperOperand, clip: ClipperOperand) -> ClipperOperand:
        return clipper_diff_with_safety_offset(self.storage(), subject, clip)

    def intersection_with_safety_offset(self, subject: ClipperOperand, clip: ClipperOperand) -> ClipperOperand:
        return clipper_intersection_with_safety_offset(self.storage(), subject, clip)

    def union(self, subject: ClipperOperand) -> ClipperOperand:
        return clipper_union(self.storage(), subject)

    def union_with_safety_offset(self, subject: ClipperOperand) -> ClipperOperand:
        return clipper_union_with_safety_offset(self.storage(), subject)

    def union2(self, subject1: ClipperOperand, subject2: ClipperOperand) -> ClipperOperand:
        return clipper_union2(self.storage(), subject1, subject2)

    def concat(self, first: ClipperOperand, second: ClipperOperand) -> ClipperOperand:
        return clipper_concat(self.storage(), first, second)

    def offset(self,
               subject: ClipperOperand,
               delta: float,
               join_type: int = CLIPPER_JOIN_MITER,
               miter_limit: float = 3.0,
               end_type: int = CLIPPER_END_CLOSED_POLYGON) -> ClipperOperand:
        return clipper_offset(self.storage(), subject, delta, join_type, miter_limit, end_type)

    def offset2(self,
                subject: ClipperOperand,
                delta1: float,
                delta2: float,
                join_type: int = CLIPPER_JOIN_MITER,
                miter_limit: float = 3.0,
                end_type: int = CLIPPER_END_CLOSED_POLYGON) -> ClipperOperand:
        return clipper_offset2(self.storage(), subject, delta1, delta2, join_type, miter_limit, end_type)

    def offset_polylines(self,
                         polylines: PolylineCollection,
                         delta: float,
                         join_type: int = CLIPPER_JOIN_SQUARE,
                         miter_limit: float = 0.0,
                         end_type: int = CLIPPER_END_OPEN_BUTT) -> ClipperOperand:
        """Offset open polylines like the native offset(Polylines); see clipper_offset_polylines()."""
        if polylines.api is not self.api:
            raise ValueError("polylines must use this context's API instance")
        return clipper_offset_polylines(self.storage(), polylines, delta, join_type, miter_limit, end_type)

    def clip_shapes_with_subject_bbox(self, subject: ClipperOperand, bbox) -> ClipperOperand:
        return clipper_clip_shapes_with_subject_bbox(self.storage(), subject, bbox)

    def diff_polyline_expolygons(
        self, subject: Polyline, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Return storage-owned pieces outside clip using this context's storage."""
        if subject.api is not self.api or clip.api is not self.api:
            raise ValueError("subject and clip must use this context's API instance")
        return clipper_diff_polyline_expolygons(self.storage(), subject, clip)

    def intersection_polyline_expolygons(
        self, subject: Polyline, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Return storage-owned pieces inside clip using this context's storage."""
        if subject.api is not self.api or clip.api is not self.api:
            raise ValueError("subject and clip must use this context's API instance")
        return clipper_intersection_polyline_expolygons(
            self.storage(), subject, clip
        )

    def diff_polylines_expolygons(
        self, subject: PolylineCollection, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Use this context's storage for the whole batch, not one host call per subject."""
        if subject.api is not self.api or clip.api is not self.api:
            raise ValueError("subject and clip must use this context's API instance")
        return clipper_diff_polylines_expolygons(self.storage(), subject, clip)

    def intersection_polylines_expolygons(
        self, subject: PolylineCollection, clip: ExPolygonCollection
    ) -> StoredPolylineCollection:
        """Use this context's storage for the whole batch, not one host call per subject."""
        if subject.api is not self.api or clip.api is not self.api:
            raise ValueError("subject and clip must use this context's API instance")
        return clipper_intersection_polylines_expolygons(self.storage(), subject, clip)

    def split_polyline_expolygons(
        self, subject: Polyline, clip: ExPolygonCollection
    ) -> tuple[StoredPolylineCollection, StoredPolylineCollection]:
        """Return storage-owned ``(inside, outside)`` pieces cut at the same points."""
        if subject.api is not self.api or clip.api is not self.api:
            raise ValueError("subject and clip must use this context's API instance")
        return clipper_split_polyline_expolygons(self.storage(), subject, clip)

    def clip_expolygons_with_subject_bbox(
        self, src: ExPolygonCollection, bbox: CBoundingBox
    ) -> StoredExPolygonCollection:
        """Return a storage-owned bbox-clipped copy using this context's storage."""
        if src.api is not self.api:
            raise ValueError("source collection must use this context's API instance")
        return clipper_clip_expolygons_with_subject_bbox(self.storage(), src, bbox)


__all__ = [
    "CLIPPER_END_CLOSED_POLYGON",
    "CLIPPER_END_CLOSED_LINE",
    "CLIPPER_END_OPEN_BUTT",
    "CLIPPER_END_OPEN_ROUND",
    "CLIPPER_END_OPEN_SQUARE",
    "CLIPPER_JOIN_MITER",
    "CLIPPER_JOIN_ROUND",
    "CLIPPER_JOIN_SQUARE",
    "CLIPPER_OPERATION_DIFFERENCE",
    "CLIPPER_OPERATION_INTERSECTION",
    "CLIPPER_OPERATION_UNION",
    "CLIPPER_OPERATION_XOR",
    "ClipperContext",
    "ClipperOperand",
    "clipper_concat",
    "clipper_clip_expolygons_with_subject_bbox",
    "clipper_clip_shapes_with_subject_bbox",
    "clipper_diff",
    "clipper_diff_polyline_expolygons",
    "clipper_diff_polylines_expolygons",
    "clipper_diff_with_safety_offset",
    "clipper_execute",
    "clipper_intersection",
    "clipper_intersection_polyline_expolygons",
    "clipper_intersection_polylines_expolygons",
    "clipper_intersection_with_safety_offset",
    "clipper_offset",
    "clipper_offset2",
    "clipper_offset_polylines",
    "clipper_split",
    "clipper_split_polyline_expolygons",
    "clipper_union",
    "clipper_union2",
    "clipper_union_with_safety_offset",
]
