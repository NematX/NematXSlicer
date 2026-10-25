#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python geometry view helpers over the SuperSlicer plugin C ABI.

The classes in this file wrap raw C handles in small Python objects so plugin
code can manipulate geometry through methods instead of calling api.host.*
manually.

Typical use
-----------

Borrow a polygon received from the host:

    polygon = api.polygon(polygon_handle)
    if polygon.valid_polygon():
        for point in polygon:
            print(point.x, point.y)

Create temporary geometry owned by the current storage:

    with api.new_polyline(storage_handle) as line:
        line.push_back(make_point(0, 0))
        line.push_back(make_point(1000000, 0))
        print(line.length())

Copy or move geometry into a collection:

    with api.new_polygon_collection(storage_handle) as polygons:
        polygons.push_back(existing_polygon_view)
        polygons.insert_move(api.new_polygon(storage_handle), 0)

Read an ExPolygon collection:

    for expolygon in api.expolygon_collection(collection_handle):
        contour = expolygon.contour()
        holes = list(expolygon.holes())

Lifetime model
--------------
- MultiPoint, Polygon, Polyline, PolygonCollection, PolylineCollection,
  ExPolygon and ExPolygonCollection are borrowed read-only views. They never
  free the handle they wrap.
- MutablePolygon, MutablePolyline, MutableExPolygon and
  MutableExPolygonCollection are borrowed mutable views. Use them only when a
  step payload or host callback explicitly gives mutable access to host-owned
  geometry.
- StoredPolygon, StoredPolyline, StoredPolygonCollection,
  StoredPolylineCollection, StoredExPolygon and StoredExPolygonCollection own a
  mutable handle allocated in a storage_handle. They call storage_free() when
  freed or garbage-collected. Assignment shares one wrapper normally, but
  copy.copy() and copy.deepcopy() are forbidden because ownership is unique.
  Use an explicit native clone/copy operation when independent geometry is
  required.
- Views returned from collections are invalidated when the owning collection is
  resized, cleared, moved from, extracted from, or destroyed.

Const-correctness model
-----------------------
- Borrowed wrappers are read-only by design. If the host gives a const handle,
  wrap it with Polygon, Polyline, ExPolygon or the matching collection view.
- Stored* wrappers expose mutation methods because they own a mutable handle
  allocated in a storage_handle.
- Stored*.adopt_owned() is intentionally not exposed through Slic3rAPI. It is
  only for low-level helpers that receive a mutable handle whose ownership was
  explicitly transferred to the plugin. Normal plugin code should use api.xxx()
  for borrowed views and api.new_xxx(storage) for owned geometry.
"""

from __future__ import annotations

import ctypes
import math
from typing import Iterator, Sequence

from slic3r_api_generated import (
    CBoundingBox,
    CPoint,
    EXPOLYGON_STATUS_OK,
    MULTIPOINT_CCW,
    MULTIPOINT_CLOSED,
    MULTIPOINT_CW,
    MultipointConstView,
    MultipointView,
    SCALED_EPSILON,
)
from slic3r_abi import abi_requires_headers


PI = 3.141592653589793238


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


def make_point(x: int, y: int) -> CPoint:
    return CPoint(int(x), int(y))


@abi_requires_headers("slic3r_geometry.h")
class LineDistancer:
    """
    Copy polygon boundaries and answer distances to their finite segments.

    Constructing the distancer copies the scaled integer points of every outer
    contour and hole from an ``ExPolygonCollection``. The source collection is
    borrowed and remains owned by its host or storage, but may be changed or
    freed immediately after construction without invalidating this object.
    ``LineDistancer`` owns only ordinary Python values; it allocates no plugin
    handle and needs no explicit release.

    Distances and radii use scaled ``CPoint`` units, not millimeters. Queries
    perform a linear scan over all copied segments, making them O(segment
    count); this helper is intended for modest boundary sets and does not build
    a spatial index. Signed distances are negative only inside an outer contour
    and outside all of that contour's holes. Points on a boundary have distance
    zero, whose sign is not significant.
    """

    def __init__(self, areas: "ExPolygonCollection | None" = None) -> None:
        self._segments: list[tuple[tuple[int, int], tuple[int, int]]] = []
        self._areas: list[tuple[list[tuple[int, int]], list[list[tuple[int, int]]]]] = []
        if areas is not None:
            for area in areas:
                self._append_area(area)

    def empty(self) -> bool:
        """Return whether no usable boundary segment has been stored."""
        return not self._segments

    def append_segment(self, start, end) -> None:
        """
        Append one finite segment after copying its two scaled endpoints.

        Manually appended segments participate in distance/radius queries but
        not in signed inside/outside classification, because they do not define
        a complete polygonal area.
        """
        a = as_point(start)
        b = as_point(end)
        self._segments.append(((int(a.x), int(a.y)), (int(b.x), int(b.y))))

    def distance_from_lines(self, point, signed_distance: bool = False) -> float:
        """
        Return the shortest distance from ``point`` to a stored finite segment.

        An empty distancer returns positive infinity. Set ``signed_distance``
        to negate the result inside one copied ExPolygon area. The method does
        not mutate the query point, source geometry, or stored boundaries.
        """
        if not self._segments:
            return math.inf
        query = as_point(point)
        query_xy = (int(query.x), int(query.y))
        best_squared = min(
            self._segment_distance_squared(query_xy, segment)
            for segment in self._segments
        )
        distance = math.sqrt(best_squared)
        return -distance if signed_distance and self._contains(query_xy) else distance

    def all_lines_in_radius(self, point, radius: float) -> list[int]:
        """
        Return segment indices whose finite distance is at most ``radius``.

        Indices follow segment insertion order. A negative radius returns an
        empty list. The method performs no allocation in plugin storage; the
        returned Python list is owned entirely by the caller.
        """
        radius = float(radius)
        if radius < 0.0:
            return []
        if not math.isfinite(radius):
            if radius == math.inf:
                return list(range(len(self._segments)))
            return []
        query = as_point(point)
        query_xy = (int(query.x), int(query.y))
        radius_squared = radius * radius
        return [
            idx
            for idx, segment in enumerate(self._segments)
            if self._segment_distance_squared(query_xy, segment) <= radius_squared
        ]

    def _append_area(self, area: "ExPolygon") -> None:
        contour = [(int(point.x), int(point.y)) for point in area.contour()]
        holes = [
            [(int(point.x), int(point.y)) for point in hole]
            for hole in area.holes()
        ]
        self._append_polygon_segments(contour)
        for hole in holes:
            self._append_polygon_segments(hole)
        self._areas.append((contour, holes))

    def _append_polygon_segments(self, points: list[tuple[int, int]]) -> None:
        if len(points) < 2:
            return
        self._segments.extend(zip(points[:-1], points[1:]))
        if points[0] != points[-1]:
            self._segments.append((points[-1], points[0]))

    @staticmethod
    def _segment_distance_squared(
        point: tuple[int, int],
        segment: tuple[tuple[int, int], tuple[int, int]],
    ) -> float:
        (ax, ay), (bx, by) = segment
        px, py = point
        dx = float(bx - ax)
        dy = float(by - ay)
        length_squared = dx * dx + dy * dy
        projection = 0.0
        if length_squared > 0.0:
            projection = ((px - ax) * dx + (py - ay) * dy) / length_squared
        projection = max(0.0, min(1.0, projection))
        nearest_x = ax + projection * dx
        nearest_y = ay + projection * dy
        return (px - nearest_x) ** 2 + (py - nearest_y) ** 2

    @staticmethod
    def _polygon_contains(polygon: list[tuple[int, int]], point: tuple[int, int]) -> bool:
        if len(polygon) < 3:
            return False
        px, py = point
        inside = False
        previous_x, previous_y = polygon[-1]
        for current_x, current_y in polygon:
            crosses = (current_y > py) != (previous_y > py)
            if crosses:
                x_at_y = (
                    (previous_x - current_x) * (py - current_y) /
                    (previous_y - current_y) + current_x
                )
                if px < x_at_y:
                    inside = not inside
            previous_x, previous_y = current_x, current_y
        return inside

    def _contains(self, point: tuple[int, int]) -> bool:
        for contour, holes in self._areas:
            if self._polygon_contains(contour, point) and not any(
                self._polygon_contains(hole, point) for hole in holes
            ):
                return True
        return False


def as_point(point) -> CPoint:
    if isinstance(point, CPoint):
        return point
    x, y = point
    return make_point(x, y)


def point_tuple(point: CPoint) -> tuple[int, int]:
    return int(point.x), int(point.y)


def point_add(lhs: CPoint, rhs: CPoint) -> CPoint:
    return make_point(lhs.x + rhs.x, lhs.y + rhs.y)


def point_sub(lhs: CPoint, rhs: CPoint) -> CPoint:
    return make_point(lhs.x - rhs.x, lhs.y - rhs.y)


def midpoint(p1: CPoint, p2: CPoint) -> CPoint:
    return make_point((p1.x + p2.x) // 2, (p1.y + p2.y) // 2)


def norm_square(point: CPoint) -> float:
    return float(point.x) * float(point.x) + float(point.y) * float(point.y)


def norm(point: CPoint) -> float:
    return math.sqrt(norm_square(point))


def point_at(p1: CPoint, p2: CPoint, distance: float) -> CPoint:
    direction = point_sub(p2, p1)
    length = norm(direction)
    if length == 0:
        return make_point(p1.x, p1.y)
    x = p1.x
    y = p1.y
    if p1.x != p2.x:
        x = int(float(p1.x) + float(direction.x) * float(distance) / length)
    if p1.y != p2.y:
        y = int(float(p1.y) + float(direction.y) * float(distance) / length)
    return make_point(x, y)


def extend_start(p1: CPoint, p2: CPoint, distance: float) -> CPoint:
    return point_at(p1, p2, -distance)


def extend_end(p1: CPoint, p2: CPoint, distance: float) -> CPoint:
    return point_at(p2, p1, -distance)


def abs_angle(rad: float) -> float:
    return rad + 2.0 * PI if rad <= 0 else rad


def angle_ccw(v1: CPoint, v2: CPoint) -> float:
    det = float(v1.x) * float(v2.y) - float(v1.y) * float(v2.x)
    dot = float(v1.x) * float(v2.x) + float(v1.y) * float(v2.y)
    return math.atan2(det, dot)


def multipoint_view_as_const(view: MultipointView) -> MultipointConstView:
    return MultipointConstView(view.array, view.size, view.flags)


def multipoint_views_equal(lhs, rhs) -> bool:
    lhs_const = multipoint_view_as_const(lhs) if isinstance(lhs, MultipointView) else lhs
    rhs_const = multipoint_view_as_const(rhs) if isinstance(rhs, MultipointView) else rhs
    if lhs_const.size != rhs_const.size:
        return False
    for idx in range(lhs_const.size):
        if lhs_const.array[idx].x != rhs_const.array[idx].x:
            return False
        if lhs_const.array[idx].y != rhs_const.array[idx].y:
            return False
    return True


# Common base for borrowed views. It stores the API object and a raw handle, but
# never owns or frees the pointed geometry.
class HandleView:
    def __init__(self, api, handle) -> None:
        self.api = api
        self._handle = _require_handle(handle)

    @property
    def address(self) -> int:
        return self._handle

    def handle(self) -> int:
        return self._handle

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self._handle)

    def is_valid_handle(self) -> bool:
        return self._handle != 0

    def same_handle(self, other: "HandleView") -> bool:
        return self._handle == other.handle()

    def __bool__(self) -> bool:
        return self.is_valid_handle()


class _UniqueOwnerMixin:
    """Reject Python object copying for wrappers with unique native ownership."""

    def __copy__(self):
        raise TypeError(
            f"{type(self).__name__} owns a unique native resource and cannot be copied"
        )

    def __deepcopy__(self, memo):
        raise TypeError(
            f"{type(self).__name__} owns a unique native resource and cannot be copied"
        )


# Ownership mixin for Stored* wrappers. The handle must have been allocated in a
# storage_handle, and the mixin releases it with storage_free() when possible.
class StoredHandleMixin(_UniqueOwnerMixin):
    def _init_storage(self, storage) -> None:
        self._storage = _require_handle(storage, "storage")
        self._owns_handle = True

    @property
    def storage_address(self) -> int:
        return self._storage

    def storage(self) -> int:
        return self._storage

    def mutable_handle(self) -> int:
        return self._handle

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return _void_p(self._handle)

    def release(self) -> int:
        handle = self._handle
        self._handle = 0
        self._owns_handle = False
        return handle

    def free_from_storage(self) -> bool:
        if not getattr(self, "_owns_handle", False) or not self._handle:
            return False
        freed = bool(self.api.host.storage_free(_void_p(self._storage), _void_p(self._handle)))
        if freed:
            self._handle = 0
            self._owns_handle = False
        return freed

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.free_from_storage()

    def __del__(self) -> None:
        try:
            self.free_from_storage()
        except Exception:
            pass


# Shared read-only point-sequence API. Polygon and Polyline expose this API
# through their underlying multipoint handle, so most point algorithms can work
# on either type.
@abi_requires_headers("slic3r_geometry.h")
class MultiPointReadMixin:
    def multipoint_handle(self) -> int:
        return self.handle()

    def size(self) -> int:
        return int(self.api.host.multipoint_size(_void_p(self.multipoint_handle())))

    def empty(self) -> bool:
        return self.size() == 0

    def is_valid(self) -> bool:
        return bool(self.api.host.multipoint_valid(_void_p(self.multipoint_handle())))

    def at(self, idx: int) -> CPoint:
        return self.api.host.multipoint_get(_void_p(self.multipoint_handle()), int(idx))

    def front(self) -> CPoint:
        if self.empty():
            raise IndexError("empty multipoint")
        return self.at(0)

    def back(self) -> CPoint:
        if self.empty():
            raise IndexError("empty multipoint")
        return self.at(self.size() - 1)

    def length(self) -> float:
        return float(self.api.host.multipoint_length(_void_p(self.multipoint_handle())))

    def view(self) -> MultipointConstView:
        return self.api.host.multipoint_view_const(_void_p(self.multipoint_handle()))

    def bounding_box(self) -> CBoundingBox:
        view = self.view()
        return self.api.host.points_bounding_box(ctypes.byref(view))

    def find_point_index(self, point_search, max_distance: int) -> int:
        view = self.view()
        return int(self.api.host.points_find_point_index(ctypes.byref(view), as_point(point_search), int(max_distance)))

    def closest_point_index(self, point_search) -> int:
        view = self.view()
        return int(self.api.host.points_closest_point_index(ctypes.byref(view), as_point(point_search)))

    def points(self) -> list[CPoint]:
        return [self.at(idx) for idx in range(self.size())]

    def __len__(self) -> int:
        return self.size()

    def __iter__(self) -> Iterator[CPoint]:
        for idx in range(self.size()):
            yield self.at(idx)

    def __getitem__(self, idx: int) -> CPoint:
        if idx < 0:
            idx += self.size()
        if idx < 0 or idx >= self.size():
            raise IndexError(idx)
        return self.at(idx)


# Shared mutable point-sequence API. Use it only on geometry you own in the
# current storage, or on borrowed geometry that the host explicitly exposed as
# mutable for the current step.
@abi_requires_headers("slic3r_geometry.h")
class MultiPointMutableMixin:
    def mutable_multipoint_handle(self) -> int:
        return self.mutable_handle()

    def set(self, idx: int, point) -> bool:
        return bool(self.api.host.multipoint_set(_void_p(self.mutable_multipoint_handle()), int(idx), as_point(point)))

    def is_cw(self) -> bool:
        view = self.api.host.multipoint_view_mut(_void_p(self.mutable_multipoint_handle()))
        return bool(self.api.host.multipoint_is_cw(ctypes.byref(view)))

    def push_back(self, point) -> None:
        self.api.host.multipoint_push_back(_void_p(self.mutable_multipoint_handle()), as_point(point))

    def insert(self, idx: int, point) -> None:
        self.api.host.multipoint_insert(_void_p(self.mutable_multipoint_handle()), int(idx), as_point(point))

    def insert_array(self, idx: int, points: Sequence[CPoint]) -> None:
        converted = [as_point(point) for point in points]
        array = (CPoint * len(converted))(*converted)
        self.api.host.multipoint_insert_array(
            _void_p(self.mutable_multipoint_handle()), int(idx), array, len(converted)
        )

    def pop_back(self) -> None:
        self.api.host.multipoint_pop_back(_void_p(self.mutable_multipoint_handle()))

    def clear(self) -> None:
        self.api.host.multipoint_clear(_void_p(self.mutable_multipoint_handle()))

    def erase(self, idx: int, erase_size: int = 1) -> None:
        self.api.host.multipoint_erase(_void_p(self.mutable_multipoint_handle()), int(idx), int(erase_size))

    def copy_from(self, other: MultiPointReadMixin) -> None:
        self.api.host.multipoint_copy(_void_p(self.mutable_multipoint_handle()), _void_p(other.multipoint_handle()))

    def scale(self, factor: float) -> None:
        self.api.host.multipoint_scale(_void_p(self.mutable_multipoint_handle()), float(factor))

    def scale_xy(self, factor_x: float, factor_y: float) -> None:
        self.api.host.multipoint_scale_xy(_void_p(self.mutable_multipoint_handle()), float(factor_x), float(factor_y))

    def translate(self, x: float, y: float) -> None:
        self.api.host.multipoint_translate(_void_p(self.mutable_multipoint_handle()), float(x), float(y))

    def rotate_origin(self, angle: float) -> None:
        self.api.host.multipoint_rotate_origin(_void_p(self.mutable_multipoint_handle()), float(angle))

    def rotate_around(self, angle: float, center) -> None:
        self.api.host.multipoint_rotate_around(_void_p(self.mutable_multipoint_handle()), float(angle), as_point(center))

    def reverse(self) -> None:
        self.api.host.multipoint_reverse(_void_p(self.mutable_multipoint_handle()))

    def densify(self, min_length: int) -> None:
        self.api.host.multipoint_densify(_void_p(self.mutable_multipoint_handle()), int(min_length))

    def simplify(self, min_length: int, min_deviation: int) -> None:
        self.api.host.multipoint_simplify(
            _void_p(self.mutable_multipoint_handle()), int(min_length), int(min_deviation)
        )

    def ensure_valid(self, resolution: int = SCALED_EPSILON) -> None:
        self.api.host.multipoint_ensure_valid(_void_p(self.mutable_multipoint_handle()), int(resolution))


# Borrowed view over an arbitrary point sequence. Use this when the host gives
# you a multipoint_handle directly and you only need to read it.
class MultiPoint(HandleView, MultiPointReadMixin):
    pass


# Borrowed read-only polygon. It is useful for inspecting contour points,
# orientation, area and containment without taking ownership of the polygon.
class Polygon(HandleView, MultiPointReadMixin):
    def multipoint_handle(self) -> int:
        return _address(self.api.host.polygon_as_multipoint_const(self.c_handle()))

    def valid_polygon(self) -> bool:
        return bool(self.api.host.polygon_valid(self.c_handle()))

    def area(self) -> float:
        return float(self.api.host.polygon_area(self.c_handle()))

    def is_counter_clockwise(self) -> bool:
        return bool(self.api.host.polygon_is_counter_clockwise(self.c_handle()))

    def is_clockwise(self) -> bool:
        return bool(self.api.host.polygon_is_clockwise(self.c_handle()))

    def contains(self, point) -> bool:
        return bool(self.api.host.polygon_contains(self.c_handle(), as_point(point)))

    def on_boundary(self, point, max_dist: int) -> bool:
        return bool(self.api.host.polygon_on_boundary(self.c_handle(), as_point(point), int(max_dist)))

    def centroid(self) -> CPoint:
        return self.api.host.polygon_centroid(self.c_handle())

    def convex_points_idx(self, min_angle: float, max_angle: float) -> list[int]:
        out = (ctypes.c_uint32 * self.size())()
        count = int(self.api.host.polygon_convex_points_idx(
            self.c_handle(), float(min_angle), float(max_angle), out, len(out)
        ))
        return [int(out[idx]) for idx in range(count)]

    def concave_points_idx(self, min_angle: float, max_angle: float) -> list[int]:
        out = (ctypes.c_uint32 * self.size())()
        count = int(self.api.host.polygon_concave_points_idx(
            self.c_handle(), float(min_angle), float(max_angle), out, len(out)
        ))
        return [int(out[idx]) for idx in range(count)]


# Borrowed mutable polygon. Use it only when a step payload or another mutable
# view explicitly gives mutable access to host-owned geometry.
class MutablePolygon(Polygon, MultiPointMutableMixin):
    def mutable_handle(self) -> int:
        return self.handle()

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def mutable_multipoint_handle(self) -> int:
        return _address(self.api.host.polygon_as_multipoint(self.mutable_c_handle()))

    def make_counter_clockwise(self) -> bool:
        return bool(self.api.host.polygon_make_counter_clockwise(self.mutable_c_handle()))

    def make_clockwise(self) -> bool:
        return bool(self.api.host.polygon_make_clockwise(self.mutable_c_handle()))


# Owned mutable polygon allocated in a storage_handle. Use it for temporary
# plugin geometry, or adopt_owned() an existing mutable handle when the host
# transfers ownership to the plugin.
class StoredPolygon(Polygon, StoredHandleMixin, MultiPointMutableMixin):
    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_polygon(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredPolygon":
        """Take ownership of a mutable polygon handle allocated in storage."""
        out = cls.__new__(cls)
        Polygon.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    def mutable_multipoint_handle(self) -> int:
        return _address(self.api.host.polygon_as_multipoint(self.mutable_c_handle()))

    def move_from(self, other: "StoredPolygon") -> None:
        self.api.host.polygon_move(self.mutable_c_handle(), other.mutable_c_handle())

    def make_counter_clockwise(self) -> bool:
        return bool(self.api.host.polygon_make_counter_clockwise(self.mutable_c_handle()))

    def make_clockwise(self) -> bool:
        return bool(self.api.host.polygon_make_clockwise(self.mutable_c_handle()))


# Borrowed read-only polyline. It has the same point iteration helpers as
# Polygon, but keeps the polyline-specific validity rules.
class Polyline(HandleView, MultiPointReadMixin):
    def multipoint_handle(self) -> int:
        return _address(self.api.host.polyline_as_multipoint_const(self.c_handle()))

    def valid_polyline(self) -> bool:
        return bool(self.api.host.polyline_valid(self.c_handle()))


# Borrowed mutable polyline. Use it only when a step payload or another mutable
# view explicitly gives mutable access to host-owned geometry.
class MutablePolyline(Polyline, MultiPointMutableMixin):
    def mutable_handle(self) -> int:
        return self.handle()

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def mutable_multipoint_handle(self) -> int:
        return _address(self.api.host.polyline_as_multipoint(self.mutable_c_handle()))

    def clip_end(self, distance: float) -> None:
        self.api.host.polyline_clip_end(self.mutable_c_handle(), float(distance))

    def clip_start(self, distance: float) -> None:
        self.api.host.polyline_clip_start(self.mutable_c_handle(), float(distance))

    def extend_end(self, distance: float) -> None:
        self.api.host.polyline_extend_end(self.mutable_c_handle(), float(distance))

    def extend_start(self, distance: float) -> None:
        self.api.host.polyline_extend_start(self.mutable_c_handle(), float(distance))


# Owned mutable polyline allocated in a storage_handle. Use it when a plugin
# needs to build, edit, clip or extend a polyline before giving it back to the
# host.
class StoredPolyline(Polyline, StoredHandleMixin, MultiPointMutableMixin):
    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_polyline(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredPolyline":
        """Take ownership of a mutable polyline handle allocated in storage."""
        out = cls.__new__(cls)
        Polyline.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    def mutable_multipoint_handle(self) -> int:
        return _address(self.api.host.polyline_as_multipoint(self.mutable_c_handle()))

    def move_from(self, other: "StoredPolyline") -> None:
        self.api.host.polyline_move(self.mutable_c_handle(), other.mutable_c_handle())

    def clip_end(self, distance: float) -> None:
        self.api.host.polyline_clip_end(self.mutable_c_handle(), float(distance))

    def clip_start(self, distance: float) -> None:
        self.api.host.polyline_clip_start(self.mutable_c_handle(), float(distance))

    def extend_end(self, distance: float) -> None:
        self.api.host.polyline_extend_end(self.mutable_c_handle(), float(distance))

    def extend_start(self, distance: float) -> None:
        self.api.host.polyline_extend_start(self.mutable_c_handle(), float(distance))


# Shared read-only collection API. Iteration returns borrowed element views; do
# not keep those views after the collection is modified or destroyed.
@abi_requires_headers("slic3r_geometry.h")
class CollectionReadMixin:
    _view_cls = None
    _stored_collection_cls = None
    _size_fn = ""
    _valid_fn = ""
    _at_const_fn = ""

    def size(self) -> int:
        return int(getattr(self.api.host, self._size_fn)(self.c_handle()))

    def empty(self) -> bool:
        return self.size() == 0

    def valid_collection(self) -> bool:
        return bool(getattr(self.api.host, self._valid_fn)(self.c_handle()))

    def view_at(self, idx: int):
        handle = getattr(self.api.host, self._at_const_fn)(self.c_handle(), int(idx))
        return self._view_cls(self.api, handle)

    def at(self, idx: int):
        return self.view_at(idx)

    def front(self):
        if self.empty():
            raise IndexError("empty collection")
        return self.view_at(0)

    def back(self):
        if self.empty():
            raise IndexError("empty collection")
        return self.view_at(self.size() - 1)

    def clone(self, storage):
        out = self._stored_collection_cls(self.api, storage)
        out.copy_from(self)
        return out

    def __len__(self) -> int:
        return self.size()

    def __iter__(self):
        for idx in range(self.size()):
            yield self.view_at(idx)

    def __getitem__(self, idx: int):
        if idx < 0:
            idx += self.size()
        if idx < 0 or idx >= self.size():
            raise IndexError(idx)
            return self.view_at(idx)


# Shared mutable collection API for Stored*Collection classes. Copy methods keep
# the source unchanged; move methods leave the source object or element empty.
@abi_requires_headers("slic3r_geometry.h", "slic3r_utils.h")
class StoredCollectionMixin:
    _stored_value_cls = None
    _clear_fn = ""
    _copy_fn = ""
    _move_fn = ""
    _append_copy_fn = ""
    _append_move_fn = ""
    _insert_copy_fn = ""
    _insert_move_fn = ""
    _erase_fn = ""
    _at_mut_fn = ""
    _move_element_fn = ""

    def clear(self) -> None:
        getattr(self.api.host, self._clear_fn)(self.mutable_c_handle())

    def copy_from(self, other) -> None:
        getattr(self.api.host, self._copy_fn)(self.mutable_c_handle(), other.c_handle())

    def move_from(self, other) -> None:
        getattr(self.api.host, self._move_fn)(self.mutable_c_handle(), other.mutable_c_handle())

    def append_copy_from(self, other) -> None:
        getattr(self.api.host, self._append_copy_fn)(self.mutable_c_handle(), other.c_handle())

    def append_move_from(self, other) -> None:
        getattr(self.api.host, self._append_move_fn)(self.mutable_c_handle(), other.mutable_c_handle())

    def push_back(self, value) -> None:
        self.insert(value, self.size())

    def insert(self, value, idx: int) -> None:
        getattr(self.api.host, self._insert_copy_fn)(self.mutable_c_handle(), int(idx), value.c_handle())

    def insert_move(self, value, idx: int) -> None:
        getattr(self.api.host, self._insert_move_fn)(self.mutable_c_handle(), int(idx), value.mutable_c_handle())

    def erase(self, idx: int, erase_size: int = 1) -> None:
        getattr(self.api.host, self._erase_fn)(self.mutable_c_handle(), int(idx), int(erase_size))

    def clone_at(self, idx: int):
        out = self._stored_value_cls(self.api, self.storage())
        out.copy_from(self.view_at(idx))
        return out

    def extract(self, idx: int):
        out = self._stored_value_cls(self.api, self.storage())
        src = getattr(self.api.host, self._at_mut_fn)(self.mutable_c_handle(), int(idx))
        getattr(self.api.host, self._move_element_fn)(out.mutable_c_handle(), _void_p(src))
        self.erase(idx)
        return out


# Borrowed read-only collection of polygons. Use this for plugin inputs that are
# owned by the host or by another StoredPolygonCollection.
class PolygonCollection(HandleView, CollectionReadMixin):
    _view_cls = Polygon
    _size_fn = "polygons_size"
    _valid_fn = "polygons_valid"
    _at_const_fn = "polygons_at_const"


# Owned mutable polygon collection allocated in a storage_handle. The collection
# owns its container; element views returned by at()/iteration are borrowed.
class StoredPolygonCollection(PolygonCollection, StoredHandleMixin, StoredCollectionMixin):
    _stored_value_cls = StoredPolygon
    _clear_fn = "polygons_clear"
    _copy_fn = "polygons_copy"
    _move_fn = "polygons_move"
    _append_copy_fn = "polygons_append_copy"
    _append_move_fn = "polygons_append_move"
    _insert_copy_fn = "polygons_insert_copy"
    _insert_move_fn = "polygons_insert_move"
    _erase_fn = "polygons_erase"
    _at_mut_fn = "polygons_at"
    _move_element_fn = "polygon_move"

    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_polygons(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredPolygonCollection":
        """Take ownership of a mutable polygon collection handle."""
        out = cls.__new__(cls)
        PolygonCollection.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    # Native collection clean-ups; see polygons_ensure_valid() and the
    # following functions in slic3r_geometry.h for the exact rules.
    def ensure_valid(self, resolution: int = SCALED_EPSILON) -> None:
        self.api.host.polygons_ensure_valid(self.mutable_c_handle(), int(resolution))

    def remove_sticks(self) -> bool:
        return bool(self.api.host.polygons_remove_sticks(self.mutable_c_handle()))

    def remove_degenerate(self) -> bool:
        return bool(self.api.host.polygons_remove_degenerate(self.mutable_c_handle()))

    def smooth_outward(self, clip_distance: int) -> None:
        self.api.host.polygons_smooth_outward(self.mutable_c_handle(), int(clip_distance))


PolygonCollection._stored_collection_cls = StoredPolygonCollection


# Borrowed read-only collection of polylines. Use this for host-provided
# polyline lists without taking ownership.
class PolylineCollection(HandleView, CollectionReadMixin):
    _view_cls = Polyline
    _size_fn = "polylines_size"
    _valid_fn = "polylines_valid"
    _at_const_fn = "polylines_at_const"


# Owned mutable polyline collection allocated in a storage_handle. Use it to
# accumulate generated paths before moving/copying them to a host output.
class StoredPolylineCollection(PolylineCollection, StoredHandleMixin, StoredCollectionMixin):
    _stored_value_cls = StoredPolyline
    _clear_fn = "polylines_clear"
    _copy_fn = "polylines_copy"
    _move_fn = "polylines_move"
    _append_copy_fn = "polylines_append_copy"
    _append_move_fn = "polylines_append_move"
    _insert_copy_fn = "polylines_insert_copy"
    _insert_move_fn = "polylines_insert_move"
    _erase_fn = "polylines_erase"
    _at_mut_fn = "polylines_at"
    _move_element_fn = "polyline_move"

    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_polylines(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredPolylineCollection":
        """Take ownership of a mutable polyline collection handle."""
        out = cls.__new__(cls)
        PolylineCollection.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    # Native collection clean-ups; see polylines_ensure_valid() in slic3r_geometry.h.
    def ensure_valid(self, resolution: int = SCALED_EPSILON) -> None:
        self.api.host.polylines_ensure_valid(self.mutable_c_handle(), int(resolution))

    def remove_degenerate(self) -> bool:
        return bool(self.api.host.polylines_remove_degenerate(self.mutable_c_handle()))


PolylineCollection._stored_collection_cls = StoredPolylineCollection


# Borrowed read-only ExPolygon. It exposes a contour polygon and borrowed hole
# polygons; the returned polygon views are valid only while this ExPolygon stays
# valid.
class ExPolygon(HandleView):
    def contour(self) -> Polygon:
        return Polygon(self.api, self.api.host.expolygon_contour_const(self.c_handle()))

    def hole_size(self) -> int:
        return int(self.api.host.expolygon_hole_size(self.c_handle()))

    def hole(self, idx: int) -> Polygon:
        return Polygon(self.api, self.api.host.expolygon_hole_at_const(self.c_handle(), int(idx)))

    def holes(self) -> Iterator[Polygon]:
        for idx in range(self.hole_size()):
            yield self.hole(idx)

    def area(self) -> float:
        return float(self.api.host.expolygon_area(self.c_handle()))

    def contains(self, point) -> bool:
        return bool(self.api.host.expolygon_contains(self.c_handle(), as_point(point)))

    def is_valid(self) -> bool:
        return int(self.api.host.expolygon_valid(self.c_handle())) == EXPOLYGON_STATUS_OK

    def overlaps(self, other: "ExPolygon") -> bool:
        return bool(self.api.host.expolygon_overlaps(self.c_handle(), other.c_handle()))


# Borrowed mutable ExPolygon. It exposes mutable hole/contour views without
# taking ownership of the ExPolygon itself.
class MutableExPolygon(ExPolygon):
    def mutable_handle(self) -> int:
        return self.handle()

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def contour_mutable(self) -> MutablePolygon:
        return MutablePolygon(self.api, self.api.host.expolygon_contour(self.mutable_c_handle()))

    def hole_mutable(self, idx: int) -> MutablePolygon:
        return MutablePolygon(self.api, self.api.host.expolygon_hole_at(self.mutable_c_handle(), int(idx)))

    def holes_mutable(self) -> Iterator[MutablePolygon]:
        for idx in range(self.hole_size()):
            yield self.hole_mutable(idx)


# Owned mutable ExPolygon allocated in a storage_handle. The contour and holes
# are still exposed as borrowed polygon views; resizing holes may invalidate
# previously returned hole views.
class StoredExPolygon(ExPolygon, StoredHandleMixin):
    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_expolygon(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredExPolygon":
        """Take ownership of a mutable ExPolygon handle allocated in storage."""
        out = cls.__new__(cls)
        ExPolygon.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    def holes_resize(self, new_size: int) -> None:
        self.api.host.expolygon_holes_resize(self.mutable_c_handle(), int(new_size))

    def holes_emplace_back(self) -> None:
        self.api.host.expolygon_holes_emplace_back(self.mutable_c_handle())

    def holes_pop_back(self) -> None:
        self.api.host.expolygon_holes_pop_back(self.mutable_c_handle())

    def holes_clear(self) -> None:
        self.api.host.expolygon_holes_clear(self.mutable_c_handle())

    def holes_erase(self, begin_idx: int, erase_size: int) -> None:
        self.api.host.expolygon_holes_erase(self.mutable_c_handle(), int(begin_idx), int(erase_size))

    def copy_from(self, other: ExPolygon) -> None:
        self.api.host.expolygon_copy(self.mutable_c_handle(), other.c_handle())

    def move_from(self, other: "StoredExPolygon") -> None:
        self.api.host.expolygon_move(self.mutable_c_handle(), other.mutable_c_handle())

    def ensure_valid(self, resolution: int = SCALED_EPSILON) -> None:
        self.api.host.expolygon_ensure_valid(self.mutable_c_handle(), int(resolution))


# Borrowed read-only collection of ExPolygons. Iteration returns borrowed
# ExPolygon views that follow the collection invalidation rules.
class ExPolygonCollection(HandleView, CollectionReadMixin):
    _view_cls = ExPolygon
    _size_fn = "expolygons_size"
    _valid_fn = "expolygons_valid"
    _at_const_fn = "expolygons_at_const"

    def valid_collection(self) -> bool:
        return int(self.api.host.expolygons_valid(self.c_handle())) == EXPOLYGON_STATUS_OK


# Borrowed mutable ExPolygon collection. Iteration remains read-only by default;
# call at_mutable()/mutable_items() when the step explicitly allows mutation.
class MutableExPolygonCollection(ExPolygonCollection):
    def mutable_handle(self) -> int:
        return self.handle()

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def at_mutable(self, idx: int) -> MutableExPolygon:
        return MutableExPolygon(self.api, self.api.host.expolygons_at(self.mutable_c_handle(), int(idx)))

    def mutable_items(self) -> Iterator[MutableExPolygon]:
        for idx in range(self.size()):
            yield self.at_mutable(idx)


# Owned mutable ExPolygon collection allocated in a storage_handle. This is the
# usual output container when a plugin creates or edits areas made of polygons
# with holes.
class StoredExPolygonCollection(ExPolygonCollection, StoredHandleMixin, StoredCollectionMixin):
    _stored_value_cls = StoredExPolygon
    _clear_fn = "expolygons_clear"
    _copy_fn = "expolygons_copy"
    _move_fn = "expolygons_move"
    _append_copy_fn = "expolygons_append_copy"
    _append_move_fn = "expolygons_append_move"
    _insert_copy_fn = "expolygons_insert_copy"
    _insert_move_fn = "expolygons_insert_move"
    _erase_fn = "expolygons_erase"
    _at_mut_fn = "expolygons_at"
    _move_element_fn = "expolygon_move"

    def __init__(self, api, storage) -> None:
        handle = api.host.storage_new_expolygons(_void_p(storage))
        super().__init__(api, handle)
        self._init_storage(storage)

    @classmethod
    def adopt_owned(cls, api, storage, handle) -> "StoredExPolygonCollection":
        """Take ownership of a mutable ExPolygon collection handle."""
        out = cls.__new__(cls)
        ExPolygonCollection.__init__(out, api, handle)
        out._init_storage(storage)
        return out

    def ensure_valid(self, resolution: int = SCALED_EPSILON) -> None:
        self.api.host.expolygons_ensure_valid(self.mutable_c_handle(), int(resolution))


ExPolygonCollection._stored_collection_cls = StoredExPolygonCollection


@abi_requires_headers("slic3r_geometry.h", "slic3r_utils.h")
def convex_hull(storage, polygon: Polygon) -> StoredPolygon:
    """
    Build a storage-owned convex hull from one borrowed polygon.

    The host copies the polygon's point cloud and ignores its orientation. The
    source remains owned by its current host or storage and is not modified.
    The result belongs to ``storage``; release it with ``free_from_storage()``
    or use it as a context manager before that storage is cleared or destroyed.
    Coordinates remain in scaled integer units.
    """
    storage_address = _require_handle(storage, "storage")
    result = polygon.api.host.polygon_convex_hull(
        _void_p(storage_address), polygon.c_handle()
    )
    return StoredPolygon.adopt_owned(
        polygon.api,
        storage_address,
        _require_handle(result, "convex hull result"),
    )


@abi_requires_headers("slic3r_geometry.h", "slic3r_utils.h")
def expolygons_to_polylines(
    storage, src: ExPolygonCollection
) -> StoredPolylineCollection:
    """
    Copy every contour and hole boundary into independent open polylines.

    ``src`` is borrowed, remains unchanged and may be released after this call.
    The returned collection and all of its elements are owned by ``storage``.
    Release the collection with ``free_from_storage()`` or a ``with`` block;
    element views obtained from it are borrowed and must not be freed directly.
    """
    storage_address = _require_handle(storage, "storage")
    result = src.api.host.expolygons_to_polylines(
        _void_p(storage_address), src.c_handle()
    )
    return StoredPolylineCollection.adopt_owned(
        src.api,
        storage_address,
        _require_handle(result, "polyline collection result"),
    )


def expolygon_equals(lhs: ExPolygon, rhs: ExPolygon) -> bool:
    if not multipoint_views_equal(lhs.contour().view(), rhs.contour().view()):
        return False
    if lhs.hole_size() != rhs.hole_size():
        return False
    for idx in range(lhs.hole_size()):
        if not multipoint_views_equal(lhs.hole(idx).view(), rhs.hole(idx).view()):
            return False
    return True


__all__ = [
    "MULTIPOINT_CCW",
    "MULTIPOINT_CLOSED",
    "MULTIPOINT_CW",
    "CPoint",
    "HandleView",
    "LineDistancer",
    "MultiPoint",
    "Polygon",
    "MutablePolygon",
    "StoredPolygon",
    "Polyline",
    "MutablePolyline",
    "StoredPolyline",
    "PolygonCollection",
    "StoredPolygonCollection",
    "PolylineCollection",
    "StoredPolylineCollection",
    "ExPolygon",
    "MutableExPolygon",
    "StoredExPolygon",
    "ExPolygonCollection",
    "MutableExPolygonCollection",
    "StoredExPolygonCollection",
    "abs_angle",
    "angle_ccw",
    "as_point",
    "convex_hull",
    "expolygon_equals",
    "expolygons_to_polylines",
    "extend_end",
    "extend_start",
    "make_point",
    "midpoint",
    "multipoint_view_as_const",
    "multipoint_views_equal",
    "norm",
    "norm_square",
    "point_add",
    "point_at",
    "point_sub",
    "point_tuple",
]
