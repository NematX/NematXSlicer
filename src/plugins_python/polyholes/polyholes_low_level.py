#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python port of the Polyholes plugin.

This low-level version intentionally shares the normal Polyholes option keys
with the C++ plugin. The host treats compatible option definitions as
idempotent, so either plugin may be initialized first. The Python plugin adds
one extra option, hole_to_polyhole_angle_start, to exercise plugin-specific
settings inside the shared UI line.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from slic3r_api import (
    CPoint,
    GuiRule,
    PluginCancelled,
    PluginBase,
    RAW_CO_BOOL,
    RAW_CO_FLOAT,
    RAW_CO_FLOAT_OR_PERCENT,
    RAW_CONFIG_OPTION_MODE_ADV_EXP,
    RAW_CONFIG_OPTION_MODE_EXPERT,
    RAW_CONFIG_OPTION_MODE_SUSI,
    RAW_CONTAINER_TYPE_REGION,
    RAW_OPTION_CATEGORY_SLICING,
    RAW_PRESET_TYPE_FFF_PRINT,
    RAW_PT_FFF,
    SCALED_EPSILON,
    STEP_POST_SLICING,
    STEP_SLICING,
    throw_if_cancelled,
    PluginRunContext,
    raw_post_slicing_context,
    report_error,
    report_progress,
    round_coord,
    scale_i,
    unscaled,
    used_config_key,
)


POLYHOLES_KEY = "hole_to_polyhole"
POLYHOLES_THRESHOLD_KEY = "hole_to_polyhole_threshold"
POLYHOLES_TWISTED_KEY = "hole_to_polyhole_twisted"
POLYHOLES_ANGLE_START_KEY = "hole_to_polyhole_angle_start"
POLYHOLES_EXCLUSIVE_GROUP = "polyholes"
POLYHOLES_SELECTOR_KEY = "exclusive_group_300_polyholes_plugin"
POLYHOLES_SETTINGS_FRAGMENT_ID = "polyholes_settings"


@dataclass
class HoleData:
    center_scaled: CPoint
    max_radius_scaled: float
    extruder_id: int
    max_deviation_scaled: int
    twist: bool
    angle_start_rad: float
    points_scaled: list[CPoint]
    layer_region_idx: int


@dataclass
class LayerHole:
    points_scaled: list[CPoint]
    layer_idx: int
    layer_region_idx: int


@dataclass
class ThroughHole:
    hole_data: HoleData
    layers: list[LayerHole]


def _point_distance_scaled(lhs: CPoint, rhs: CPoint) -> float:
    return math.hypot(float(lhs.x - rhs.x), float(lhs.y - rhs.y))


def _point_mid_scaled(lhs: CPoint, rhs: CPoint) -> CPoint:
    return CPoint((lhs.x + rhs.x) // 2, (lhs.y + rhs.y) // 2)


def _points_equal(lhs: list[CPoint], rhs: list[CPoint]) -> bool:
    if len(lhs) != len(rhs):
        return False
    return all(a.x == b.x and a.y == b.y for a, b in zip(lhs, rhs))


def _create_polyholes(
    api,
    storage_address: int,
    center_scaled: CPoint,
    radius_scaled: float,
    nozzle_diameter_scaled: int,
    twist: bool,
    angle_start_rad: float,
    run_ctx_address: int,
) -> list[int]:
    """Build storage-owned polygons; return ownership on success and release every partial allocation on failure or cancellation."""
    throw_if_cancelled(run_ctx_address)
    if nozzle_diameter_scaled <= 0:
        nozzle_diameter_scaled = 1
    radius_mm = unscaled(radius_scaled)
    nozzle_diameter_mm = unscaled(nozzle_diameter_scaled)
    edge_count = max(3, round_coord(4.0 * radius_mm * 0.4 / nozzle_diameter_mm))
    polyhole_count = 5 if twist else 1
    rotation_rad = 2.0 * math.pi / (edge_count * polyhole_count) if twist else 0.0
    new_radius_scaled = radius_scaled / math.cos(math.pi / edge_count)

    polygons: list[int] = []
    complete = False
    try:
        for _ in range(polyhole_count):
            polygon = api.storage_new_polygon(storage_address)
            if not polygon:
                raise MemoryError("Cannot allocate a polyhole polygon")
            polygons.append(polygon)
        for poly_idx in range(polyhole_count):
            throw_if_cancelled(run_ctx_address)
            polygon_idx = poly_idx // 2 if poly_idx % 2 == 0 else (polyhole_count + 1) // 2 + poly_idx // 2
            polygon = polygons[polygon_idx]
            for edge_idx in range(edge_count):
                if edge_idx % 64 == 0:
                    throw_if_cancelled(run_ctx_address)
                angle_rad = angle_start_rad + rotation_rad * poly_idx + 2.0 * math.pi * edge_idx / edge_count
                x_scaled = round_coord(float(center_scaled.x) + new_radius_scaled * math.cos(angle_rad))
                y_scaled = round_coord(float(center_scaled.y) + new_radius_scaled * math.sin(angle_rad))
                api.polygon_push_back(
                    polygon,
                    CPoint(x_scaled, y_scaled),
                )
            api.polygon_make_clockwise(polygon)
        complete = True
        return polygons
    finally:
        if not complete:
            for polygon in polygons:
                api.host.storage_free(storage_address, polygon)


def _replace_matching_hole(
    api,
    expolygon_address: int,
    points_to_replace_scaled: list[CPoint],
    replacement_polygon: int,
) -> bool:
    for hole in api.expolygon_holes(expolygon_address, mutable=True):
        if _points_equal(api.polygon_points(hole), points_to_replace_scaled):
            api.polygon_replace_points(hole, replacement_polygon)
            return True
    return False


class PythonPolyholesPlugin(PluginBase):
    def __init__(self, api):
        """Retain the borrowed application API and copy a group description for registration after this object."""
        super().__init__(
            "python.polyholes",
            STEP_POST_SLICING,
            name="Python polyholes",
            description="Low-level Python version of the polyholes post-slicing plugin.",
            priority=0,
            exclusive_group_priority=10,
            exclusive_group=POLYHOLES_EXCLUSIVE_GROUP,
            used_config_keys=[
                used_config_key(POLYHOLES_KEY, RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT),
                used_config_key(POLYHOLES_THRESHOLD_KEY, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT),
                used_config_key(POLYHOLES_TWISTED_KEY, RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT),
                used_config_key(POLYHOLES_ANGLE_START_KEY, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT),
            ],
            defined_config_keys=[
                POLYHOLES_KEY,
                POLYHOLES_THRESHOLD_KEY,
                POLYHOLES_TWISTED_KEY,
                POLYHOLES_ANGLE_START_KEY,
            ],
        )
        self.declare_exclusive_group(label="Polyholes plugin",
                                     tooltip="Choose which active plugin converts round vertical holes to polyholes.")
        self.api = api

    def initialize(self, storage_address: int) -> None:
        self.api.create_option_def(
            opt_key=POLYHOLES_KEY,
            type=RAW_CO_BOOL,
            container_type=RAW_CONTAINER_TYPE_REGION,
            option_preset_type=RAW_PRESET_TYPE_FFF_PRINT,
            printer_technology=RAW_PT_FFF,
            label="Convert round holes to polyholes",
            full_label="Convert round holes to polyholes",
            category=RAW_OPTION_CATEGORY_SLICING,
            invalidates_step=STEP_SLICING,
            translation_domain="Slic3r",
            tooltip=(
                "Search for almost-circular holes that span more than one layer and convert the geometry to polyholes."
                " Use the nozzle size and the (biggest) diameter to compute the polyhole."
                "\nSee http://hydraraptor.blogspot.com/2011/02/polyholes.html"
            ),
            mode=RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI,
            default_serialized_value="0",
        )
        self.api.create_option_def(
            opt_key=POLYHOLES_THRESHOLD_KEY,
            type=RAW_CO_FLOAT_OR_PERCENT,
            container_type=RAW_CONTAINER_TYPE_REGION,
            option_preset_type=RAW_PRESET_TYPE_FFF_PRINT,
            printer_technology=RAW_PT_FFF,
            label="Roundness margin",
            full_label="Polyhole detection margin",
            category=RAW_OPTION_CATEGORY_SLICING,
            invalidates_step=STEP_SLICING,
            translation_domain="Slic3r",
            tooltip=(
                "Maximum deflection of a point to the estimated radius of the circle."
                "\nAs cylinders are often exported as triangles of varying size, points may not be on the circle circumference."
                " This setting allows you some leeway to broaden the detection."
                "\nIn mm or in % of the radius."
            ),
            sidetext="mm or %",
            has_max_literal=1,
            max_literal_value=10.0,
            max_literal_is_percent=0,
            mode=RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI,
            default_serialized_value="0.01",
        )
        self.api.create_option_def(
            opt_key=POLYHOLES_TWISTED_KEY,
            type=RAW_CO_BOOL,
            container_type=RAW_CONTAINER_TYPE_REGION,
            option_preset_type=RAW_PRESET_TYPE_FFF_PRINT,
            printer_technology=RAW_PT_FFF,
            label="Twisting",
            full_label="Polyhole twist",
            category=RAW_OPTION_CATEGORY_SLICING,
            invalidates_step=STEP_SLICING,
            translation_domain="Slic3r",
            tooltip="Rotate the polyhole every layer.",
            mode=RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI,
            default_serialized_value="1",
        )
        self.api.create_option_def(
            opt_key=POLYHOLES_ANGLE_START_KEY,
            type=RAW_CO_FLOAT,
            container_type=RAW_CONTAINER_TYPE_REGION,
            option_preset_type=RAW_PRESET_TYPE_FFF_PRINT,
            printer_technology=RAW_PT_FFF,
            label="Start angle",
            full_label="Polyhole start angle",
            category=RAW_OPTION_CATEGORY_SLICING,
            invalidates_step=STEP_SLICING,
            translation_domain="Slic3r",
            tooltip="Initial rotation angle applied to Python-generated polyholes.",
            sidetext="deg",
            mode=RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI,
            default_serialized_value="0",
        )

        self.api.add_ui_fragment(
            "print.ui",
            POLYHOLES_EXCLUSIVE_GROUP,
            "page:Slicing\n"
            "group:Modifying slices\n"
            "line:insert$beforeline$Convert round vertical holes to polyholes:Polyholes plugin\n"
            f"setting:{POLYHOLES_SELECTOR_KEY}\n"
            "end_line\n",
            priority=1,
        )
        self.api.add_ui_fragment(
            "print.ui",
            POLYHOLES_SETTINGS_FRAGMENT_ID,
            "page:Slicing\n"
            "group:Modifying slices\n"
            "line:insert$afterline$Vertical Hole shrinking compensation:Convert round vertical holes to polyholes\n"
            f"setting:label$_:{POLYHOLES_KEY}\n"
            f"setting:sidetext_width$5:{POLYHOLES_THRESHOLD_KEY}\n"
            f"setting:{POLYHOLES_TWISTED_KEY}\n"
            "end_line\n",
            priority=0,
        )
        self.api.add_ui_fragment(
            "print.ui",
            "polyholes_angle_start",
            "page:Slicing\n"
            "group:Modifying slices\n"
            "line:Convert round vertical holes to polyholes\n"
            f"setting:insert$beforesetting${POLYHOLES_TWISTED_KEY}:sidetext_width$5:{POLYHOLES_ANGLE_START_KEY}\n"
            "end_line\n",
            priority=1,
        )
        GuiRule.activate([
            POLYHOLES_THRESHOLD_KEY,
            POLYHOLES_TWISTED_KEY,
            POLYHOLES_ANGLE_START_KEY,
        ]).when_true(POLYHOLES_KEY).add_to(self.api)

    def run(self, run_ctx_address: int) -> None:
        throw_if_cancelled(run_ctx_address)
        ctx = raw_post_slicing_context(run_ctx_address)
        common = PluginRunContext.from_address(run_ctx_address) if run_ctx_address else None
        if ctx is None or not ctx.print or not ctx.object or common is None:
            return

        try:
            self._run_polyholes(run_ctx_address, common, ctx)
        except PluginCancelled:
            raise
        except Exception as exc:
            report_error(run_ctx_address, f"Python Polyholes failed: {exc}")

    def _run_polyholes(self, run_ctx_address: int, common: PluginRunContext, ctx) -> None:
        """Replace suitable slice holes and rebuild changed layers; finally release prepared polygons individually, including on failure or cancellation."""
        layer_count = self.api.host.object_count_layer(ctx.object)
        layer_holes: list[list[HoleData]] = [[] for _ in range(layer_count)]

        for layer_idx in range(layer_count):
            throw_if_cancelled(run_ctx_address)
            layer = self.api.host.object_get_layer(ctx.object, layer_idx)
            for region_idx in range(self.api.host.layer_count_region(layer)):
                if region_idx % 64 == 0:
                    throw_if_cancelled(run_ctx_address)
                layer_region = self.api.host.layer_get_region(layer, region_idx)
                print_region = self.api.host.layer_region_get_print_region(layer_region)
                region_config = self.api.host.print_region_get_config(print_region)
                if not self.api.config_bool(region_config, POLYHOLES_KEY):
                    continue

                twist = self.api.config_bool(region_config, POLYHOLES_TWISTED_KEY)
                angle_start_rad = math.radians(self.api.config_float(region_config, POLYHOLES_ANGLE_START_KEY))
                perimeter_extruder = self.api.config_int(region_config, "perimeter_extruder") - 1
                region_slices = self.api.host.layer_region_get_slices(layer_region)
                for expolygon_idx, expolygon in enumerate(self.api.expolygons(region_slices)):
                    if expolygon_idx % 64 == 0:
                        throw_if_cancelled(run_ctx_address)
                    for hole_idx, hole in enumerate(self.api.expolygon_holes(expolygon)):
                        if hole_idx % 64 == 0:
                            throw_if_cancelled(run_ctx_address)
                        points_scaled = self.api.polygon_points(hole)
                        if len(points_scaled) <= 8:
                            continue
                        if self.api.polygon_convex_point_count(hole, 0.0, math.pi) != 0:
                            continue

                        center_scaled = self.api.polygon_centroid(hole)
                        min_radius_scaled = float("inf")
                        max_radius_scaled = 0.0
                        radius_sum_scaled = 0.0
                        for point_idx, point in enumerate(points_scaled):
                            if point_idx % 64 == 0:
                                throw_if_cancelled(run_ctx_address)
                            distance_scaled = _point_distance_scaled(point, center_scaled)
                            min_radius_scaled = min(min_radius_scaled, distance_scaled)
                            max_radius_scaled = max(max_radius_scaled, distance_scaled)
                            radius_sum_scaled += distance_scaled

                        min_line_radius_scaled = float("inf")
                        max_line_radius_scaled = 0.0
                        previous = points_scaled[-1]
                        for point_idx, point in enumerate(points_scaled):
                            if point_idx % 64 == 0:
                                throw_if_cancelled(run_ctx_address)
                            midline_scaled = _point_mid_scaled(previous, point)
                            distance_scaled = _point_distance_scaled(center_scaled, midline_scaled)
                            min_line_radius_scaled = min(min_line_radius_scaled, distance_scaled)
                            max_line_radius_scaled = max(max_line_radius_scaled, distance_scaled)
                            previous = point

                        reference_radius_mm = unscaled(radius_sum_scaled / len(points_scaled))
                        max_variation_scaled = scale_i(self.api.config_float_or_percent_effective(
                            region_config,
                            POLYHOLES_THRESHOLD_KEY,
                            reference_radius_mm,
                        ))
                        max_variation_scaled = max(SCALED_EPSILON, max_variation_scaled)
                        if max_radius_scaled - min_radius_scaled < max_variation_scaled * 2 and max_line_radius_scaled - min_line_radius_scaled < max_variation_scaled * 2:
                            layer_holes[layer_idx].append(HoleData(
                                center_scaled=center_scaled,
                                max_radius_scaled=max_radius_scaled,
                                extruder_id=perimeter_extruder,
                                max_deviation_scaled=max_variation_scaled,
                                twist=twist,
                                angle_start_rad=angle_start_rad,
                                points_scaled=points_scaled,
                                layer_region_idx=region_idx,
                            ))
            report_progress(run_ctx_address, (layer_idx + 1) / max(1, layer_count), "Python Polyholes: searching holes")

        through_holes = self._group_holes(ctx, layer_holes, run_ctx_address)
        print_config = self.api.host.print_get_config(ctx.print)

        prepared_replacements: list[tuple[ThroughHole, list[int]]] = []
        try:
            for through_hole in through_holes:
                throw_if_cancelled(run_ctx_address)
                nozzle_diameter_scaled = scale_i(self.api.config_float(
                    print_config,
                    "nozzle_diameter",
                    max(0, through_hole.hole_data.extruder_id),
                ))
                replacements = _create_polyholes(
                    self.api,
                    common.plugin_storage,
                    through_hole.hole_data.center_scaled,
                    through_hole.hole_data.max_radius_scaled,
                    nozzle_diameter_scaled,
                    through_hole.hole_data.twist,
                    through_hole.hole_data.angle_start_rad,
                    run_ctx_address,
                )
                prepared_replacements.append((through_hole, replacements))

            # Raw slices and their derived layer caches form one consistency commit.
            # Once mutation starts, every touched layer is recomputed before the next
            # cancellation check so callers cannot observe stale slice-island caches.
            throw_if_cancelled(run_ctx_address)
            modified_layers: set[int] = set()
            for hole_idx, (through_hole, replacements) in enumerate(prepared_replacements):
                for layer_hole in through_hole.layers:
                    mutable_layer = self.api.host.object_get_layer_mutable(ctx.object, layer_hole.layer_idx)
                    mutable_region = self.api.host.layer_get_region_mutable(mutable_layer, layer_hole.layer_region_idx)
                    mutable_slices = ctx.layer_region_borrow_mutable_slices(mutable_region)
                    replacement = replacements[layer_hole.layer_idx % len(replacements)]
                    modified = 0
                    for expolygon in self.api.expolygons(mutable_slices, mutable=True):
                        if _replace_matching_hole(
                            self.api, expolygon, layer_hole.points_scaled, replacement
                        ):
                            modified += 1
                    if modified:
                        modified_layers.add(layer_hole.layer_idx)
                report_progress(
                    run_ctx_address,
                    (hole_idx + 1) / max(1, len(through_holes)),
                    "Python Polyholes: converting holes",
                )

            for layer_idx in sorted(modified_layers):
                mutable_layer = self.api.host.object_get_layer_mutable(ctx.object, layer_idx)
                ctx.layer_recompute_slices_and_islands_from_layer_region(mutable_layer)

            throw_if_cancelled(run_ctx_address)
        finally:
            for _, replacements in prepared_replacements:
                for polygon in replacements:
                    self.api.host.storage_free(common.plugin_storage, polygon)

    def _group_holes(
        self, ctx, layer_holes: list[list[HoleData]], run_ctx_address: int
    ) -> list[ThroughHole]:
        throw_if_cancelled(run_ctx_address)
        through_holes: list[ThroughHole] = []
        min_layer_count = 2
        layer_count = len(layer_holes)

        for layer_idx in range(layer_count):
            if layer_idx % 64 == 0:
                throw_if_cancelled(run_ctx_address)
            for hole_idx, main_hole in enumerate(list(layer_holes[layer_idx])):
                if hole_idx % 64 == 0:
                    throw_if_cancelled(run_ctx_address)
                max_z_scaled = self.api.host.layer_get_print_z(self.api.host.object_get_layer(ctx.object, layer_idx))
                holes = [LayerHole(main_hole.points_scaled, layer_idx, main_hole.layer_region_idx)]
                for search_layer_idx in range(layer_idx + 1, layer_count):
                    if search_layer_idx % 64 == 0:
                        throw_if_cancelled(run_ctx_address)
                    search_layer = self.api.host.object_get_layer(ctx.object, search_layer_idx)
                    if self.api.host.layer_get_print_z(search_layer) - self.api.host.layer_get_height(search_layer) - max_z_scaled > 0:
                        break

                    candidates = layer_holes[search_layer_idx]
                    for search_hole_idx, search_hole in enumerate(candidates):
                        if search_hole_idx % 64 == 0:
                            throw_if_cancelled(run_ctx_address)
                        if (
                            main_hole.extruder_id == search_hole.extruder_id
                            and _point_distance_scaled(main_hole.center_scaled, search_hole.center_scaled) < main_hole.max_deviation_scaled
                            and abs(main_hole.max_radius_scaled - search_hole.max_radius_scaled) < main_hole.max_deviation_scaled
                        ):
                            max_z_scaled = self.api.host.layer_get_print_z(search_layer)
                            holes.append(LayerHole(search_hole.points_scaled, search_layer_idx, search_hole.layer_region_idx))
                            del candidates[search_hole_idx]
                            break

                if len(holes) >= min_layer_count or (len(holes) == 1 and holes[0].layer_idx == 0):
                    through_holes.append(ThroughHole(main_hole, holes))

        return through_holes


def register_plugin(api):
    return PythonPolyholesPlugin(api)
