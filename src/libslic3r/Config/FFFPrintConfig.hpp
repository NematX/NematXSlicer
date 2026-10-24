///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_FFFPrintConfig_hpp_
#define slic3r_FFFPrintConfig_hpp_

#include <cstdint>

#include "PrintConfig.hpp"

// FFF-specific static print configuration classes.
//
// This header keeps the common dynamic/config-definition layer in PrintConfig.hpp
// separate from the large FFF static configuration hierarchy. Include it when
// code needs PrintObjectConfig, PrintRegionConfig, PrintConfig, GCodeConfig,
// FullPrintConfig, or FFF-specific enum values.
//
// print_custom_setting preserves unknown text in JSON groups named print and printer.
// filament_custom_setting preserves one JSON string per filament position.
// These stored values do not act as slicing options or custom variables.

namespace Slic3r {


void initialize_fff_print_config_cache();

enum CompleteObjectSort {
    cosNearest,
    cosObject,
    cosZ,
    cosY,
};

enum WipeAlgo {
    waLinear,
    waQuadra,
    waHyper,
};

enum GCodeFlavor : uint8_t {
    gcfRepRap,
    gcfSprinter,
    gcfRepetier,
    gcfTeacup,
    gcfMakerWare,
    gcfMarlinLegacy,
    gcfMarlinFirmware,
    gcfKlipper,
    gcfSailfish,
    gcfMach3,
    gcfMachinekit,
    gcfSmoothie,
    gcfNoExtrusion,
};

enum class MachineLimitsUsage : uint8_t {
    EmitToGCode,
    TimeEstimateOnly,
    Limits,
    Ignore,
    Count,
};

enum class FuzzySkinType {
    None,
    External,
    Shell,
    All,
};

enum InfillPattern : uint8_t{
    ipRectilinear,
    ipMonotonic,
    ipAlignedRectilinear,
    ipGrid,
    ipTriangles, ipStars, ipCubic,
    ipLine,
    ipConcentric,
    ipHoneycomb, ip3DHoneycomb,
    ipHilbertCurve, ipArchimedeanChords, ipOctagramSpiral,
    ipAdaptiveCubic, ipSupportCubic, ipSupportBase,
    ipSmooth,
    ipRectiWithPerimeter,
    ipScatteredRectilinear,
    ipSawtooth,
    ipLightning,
    ipEnsuring,
    ipAuto,
    ipCount,
};

enum class IroningType {
    TopSurfaces,
    TopmostOnly,
    AllSolid,
    Count,
};

enum PerimeterDirection {
   pdCCW_CW,
   pdCCW_CCW,
   pdCW_CCW,
   pdCW_CW,
};

enum SupportMaterialPattern {
    smpRectilinear,
    smpRectilinearGrid,
    smpHoneycomb,
};

enum SupportMaterialStyle {
    smsGrid,
    smsSnug,
    smsTree,
    smsOrganic,
    smsFilled,
};

//from prusa, not used in superslicer as InfillPattern is enough.
//enum SupportMaterialInterfacePattern {
//    smipAuto, smipRectilinear, smipConcentric,
//};

enum SeamPosition {
    spRandom,
    spAllRandom,
    spNearest, //not used anymore
    spAligned,
    spExtremlyAligned,
    spRear,
    spCustom, // or seam object
    spCost,
};

// Orca
enum class SeamScarfType {
    None,
    External,
    All,
};

enum DenseInfillAlgo {
    dfaAutomatic,
    dfaAutoNotFull,
    dfaAutoOrEnlarged,
    dfaAutoOrNothing,
    dfaEnlarged,
    dfaDisabled,
};

enum NoPerimeterUnsupportedAlgo {
    npuaNone, npuaNoPeri, npuaBridges, npuaBridgesOverhangs, npuaFilled,
};

enum InfillConnection {
    icConnected, icHoles, icOuterShell, icNotConnected,
};

enum RemainingTimeType : uint8_t{
    rtNone      = 0,
    rtM117      = 1<<0,
    rtM73       = 1<<1,
    rtM73_Quiet = 1<<2,
    rtM73_M117 = rtM73 | rtM117,
};
//note: check if the enum_bitmask can't be used (and improve it?)
inline RemainingTimeType operator|(RemainingTimeType a, RemainingTimeType b) {
    return static_cast<RemainingTimeType>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
}
inline RemainingTimeType operator&(RemainingTimeType a, RemainingTimeType b) {
    return static_cast<RemainingTimeType>(static_cast<uint64_t>(a) & static_cast<uint64_t>(b));
}
inline RemainingTimeType operator^(RemainingTimeType a, RemainingTimeType b) {
    return static_cast<RemainingTimeType>(static_cast<uint64_t>(a) ^ static_cast<uint64_t>(b));
}
inline RemainingTimeType operator|=(RemainingTimeType& a, RemainingTimeType b) {
    a = a | b; return a;
}
inline RemainingTimeType operator&=(RemainingTimeType& a, RemainingTimeType b) {
    a = a & b; return a;
}

enum SupportZDistanceType {
    zdFilament, zdPlane, zdNone,
};

enum BrimType {
    btNoBrim,
    btOuterOnly,
    btInnerOnly,
    btOuterAndInner,
};

enum DraftShield {
    dsDisabled,
    dsLimited,
    dsEnabled,
};

enum class LabelObjectsStyle {
    Disabled,
    Octoprint,
    Firmware,
    Both,
};

enum class PerimeterGeneratorType
{
    // Classic perimeter generator using Clipper offsets with constant extrusion width.
    Classic,
    // Perimeter generator with variable extrusion width based on the paper
    // "A framework for adaptive width control of dense contour-parallel toolpaths in fused deposition modeling" ported from Cura.
    Arachne
};

enum class GCodeThumbnailsFormat {
    PNG, JPG, QOI, BIQU
};

enum class EnsureVerticalShellThickness {
    Disabled,
    Partial,
    Enabled,
    Enabled_old,
};

#define CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(NAME) \
    template<> const t_config_enum_names& ConfigOptionEnum<NAME>::get_enum_names(); \
    template<> const t_config_enum_values& ConfigOptionEnum<NAME>::get_enum_values();

CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(ArcFittingType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(BridgeType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(BrimType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(CompleteObjectSort)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(DenseInfillAlgo)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(DraftShield)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(EnsureVerticalShellThickness)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(FuzzySkinType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(GCodeFlavor)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(GCodeThumbnailsFormat)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(InfillConnection)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(InfillPattern)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(IroningType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(LabelObjectsStyle)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(MachineLimitsUsage)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(NoPerimeterUnsupportedAlgo)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(PerimeterDirection)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(PerimeterGeneratorType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(RemainingTimeType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(SeamPosition)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(SeamScarfType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(SupportMaterialPattern)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(SupportMaterialStyle)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(SupportZDistanceType)
CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS(WipeAlgo)

#undef CONFIG_OPTION_ENUM_DECLARE_STATIC_MAPS

PRINT_CONFIG_CLASS_DEFINE_WITH_SCOPE(
    PrintObjectConfig,
    StaticPrintConfig::DynamicOptionScope::FFFObject,

    ((ConfigOptionFloat,                brim_width))
    ((ConfigOptionFloat,                brim_width_interior))
    ((ConfigOptionBool,                 brim_per_object))
    //((ConfigOptionEnum<BrimType>,       brim_type))
    ((ConfigOptionString,               object_gcode))
    ((ConfigOptionPercent,              external_perimeter_cut_corners))
    //((ConfigOptionBool,                 exact_last_layer_height))
    ((ConfigOptionFloatOrPercent,       extrusion_width))
    ((ConfigOptionFloatOrPercent,       extrusion_spacing))
    ((ConfigOptionBool,                 fill_angle_follow_model))
    ((ConfigOptionFloatOrPercent,       first_layer_acceleration_over_raft))
    ((ConfigOptionFloatOrPercent,       first_layer_height))
    ((ConfigOptionFloatOrPercent,       first_layer_extrusion_width))
    ((ConfigOptionFloatOrPercent,       first_layer_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       first_layer_infill_extrusion_width))
    ((ConfigOptionFloatOrPercent,       first_layer_infill_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       first_layer_speed_over_raft))
    ((ConfigOptionPercent,              first_layer_strong_start))
    ((ConfigOptionFloat,                hole_size_threshold))
    //((ConfigOptionBool,                 infill_only_where_needed))
    // Force the generation of solid shells between adjacent materials/volumes.
    ((ConfigOptionBool,                 interface_shells))
    ((ConfigOptionFloat,                layer_height))
    ((ConfigOptionFloat,                mmu_segmented_region_max_width))
    ((ConfigOptionFloat,                mmu_segmented_region_interlocking_depth))
    ((ConfigOptionFloat,                model_precision))
    ((ConfigOptionPercent,              perimeter_bonding))
    ((ConfigOptionFloat,                raft_contact_distance))
    ((ConfigOptionEnum<SupportZDistanceType>, raft_contact_distance_type))
    ((ConfigOptionFloat,                raft_expansion))
    ((ConfigOptionPercent,              raft_first_layer_density))
    ((ConfigOptionFloat,                raft_first_layer_expansion))
    ((ConfigOptionFloatOrPercent,       raft_interface_layer_height))
    ((ConfigOptionInt,                  raft_layers))
    ((ConfigOptionFloatOrPercent,       raft_layer_height))
    ((ConfigOptionEnum<SeamPosition>,   seam_position))
//    ((ConfigOptionFloat,                seam_preferred_direction))
//    ((ConfigOptionFloat,                seam_preferred_direction_jitter))
    ((ConfigOptionFloat,                slice_closing_radius))
    ((ConfigOptionEnum<SlicingMode>,    slicing_mode))
    ((ConfigOptionBool,                 staggered_inner_seams))
    ((ConfigOptionBool,                 support_material))
    // Automatic supports (generated based fdm support point generator).
    ((ConfigOptionBool,                 support_material_auto))
    // Direction of the support pattern (in XY plane).
    ((ConfigOptionFloat,                support_material_angle))
    ((ConfigOptionFloat,                support_material_angle_height))
    ((ConfigOptionFloatOrPercent,       support_material_bottom_interface_expansion))
    ((ConfigOptionInt,                  support_material_bottom_interface_layers))
    ((ConfigOptionEnum<InfillPattern>,  support_material_bottom_interface_pattern))
    ((ConfigOptionBool,                 support_material_buildplate_only))
    ((ConfigOptionEnum<SupportZDistanceType>,   support_material_contact_distance_type))
    // support_material_contact_distance (PS) == support_material_contact_distance_top (SuSi 2.3 &-)
    ((ConfigOptionFloatOrPercent,       support_material_contact_distance))
    // support_material_bottom_contact_distance (PS 2.4) == support_material_contact_distance_bottom (SuSi 2.3 &-)
    ((ConfigOptionFloatOrPercent,       support_material_bottom_contact_distance))
    // Morphological closing of support areas. Only used for "sung" supports.
    ((ConfigOptionFloat,                support_material_closing_radius))
    ((ConfigOptionInt,                  support_material_enforce_layers))
    ((ConfigOptionInt,                  support_material_extruder))
    ((ConfigOptionFloatOrPercent,       support_material_extrusion_width))
    ((ConfigOptionFloat,                support_material_interface_angle))
    ((ConfigOptionFloat,                support_material_interface_angle_increment))
    ((ConfigOptionBool,                 support_material_interface_contact_loops))
    ((ConfigOptionInt,                  support_material_interface_extruder))
    ((ConfigOptionInt,                  support_material_interface_layers))
    ((ConfigOptionFloatOrPercent,       support_material_interface_layer_height))
    // Spacing between interface lines (the hatching distance). Set zero to get a solid interface.
    ((ConfigOptionFloat,                support_material_interface_spacing))
    ((ConfigOptionFloatOrPercent,       support_material_interface_speed))
    ((ConfigOptionEnum<SupportMaterialPattern>,  support_material_pattern))
    ((ConfigOptionFloatOrPercent,       support_material_layer_height))
    // Spacing between support material lines (the hatching distance).
    ((ConfigOptionFloat,                support_material_spacing))
    ((ConfigOptionFloatOrPercent,       support_material_speed))
    ((ConfigOptionEnum<SupportMaterialStyle>, support_material_style))
    ((ConfigOptionBool,                 support_material_synchronize_layers))
    // Overhang angle threshold.
    ((ConfigOptionInt,                  support_material_threshold))
    ((ConfigOptionEnum<InfillPattern>,  support_material_top_interface_pattern))
    ((ConfigOptionBool,                 support_material_with_sheath))
    ((ConfigOptionFloatOrPercent,       support_material_xy_spacing))
    // Tree supports
    ((ConfigOptionFloat,                support_tree_angle))
    ((ConfigOptionFloat,                support_tree_angle_slow))
    ((ConfigOptionFloat,                support_tree_branch_diameter))
    ((ConfigOptionFloat,                support_tree_branch_diameter_angle))
    ((ConfigOptionFloat,                support_tree_branch_diameter_double_wall))
    ((ConfigOptionPercent,              support_tree_top_rate))
    ((ConfigOptionFloat,                support_tree_branch_distance))
    ((ConfigOptionFloatOrPercent,       support_tree_max_overhang))
    ((ConfigOptionFloat,                support_tree_tip_diameter))
    // The rest
    ((ConfigOptionBool,                 wipe_into_objects))
    ((ConfigOptionBool,                 wipe_tower))
    ((ConfigOptionFloat,                wipe_tower_bridging))
    ((ConfigOptionFloatOrPercent,       wipe_tower_brim_width))
    ((ConfigOptionFloat,                wipe_tower_cone_angle))
    ((ConfigOptionPercent,              wipe_tower_extra_spacing))
    ((ConfigOptionInt,                  wipe_tower_extruder))
    ((ConfigOptionBool,                 wipe_tower_extruder_toolchange))
    ((ConfigOptionFloatOrPercent,       wipe_tower_extrusion_width))
    ((ConfigOptionFloat,                wipe_tower_object_x))
    ((ConfigOptionFloat,                wipe_tower_object_y))
    ((ConfigOptionBool,                 wipe_tower_per_object))
    ((ConfigOptionBool,                 wipe_tower_rest_in_middle))
    ((ConfigOptionFloat,                wipe_tower_rotation_angle))
    ((ConfigOptionFloat,                wipe_tower_width))
    ((ConfigOptionFloat,                wipe_tower_x))
    ((ConfigOptionFloat,                wipe_tower_y))
)

PRINT_CONFIG_CLASS_DEFINE_WITH_SCOPE(
    PrintRegionConfig,
    StaticPrintConfig::DynamicOptionScope::FFFRegion,

    ((ConfigOptionBool,                 avoid_crossing_perimeters))
    ((ConfigOptionBool,                 avoid_crossing_top))
    ((ConfigOptionBool,                 avoid_travel_island))
    ((ConfigOptionFloat,                avoid_travel_island_weight))
    ((ConfigOptionFloat,                bridge_angle))
    // bridge_fill_pattern is defined by the infill.generator.default plugin.
    ((ConfigOptionEnum<BridgeType>,     bridge_type))
    ((ConfigOptionInt,                  bottom_solid_layers))
    ((ConfigOptionFloat,                bottom_solid_min_thickness))
    ((ConfigOptionPercent,              bridge_flow_ratio))
    ((ConfigOptionPercent,              over_bridge_flow_ratio))
    ((ConfigOptionPercent,              bridge_overlap))
    ((ConfigOptionPercent,              bridge_overlap_min))
    // bottom_fill_pattern is defined by the infill.generator.default plugin.
    ((ConfigOptionFloatOrPercent,       bridged_infill_margin))
    ((ConfigOptionFloat,                curve_smoothing_precision))
    ((ConfigOptionFloat,                curve_smoothing_cutoff_dist))
    ((ConfigOptionFloat,                curve_smoothing_angle_convex))
    ((ConfigOptionFloat,                curve_smoothing_angle_concave))
    ((ConfigOptionFloatOrPercent,       default_speed))
    ((ConfigOptionBool,                 enforce_full_fill_volume))
    ((ConfigOptionEnum<EnsureVerticalShellThickness>, ensure_vertical_shell_thickness))
    ((ConfigOptionFloatOrPercent,       external_infill_margin))
    ((ConfigOptionFloatOrPercent,       external_perimeter_extrusion_width))
    ((ConfigOptionFloatOrPercent,       external_perimeter_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       external_perimeter_extrusion_change_odd_layers))
    ((ConfigOptionPercent,              external_perimeter_overlap))
    ((ConfigOptionBool,                 external_perimeters_first))
    ((ConfigOptionBool,                 external_perimeters_first_force))
    ((ConfigOptionBool,                 external_perimeters_hole))
    ((ConfigOptionBool,                 external_perimeters_nothole))
    ((ConfigOptionBool,                 extra_perimeters))
    ((ConfigOptionBool,                 only_one_perimeter_top))
    ((ConfigOptionBool,                 fill_aligned_z))
    ((ConfigOptionFloat,                fill_angle))
    ((ConfigOptionBool,                 fill_angle_cross))
    ((ConfigOptionFloat,                fill_angle_increment))
    ((ConfigOptionFloats,               fill_angle_template))
    ((ConfigOptionPercent,              fill_density))
    // fill_pattern is defined by the infill.generator.default plugin.
    ((ConfigOptionEnum<FuzzySkinType>,  fuzzy_skin))
    ((ConfigOptionFloatOrPercent,       fuzzy_skin_thickness))
    ((ConfigOptionFloatOrPercent,       fuzzy_skin_point_dist))
    ((ConfigOptionPercent,              fill_top_flow_ratio))
    ((ConfigOptionPercent,              fill_smooth_distribution))
    ((ConfigOptionFloatOrPercent,       fill_smooth_width))
    ((ConfigOptionPercent,              gap_fill_flow_match_perimeter))
    ((ConfigOptionBool,                 gap_fill_perimeter))
    ((ConfigOptionFloatOrPercent,       infill_anchor))
    ((ConfigOptionFloatOrPercent,       infill_anchor_max))
    ((ConfigOptionInt,                  infill_extruder))
    ((ConfigOptionFloatOrPercent,       infill_extrusion_width))
    ((ConfigOptionFloatOrPercent,       infill_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       infill_extrusion_change_odd_layers))
    ((ConfigOptionInt,                  infill_every_layers))
    ((ConfigOptionFloatOrPercent,       infill_overlap))
    ((ConfigOptionEnum<InfillConnection>,  infill_connection))
    ((ConfigOptionEnum<InfillConnection>,  infill_connection_solid))
    ((ConfigOptionEnum<InfillConnection>,  infill_connection_top))
    ((ConfigOptionEnum<InfillConnection>,  infill_connection_bottom))
    ((ConfigOptionEnum<InfillConnection>,  infill_connection_bridge))
    ((ConfigOptionBool,                 infill_dense))
    ((ConfigOptionEnum<DenseInfillAlgo>,  infill_dense_algo))
    ((ConfigOptionBool,                 infill_first))
    ((ConfigOptionBool,                 infill_filled_bottom))
    ((ConfigOptionBool,                 infill_filled_solid))
    ((ConfigOptionBool,                 infill_filled_top))
    ((ConfigOptionBool,                 internal_bridge_expansion))
    ((ConfigOptionFloatOrPercent,       internal_bridge_min_width))
    // Ironing options
    ((ConfigOptionBool,                 ironing))
    ((ConfigOptionFloat,                ironing_angle))
    ((ConfigOptionEnum<IroningType>,    ironing_type))
    ((ConfigOptionPercent,              ironing_flowrate))
    ((ConfigOptionFloatOrPercent,       ironing_spacing))
    // milling options
    ((ConfigOptionFloatOrPercent,       milling_after_z))
    ((ConfigOptionFloatOrPercent,       milling_extra_size))
    ((ConfigOptionBool,                 milling_post_process))
    ((ConfigOptionFloatOrPercent,       min_bead_width))
    ((ConfigOptionFloatOrPercent,       min_feature_size))
    // Detect bridging perimeters
    ((ConfigOptionBool,                 overhangs))
    ((ConfigOptionGraph,                overhangs_dynamic_flow))
    ((ConfigOptionGraph,                overhangs_dynamic_speed))
    ((ConfigOptionFloatOrPercent,       overhangs_extrusion_spacing))
    ((ConfigOptionPercent,              overhangs_flow_ratio))
    ((ConfigOptionBool,                 overhangs_reverse))
    ((ConfigOptionFloatOrPercent,       overhangs_reverse_threshold))
    ((ConfigOptionFloatOrPercent,       overhangs_speed))
    ((ConfigOptionInt,                  overhangs_speed_enforce))
    ((ConfigOptionEnum<BridgeType>,     overhangs_type))
    ((ConfigOptionFloatOrPercent,       overhangs_width))
    ((ConfigOptionFloatOrPercent,       overhangs_width_speed))
    ((ConfigOptionEnum<NoPerimeterUnsupportedAlgo>,  no_perimeter_unsupported_algo))
    ((ConfigOptionFloatOrPercent,       perimeter_acceleration))
    ((ConfigOptionEnum<PerimeterDirection>, perimeter_direction))
    ((ConfigOptionInt,                  perimeter_extruder))
    ((ConfigOptionFloatOrPercent,       perimeter_extrusion_width))
    ((ConfigOptionFloatOrPercent,       perimeter_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       perimeter_extrusion_change_odd_layers))
    ((ConfigOptionEnum<PerimeterGeneratorType>, perimeter_generator))
    ((ConfigOptionBool,                 perimeter_loop))
    ((ConfigOptionEnum<SeamPosition>,   perimeter_loop_seam))
    ((ConfigOptionPercent,              perimeter_overlap))
    // Total number of perimeters.
    ((ConfigOptionInt,                  perimeters))
    ((ConfigOptionPercent,              print_extrusion_multiplier))
    ((ConfigOptionString,               region_gcode))
    ((ConfigOptionFloatOrPercent,       slice_merge_dent))
    ((ConfigOptionFloatOrPercent,       slice_merge_min_width))
    ((ConfigOptionFloatOrPercent,       seam_notch_all))
    ((ConfigOptionFloat,                seam_notch_angle))
    ((ConfigOptionFloatOrPercent,       seam_notch_inner))
    ((ConfigOptionFloatOrPercent,       seam_notch_outer))
    ((ConfigOptionEnum<SeamScarfType>,  seam_slope_type))
    ((ConfigOptionFloatOrPercent,       seam_slope_min_height))
    ((ConfigOptionFloatOrPercent,       seam_slope_max_length))
    ((ConfigOptionGraph,                small_area_infill_flow_compensation_model))
    ((ConfigOptionFloatOrPercent,       small_perimeter_speed))
    ((ConfigOptionFloatOrPercent,       small_perimeter_min_length))
    ((ConfigOptionFloatOrPercent,       small_perimeter_max_length))
    // solid_fill_pattern is defined by the infill.generator.default plugin.
    ((ConfigOptionFloat,                solid_infill_below_area))
    ((ConfigOptionFloat,                solid_infill_below_layer_area))
    ((ConfigOptionFloatOrPercent,       solid_infill_below_width))
    ((ConfigOptionInt,                  solid_infill_extruder))
    ((ConfigOptionFloatOrPercent,       solid_infill_extrusion_width))
    ((ConfigOptionFloatOrPercent,       solid_infill_extrusion_spacing))
    ((ConfigOptionFloatOrPercent,       solid_infill_extrusion_change_odd_layers))
    ((ConfigOptionInt,                  solid_infill_every_layers))
    ((ConfigOptionPercent,              solid_infill_overlap))
    ((ConfigOptionInt,                  solid_over_perimeters))
    ((ConfigOptionInt,                  print_first_layer_temperature))
    ((ConfigOptionInt,                  print_temperature))
    // top_fill_pattern is defined by the infill.generator.default plugin.
    ((ConfigOptionFloatOrPercent,       top_infill_extrusion_width))
    ((ConfigOptionFloatOrPercent,       top_infill_extrusion_spacing))
    ((ConfigOptionInt,                  top_solid_layers))
    ((ConfigOptionFloat,                top_solid_min_thickness))
    ((ConfigOptionPercent,              top_solid_infill_overlap))
    ((ConfigOptionInt,                  wall_distribution_count))
    ((ConfigOptionFloatOrPercent,       wall_transition_length))
    ((ConfigOptionFloatOrPercent,       wall_transition_filter_deviation))
    ((ConfigOptionFloat,                wall_transition_angle))
    ((ConfigOptionBool,                 wipe_into_infill))
)

PRINT_CONFIG_CLASS_DEFINE(
    MachineEnvelopeConfig,

    ((ConfigOptionEnum<MachineLimitsUsage>, machine_limits_usage))
    ((ConfigOptionFloats,              machine_max_acceleration_x))
    ((ConfigOptionFloats,              machine_max_acceleration_y))
    ((ConfigOptionFloats,              machine_max_acceleration_z))
    ((ConfigOptionFloats,              machine_max_acceleration_e))
    ((ConfigOptionFloats,              machine_max_feedrate_x))
    ((ConfigOptionFloats,              machine_max_feedrate_y))
    ((ConfigOptionFloats,              machine_max_feedrate_z))
    ((ConfigOptionFloats,              machine_max_feedrate_e))
    ((ConfigOptionFloats,              machine_max_acceleration_extruding))
    ((ConfigOptionFloats,              machine_max_acceleration_retracting))
    ((ConfigOptionFloats,              machine_max_acceleration_travel))
    ((ConfigOptionFloats,              machine_max_jerk_x))
    ((ConfigOptionFloats,              machine_max_jerk_y))
    ((ConfigOptionFloats,              machine_max_jerk_z))
    ((ConfigOptionFloats,              machine_max_jerk_e))
    ((ConfigOptionFloats,              machine_min_travel_rate))
    ((ConfigOptionFloats,              machine_min_extruding_rate))
)

PRINT_CONFIG_CLASS_DEFINE(
    GCodeConfig,

    ((ConfigOptionEnum<ArcFittingType>, arc_fitting))
    ((ConfigOptionBool,                arc_fitting_ignore_holes))
    ((ConfigOptionFloatOrPercent,      arc_fitting_resolution))
    ((ConfigOptionFloatOrPercent,      arc_fitting_tolerance))
    ((ConfigOptionBool,                autoemit_temperature_commands))
    ((ConfigOptionString,              before_layer_gcode))
    ((ConfigOptionBool,                binary_gcode))
    ((ConfigOptionFloat,               cooling_tube_retraction))
    ((ConfigOptionFloat,               cooling_tube_length))
    ((ConfigOptionFloats,              deretract_speed))
    ((ConfigOptionFloat,               extra_loading_move))
    ((ConfigOptionGraphs,              extruder_clearance))
    ((ConfigOptionGraphs,              extruder_extrusion_multiplier_speed))
    ((ConfigOptionPercents,            extruder_fan_offset))
    ((ConfigOptionPoints,              extruder_offset))
    ((ConfigOptionFloats,              extruder_temperature_offset))
    ((ConfigOptionString,              extrusion_axis))
    ((ConfigOptionFloats,              extrusion_multiplier))
    ((ConfigOptionFloat,               fan_kickstart))
    ((ConfigOptionBool,                fan_percentage))
    ((ConfigOptionBool,                fan_speedup_overhangs))
    ((ConfigOptionFloat,               fan_speedup_time))
    ((ConfigOptionFloatsOrPercents,    filament_bridge_pa))
    ((ConfigOptionFloatsOrPercents,    filament_bridge_internal_pa))
    ((ConfigOptionFloatsOrPercents,    filament_brim_pa))
    ((ConfigOptionFloats,              filament_cooling_final_speed))
    ((ConfigOptionFloats,              filament_cooling_initial_speed))
    ((ConfigOptionInts,                filament_cooling_moves))
    ((ConfigOptionFloats,              filament_cost))
    ((ConfigOptionFloats,              filament_density))
    ((ConfigOptionFloats,              filament_diameter))
    ((ConfigOptionFloatsOrPercents,    filament_external_perimeter_pa))
    ((ConfigOptionPercents,            filament_fill_top_flow_ratio))
    ((ConfigOptionPercents,            filament_first_layer_flow_ratio))
    ((ConfigOptionFloatsOrPercents,    filament_first_layer_pa))
    ((ConfigOptionFloatsOrPercents,    filament_first_layer_pa_over_raft))
    ((ConfigOptionFloatsOrPercents,    filament_gap_fill_pa))
    ((ConfigOptionFloatsOrPercents,    filament_infill_pa))
    ((ConfigOptionFloatsOrPercents,    filament_ironing_pa))
    ((ConfigOptionFloats,              filament_load_time))
    ((ConfigOptionFloats,              filament_loading_speed))
    ((ConfigOptionFloats,              filament_loading_speed_start))
    ((ConfigOptionFloats,              filament_max_wipe_tower_speed))
    ((ConfigOptionFloats,              filament_minimal_purge_on_wipe_tower))
    ((ConfigOptionBools,               filament_multitool_ramming))
    ((ConfigOptionFloats,              filament_multitool_ramming_flow))
    ((ConfigOptionFloats,              filament_multitool_ramming_volume))
    ((ConfigOptionFloatsOrPercents,    filament_overhangs_pa))
    ((ConfigOptionFloatsOrPercents,    filament_perimeter_pa))
    ((ConfigOptionStrings,             filament_ramming_parameters))
    ((ConfigOptionFloats,              filament_spool_weight))
    ((ConfigOptionBools,               filament_use_skinnydip))     /* SKINNYDIP OPTIONS BEGIN */
    ((ConfigOptionBools,               filament_use_fast_skinnydip))
    ((ConfigOptionFloats,              filament_skinnydip_distance))
    ((ConfigOptionInts,                filament_melt_zone_pause))
    ((ConfigOptionInts,                filament_cooling_zone_pause))
    ((ConfigOptionBools,               filament_enable_toolchange_temp))
    ((ConfigOptionInts,                filament_toolchange_temp))
    ((ConfigOptionBools,               filament_enable_toolchange_part_fan))
    ((ConfigOptionInts,                filament_toolchange_part_fan_speed))
    ((ConfigOptionFloats,              filament_dip_insertion_speed))
    ((ConfigOptionFloats,              filament_dip_extraction_speed)) /* SKINNYDIP OPTIONS END */
    ((ConfigOptionFloats,              filament_pressure_advance))
    ((ConfigOptionFloatsOrPercents,    filament_solid_infill_pa))
    ((ConfigOptionBools,               filament_soluble))
    ((ConfigOptionFloatsOrPercents,    filament_support_material_pa))
    ((ConfigOptionFloatsOrPercents,    filament_support_material_interface_pa))
    ((ConfigOptionFloatsOrPercents,    filament_thin_walls_pa))
    ((ConfigOptionFloats,              filament_toolchange_delay))
    ((ConfigOptionFloatsOrPercents,    filament_top_solid_infill_pa))
    ((ConfigOptionStrings,             filament_type))
    ((ConfigOptionFloatsOrPercents,    filament_travel_pa))
    ((ConfigOptionFloats,              filament_unloading_speed))
    ((ConfigOptionFloats,              filament_unloading_speed_start))
    ((ConfigOptionFloats,              filament_unload_time))
    ((ConfigOptionFloats,              filament_wipe_advanced_pigment))
    ((ConfigOptionBool,                gcode_ascii))
    ((ConfigOptionInt,                 gcode_command_buffer))
    ((ConfigOptionBool,                gcode_comments))
    ((ConfigOptionString,              gcode_filename_illegal_char))
    ((ConfigOptionEnum<GCodeFlavor>,   gcode_flavor))
    ((ConfigOptionEnum<LabelObjectsStyle>,  gcode_label_objects))
    ((ConfigOptionFloatOrPercent,      gcode_min_resolution))
    ((ConfigOptionInt,                 gcode_precision_xyz))
    ((ConfigOptionInt,                 gcode_precision_e))
    // Triples of strings: "search pattern", "replace with pattern", "attribs"
    // where "attribs" are one of:
    //      r - regular expression
    //      i - case insensitive
    //      w - whole word
    ((ConfigOptionStrings,             gcode_substitutions))
    ((ConfigOptionBool,                high_current_on_filament_swap))
    ((ConfigOptionString,              layer_gcode))
    ((ConfigOptionFloat,               max_gcode_per_second))
    ((ConfigOptionFloat,               max_volumetric_extrusion_rate_slope_positive))
    ((ConfigOptionFloat,               max_volumetric_extrusion_rate_slope_negative))
    ((ConfigOptionFloats,              milling_z_lift))
    ((ConfigOptionFloat,               parking_pos_retraction))
    ((ConfigOptionInt,                 print_bed_temperature))
    ((ConfigOptionInt,                 print_first_layer_bed_temperature))
    ((ConfigOptionBool,                remaining_times))
    ((ConfigOptionEnum<RemainingTimeType>, remaining_times_type))
    ((ConfigOptionPercents,            retract_before_wipe))
    ((ConfigOptionFloats,              retract_length))
    ((ConfigOptionFloats,              retract_length_toolchange))
    ((ConfigOptionFloats,              retract_lift))
    ((ConfigOptionFloats,              retract_lift_above))
    ((ConfigOptionFloats,              retract_lift_below))
    ((ConfigOptionBools,               retract_lift_first_layer))
    ((ConfigOptionStrings,             retract_lift_top))
    ((ConfigOptionFloats,              retract_lift_before_travel))
    ((ConfigOptionFloats,              retract_restart_extra))
    ((ConfigOptionFloats,              retract_restart_extra_toolchange))
    ((ConfigOptionBools,               retract_restart_toolchange_on_perimeter))
    ((ConfigOptionPercents,            retract_restart_wipe_toolchange))
    ((ConfigOptionFloats,              retract_speed))
    ((ConfigOptionBool,                silent_mode))
    ((ConfigOptionBool,                start_gcode_manual))
    ((ConfigOptionBool,                single_extruder_multi_material))
    ((ConfigOptionBool,                single_extruder_multi_material_priming))
    ((ConfigOptionBools,               travel_ramping_lift))
    // ((ConfigOptionFloats,              travel_max_lift))
    ((ConfigOptionFloats,              travel_slope))
    ((ConfigOptionBools,               travel_lift_before_obstacle))
    ((ConfigOptionFloats,              temperature_heat_speed))
    ((ConfigOptionStrings,             tool_name))
    ((ConfigOptionFloat,               travel_speed))
    ((ConfigOptionFloat,               travel_speed_z))
    ((ConfigOptionBool,                use_firmware_retraction))
    ((ConfigOptionBool,                use_relative_e_distances))
    ((ConfigOptionBool,                use_volumetric_e))
    ((ConfigOptionBool,                variable_layer_height))
    ((ConfigOptionBool,                wipe_advanced))
    ((ConfigOptionEnum<WipeAlgo>,      wipe_advanced_algo))
    ((ConfigOptionFloat,               wipe_advanced_nozzle_melted_volume))
    ((ConfigOptionFloat,               wipe_advanced_multiplier))
    ((ConfigOptionFloats,              wipe_extra_perimeter))
    ((ConfigOptionPercents,            wipe_inside_depth))
    ((ConfigOptionBools,               wipe_inside_end))
    ((ConfigOptionBools,               wipe_inside_start))
    ((ConfigOptionFloatsOrPercents,    wipe_lift))
    ((ConfigOptionFloatsOrPercents,    wipe_lift_length))
    ((ConfigOptionFloatsOrPercents,    wipe_min))
    ((ConfigOptionBools,               wipe_only_crossing))
    ((ConfigOptionBools,               wipe_return))
    ((ConfigOptionFloats,              wipe_speed))
    ((ConfigOptionBool,                wipe_tower_no_sparse_layers))
    ((ConfigOptionFloat,               wipe_tower_speed))
    ((ConfigOptionFloatOrPercent,      wipe_tower_wipe_starting_speed))
    ((ConfigOptionFloat,               z_offset))
    ((ConfigOptionFloat,               z_step))

)
#ifdef HAS_PRESSURE_EQUALIZER
    ((ConfigOptionFloat, max_volumetric_extrusion_rate_slope_positive))
    ((ConfigOptionFloat, max_volumetric_extrusion_rate_slope_negative))
#endif

static inline std::string get_extrusion_axis(const GCodeConfig& cfg)
{
    return
        ((cfg.gcode_flavor.value == gcfMach3) || (cfg.gcode_flavor.value == gcfMachinekit)) ? "A" :
        (cfg.gcode_flavor.value == gcfNoExtrusion) ? "" : cfg.extrusion_axis.value;
}

PRINT_CONFIG_CLASS_DERIVED_DEFINE_WITH_SCOPE(
    PrintConfig,
    (MachineEnvelopeConfig, GCodeConfig),
    StaticPrintConfig::DynamicOptionScope::FFFPrint,

    ((ConfigOptionBool,                 allow_empty_layers))
    ((ConfigOptionBool,                 avoid_crossing_curled_overhangs))
    ((ConfigOptionBool,                 avoid_crossing_not_first_layer))
    ((ConfigOptionFloatOrPercent,       avoid_crossing_perimeters_max_detour))
    ((ConfigOptionPoints,               bed_shape))
    ((ConfigOptionInts,                 bed_temperature))
    ((ConfigOptionFloatOrPercent,       bridge_precision))
    ((ConfigOptionInts,                 chamber_temperature))
    ((ConfigOptionBool,                 complete_objects))
    ((ConfigOptionBool,                 complete_objects_one_brim))
    ((ConfigOptionEnum<CompleteObjectSort>, complete_objects_sort))
    ((ConfigOptionFloats,               colorprint_heights))
    //((ConfigOptionBools,                cooling))
    ((ConfigOptionEnum<DraftShield>,    draft_shield))
    ((ConfigOptionFloat,                duplicate_distance))
    ((ConfigOptionBool,                 enforce_retract_first_layer))
    ((ConfigOptionFloat,                extruder_clearance_height))
    ((ConfigOptionFloat,                extruder_clearance_radius))
    ((ConfigOptionStrings,              extruder_colour))
    //((ConfigOptionBools,                fan_always_on))
    ((ConfigOptionStrings,              filament_colour))
    ((ConfigOptionStrings,              filament_custom_setting))
    ((ConfigOptionStrings,              filament_custom_variables))
    ((ConfigOptionStrings,              filament_notes))
    ((ConfigOptionPercents,             filament_max_overlap))
    ((ConfigOptionPercents,             filament_shrink))
    ((ConfigOptionInts,                 first_layer_bed_temperature))
    ((ConfigOptionInts,                 first_layer_temperature))
    ((ConfigOptionInts,                 idle_temperature))
    ((ConfigOptionFloat,                lift_min))
    ((ConfigOptionFloatsOrPercents,     max_layer_height))
    ((ConfigOptionFloat,                max_print_height))
    ((ConfigOptionPercents,             max_speed_reduction))
    ((ConfigOptionFloats,               milling_diameter))
    ((ConfigOptionStrings,              milling_toolchange_end_gcode))
    ((ConfigOptionStrings,              milling_toolchange_start_gcode))
    //((ConfigOptionInts,                 min_fan_speed)) // now fan_printer_min_speed
    ((ConfigOptionFloatsOrPercents,     min_layer_height))
    ((ConfigOptionFloats,               min_print_speed))
    ((ConfigOptionFloat,                min_skirt_length))
    ((ConfigOptionString,               notes))
    ((ConfigOptionFloats,               nozzle_diameter))
    ((ConfigOptionBool,                 only_retract_when_crossing_perimeters))
    ((ConfigOptionBool,                 ooze_prevention))
    ((ConfigOptionString,               output_filename_format))
    ((ConfigOptionGraphs,               overhangs_dynamic_fan_speed))
    ((ConfigOptionBool,                 parallel_islands))
    ((ConfigOptionFloat,                parallel_objects_step))
    ((ConfigOptionFloat,                parallel_objects_step_max_z))
    ((ConfigOptionStrings,              post_process))
    ((ConfigOptionPoint,                priming_position))
    ((ConfigOptionString,               print_custom_setting))
    ((ConfigOptionString,               print_custom_variables))
    ((ConfigOptionString,               printer_custom_variables))
    ((ConfigOptionString,               printer_model))
    ((ConfigOptionString,               printer_notes))
    ((ConfigOptionFloat,                resolution))
    ((ConfigOptionFloat,                resolution_internal))
    ((ConfigOptionFloats,               retract_before_travel))
    ((ConfigOptionBools,                retract_layer_change))
    ((ConfigOptionInt,                  skirt_brim))
    ((ConfigOptionFloat,                skirt_distance))
    ((ConfigOptionBool,                 skirt_distance_from_brim))
    ((ConfigOptionInt,                  skirt_height))
    ((ConfigOptionFloatOrPercent,       skirt_extrusion_width))
    ((ConfigOptionFloatsOrPercents,     seam_gap))
    ((ConfigOptionFloatsOrPercents,     seam_gap_external))
    ((ConfigOptionInt,                  skirts))
    ((ConfigOptionBool,                 spiral_vase))
    ((ConfigOptionInt,                  standby_temperature_delta))
    ((ConfigOptionFloatOrPercent,       support_material_acceleration))
    ((ConfigOptionFloatOrPercent,       support_material_interface_acceleration))
    ((ConfigOptionInts,                 temperature))
    ((ConfigOptionInt,                  threads))
    ((ConfigOptionPoints,               thumbnails))
    ((ConfigOptionString,               thumbnails_color))
    ((ConfigOptionEnum<ThumbnailColorMode>, thumbnail_color_mode))
    ((ConfigOptionBool,                 thumbnails_end_file))
    ((ConfigOptionBool,                 thumbnails_tag_format))
    ((ConfigOptionBool,                 thumbnails_with_bed))
    ((ConfigOptionPercent,              time_estimation_compensation))
    ((ConfigOptionFloat,                time_cost))
    ((ConfigOptionFloat,                time_start_gcode))
    ((ConfigOptionFloat,                time_toolchange))
    ((ConfigOptionBools,                wipe))
    ((ConfigOptionFloats,               wiping_volumes_matrix))
    ((ConfigOptionFloats,               wiping_volumes_extruders))
    ((ConfigOptionFloat,                init_z_rotate))

)

//static inline
double min_object_distance(const PrintConfig& config); //TODO: remove
//static inline
double min_object_distance(const ConfigBase* config, double height = 0); //TODO: remove

// This object is mapped to Perl as Slic3r::Config::Full.
PRINT_CONFIG_CLASS_DERIVED_DEFINE0_WITH_SCOPE(
    FullPrintConfig,
    (PrintObjectConfig, PrintRegionConfig, PrintConfig),
    StaticPrintConfig::DynamicOptionScope::FFFAggregate
)
// Validate the FullPrintConfig. Returns an empty string on success, otherwise an error message is returned.
std::string validate(const FullPrintConfig &config);

// This object is mapped to Perl as Slic3r::Config::PrintRegion.

// Validate the FullPrintConfig. Returns an empty string on success, otherwise an error message is returned.
std::string validate(const FullPrintConfig &config);

bool is_XL_printer(const PrintConfig &cfg);
Points get_bed_shape(const PrintConfig &cfg);

// Register built-in FFF option definitions and defaults.
void init_fff_params(PrintConfigDef &definition);

} // namespace Slic3r

#endif // slic3r_FFFPrintConfig_hpp_
