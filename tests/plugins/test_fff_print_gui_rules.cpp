///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/GuiRulesCore.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

/*
FFF print GUI rules tests
=========================

The core rule registry describes which configuration controls are meaningful.
These tests compare it with an independent imperative model on default settings,
curated tower combinations and seeded fuzz. Towers distinguish global XY from
shared object offsets; no GUI implementation is loaded or tested here.
*/

using namespace Slic3r;

namespace {

using EnabledMap = std::map<std::string, bool>;

// Owner of the built-in activation rules: the gui_rules.core plugin.
const char *const core_gui_rules_owner = "gui_rules.core";
// Owner of the legacy infill selectors moved out of the core rule plugin.
const char *const infill_gui_rules_owner = "infill.generator.default";

// Compute the independent activation map from borrowed config without retaining it.
EnabledMap reference_enabled(const DynamicPrintConfig &config);
// Randomize relevant owned configuration values with the supplied generator.
void randomize(DynamicPrintConfig &config, std::mt19937 &rng);

// Read borrowed settings into an owned activation map independent of the registry, leaving parallel scheduling irrelevant to towers; absent targets are skipped and option/allocation failures propagate without ownership changes.
EnabledMap reference_enabled(const DynamicPrintConfig &config)
{
    EnabledMap out;
    // Dynamic plugin options such as the seam cost settings are not part of the
    // static full print config, so the reference skips absent targets exactly
    // like the rule evaluator does.
    const auto set = [&out, &config](const char *key, bool enabled) {
        if (config.has(key))
            out[key] = enabled;
    };
    const auto b = [&config](const char *key) { return config.opt_bool(key); };
    const auto i = [&config](const char *key) { return config.opt_int(key); };
    const auto fl = [&config](const char *key) { return config.opt_float(key); };
    const auto en = [&config](const char *key) { return config.is_enabled(key); };
    const auto enum_name = [&config](const char *key) {
        std::string value = config.option(key)->serialize();
        if (!value.empty() && value.front() == '!')
            value.erase(value.begin());
        return value;
    };

    const bool P = i("perimeters") > 0;
    const bool A = P &&
        config.opt_enum<PerimeterGeneratorType>("perimeter_generator") == PerimeterGeneratorType::Arachne;
    const bool not_A = !A;
    const bool ext = b("external_perimeters_first");
    const bool loop = b("perimeter_loop");
    const bool loop_seam = loop && !A;
    const bool H = fl("fill_density") > 0;
    const bool T = i("top_solid_layers") > 0;
    const bool B = i("bottom_solid_layers") > 0;
    const bool S = i("solid_infill_every_layers") == 1;
    const bool D = i("solid_infill_every_layers") > 0;
    const bool E = fl("solid_infill_below_area") > 0;
    const bool spiral = b("spiral_vase");
    const bool has_spiral = P && spiral;
    const bool has_top = T || has_spiral || S;
    const bool has_bottom = B || S;
    const bool has_solid = has_top || has_bottom || (H && (D || E));
    const bool overhangs = P && b("overhangs");
    const bool have_skirt = i("skirts") > 0;
    const bool have_brim = fl("brim_width") > 0 || fl("brim_width_interior") > 0;
    const bool supp = b("support_material");
    const bool raft = i("raft_layers") > 0;
    const bool have_support = supp || raft;
    const bool have_iface = have_support &&
        (i("support_material_interface_layers") > 0 || i("support_material_bottom_interface_layers") > 0);
    const SupportMaterialStyle style = config.opt_enum<SupportMaterialStyle>("support_material_style");
    const bool organic = style == SupportMaterialStyle::smsOrganic &&
        (supp || i("support_material_enforce_layers") > 0);
    const bool tree_family = (style == SupportMaterialStyle::smsTree || style == SupportMaterialStyle::smsOrganic) &&
        (supp || i("support_material_enforce_layers") > 0);
    const bool default_accel = fl("default_acceleration") > 0;
    const bool pp_ironing = has_top && b("ironing");
    const bool ext_not_loop = ext && !loop_seam;
    const bool can_reverse = !A && overhangs && !b("perimeter_reverse");
    const bool soluble = have_support &&
        config.opt_enum<SupportZDistanceType>("support_material_contact_distance_type") == zdNone;
    const bool seam_cost = P && config.opt_enum<SeamPosition>("seam_position") == spCost;
    const bool notch = P && (fl("seam_notch_all") != 0 || fl("seam_notch_inner") != 0 || fl("seam_notch_outer") != 0);


    const bool seq = config.opt_serialize("print_plan_builder") == "ordering.plan_builder.by_object";
    const bool whole = seq && fl("parallel_objects_step") == 0.;
    const bool parallel_ok = seq && fl("parallel_objects_step") != 0.;
    const bool own_tower = b("wipe_tower_per_object");
    const bool have_wipe = !spiral;
    const bool wipe_ready = have_wipe && (b("wipe_tower") || own_tower);
    const bool avoid = b("avoid_crossing_perimeters");
    const bool infill_or_solid = H || has_solid;
    const bool smooth =
        (has_top && enum_name("top_fill_pattern") == "smooth") ||
        (has_bottom && enum_name("bottom_fill_pattern") == "smooth") ||
        (has_solid && enum_name("solid_fill_pattern") == "smooth") ||
        (have_support && enum_name("support_material_top_interface_pattern") == "smooth") ||
        (have_support && enum_name("support_material_bottom_interface_pattern") == "smooth");
    const bool default_width =
        (en("first_layer_extrusion_width") && fl("first_layer_extrusion_width") == 0) ||
        (en("first_layer_infill_extrusion_width") && fl("first_layer_infill_extrusion_width") == 0) ||
        (fl("perimeter_extrusion_width") == 0 && (P || have_brim)) ||
        (fl("external_perimeter_extrusion_width") == 0 && P) ||
        (fl("infill_extrusion_width") == 0 && infill_or_solid) ||
        (fl("solid_infill_extrusion_width") == 0 && has_solid) ||
        (fl("top_infill_extrusion_width") == 0 && has_top) ||
        (fl("support_material_extrusion_width") == 0 && have_support) ||
        (fl("skirt_extrusion_width") == 0 && have_skirt);

    const char *const have_perimeters_keys[] = {
        "extra_perimeters",
        "external_perimeters_first",
        "external_perimeter_extrusion_width", "external_perimeter_extrusion_spacing",
        "external_perimeter_extrusion_change_odd_layers", "overhangs", "seam_position",
        "staggered_inner_seams", "perimeter_generator",
        "small_perimeter_speed", "spiral_vase",
        "seam_notch_all", "seam_notch_inner", "seam_notch_outer",
        "no_perimeter_unsupported_algo", "only_one_perimeter_top"
    };
    for (const char *key : have_perimeters_keys)
        set(key, P);

    set("overhangs_dynamic_speed", overhangs);
    set("overhangs_dynamic_flow", overhangs && en("overhangs_flow_ratio"));
    set("small_perimeter_min_length", fl("small_perimeter_speed") > 0);
    set("small_perimeter_max_length", fl("small_perimeter_speed") > 0);

    const char *const arachne_keys[] = {
        "wall_transition_length", "wall_transition_filter_deviation", "wall_transition_angle",
        "wall_distribution_count", "min_feature_size", "min_bead_width"
    };
    for (const char *key : arachne_keys)
        set(key, A);

    set("overhangs_extrusion_spacing", not_A);
    set("perimeter_loop", P && !A);

    set("seam_slope_type", ext_not_loop);
    const bool scarf = config.opt_enum<SeamScarfType>("seam_slope_type") != SeamScarfType::None;
    set("seam_slope_min_height", ext_not_loop && scarf);
    set("seam_slope_max_length", ext_not_loop && scarf);
    set("external_perimeters_first_force", ext_not_loop && !A);
    set("external_perimeters_nothole", ext_not_loop && !A && !b("external_perimeters_first_force"));
    set("external_perimeters_hole", ext_not_loop && !A && !b("external_perimeters_first_force"));
    set("perimeter_bonding", ext && !A && fl("perimeter_overlap") == 100 && fl("external_perimeter_overlap") == 100);


    set("overhangs_reverse", can_reverse);
    set("overhangs_reverse_threshold", can_reverse && b("overhangs_reverse"));
    set("overhangs_speed_enforce", overhangs && !loop_seam);
    set("overhangs_speed", overhangs);
    set("overhangs_width_speed", overhangs);
    set("overhangs_flow_ratio", overhangs);
    set("overhangs_width", overhangs && en("overhangs_flow_ratio"));

    set("seam_angle_cost", seam_cost);
    set("seam_travel_cost", seam_cost);
    set("seam_visibility", seam_cost);
    set("perimeter_loop_seam", loop_seam);
    set("seam_notch_angle", notch);



    const bool fuzzy = config.opt_enum<FuzzySkinType>("fuzzy_skin") != FuzzySkinType::None;
    set("fuzzy_skin_thickness", fuzzy);
    set("fuzzy_skin_point_dist", fuzzy);

    const char *const have_infill_keys[] = {
        "fill_aligned_z", "fill_pattern", "infill_connection", "solid_infill_every_layers",
        "solid_infill_below_area", "solid_infill_below_layer_area", "solid_infill_below_width",
        "infill_extruder"
    };
    for (const char *key : have_infill_keys)
        set(key, H);
    const bool anchors = H && config.opt_enum<InfillConnection>("infill_connection") != InfillConnection::icNotConnected;
    set("infill_anchor_max", anchors);
    set("infill_anchor", anchors && fl("infill_anchor_max") > 0);
    set("infill_dense", fl("fill_density") < 50);
    set("infill_dense_algo", b("infill_dense") && fl("fill_density") < 50);
    set("infill_every_layers", H && !(b("infill_dense") && fl("fill_density") < 50));
    set("infill_only_where_needed", H && !(b("infill_dense") && fl("fill_density") < 50));

    const char *const solid_keys[] = {
        "enforce_full_fill_volume", "external_infill_margin", "bridged_infill_margin",
        "solid_infill_extruder", "solid_infill_extrusion_width", "solid_infill_extrusion_spacing",
        "solid_infill_extrusion_change_odd_layers"
    };
    for (const char *key : solid_keys)
        set(key, has_solid);

    set("infill_first", infill_or_solid);
    const char *const infill_or_solid_keys[] = {
        "fill_angle_cross", "fill_angle_follow_model", "fill_angle_increment", "fill_angle_template",
        "bridge_angle", "infill_extrusion_width", "infill_extrusion_spacing",
        "infill_extrusion_change_odd_layers"
    };
    for (const char *key : infill_or_solid_keys)
        set(key, infill_or_solid);
    set("fill_angle", infill_or_solid && config.option<ConfigOptionFloats>("fill_angle_template")->size() == 0);

    const bool ensure = config.opt_enum<EnsureVerticalShellThickness>("ensure_vertical_shell_thickness") !=
        EnsureVerticalShellThickness::Disabled;
    set("top_solid_min_thickness", !has_spiral && has_top && ensure);
    set("bottom_solid_min_thickness", !has_spiral && has_bottom && ensure);

    const char *const top_keys[] = {
        "ironing", "top_fill_pattern", "infill_connection_top", "top_infill_extrusion_width",
        "top_infill_extrusion_spacing"
    };
    for (const char *key : top_keys)
        set(key, has_top);
    set("bottom_fill_pattern", has_bottom);
    set("infill_connection_bottom", has_bottom);
    set("solid_fill_pattern", has_solid);
    set("infill_connection_solid", has_solid);
    set("bridge_fill_pattern", has_solid);
    set("infill_connection_bridge", has_solid);

    set("ironing_type", pp_ironing);
    set("ironing_flowrate", pp_ironing);
    set("ironing_spacing", pp_ironing);
    set("ironing_angle", pp_ironing);
    set("fill_smooth_width", smooth);
    set("fill_smooth_distribution", smooth);

    const char *const skirt_keys[] = {
        "skirt_width", "skirt_brim", "skirt_distance", "skirt_distance_from_brim",
        "draft_shield", "min_skirt_length", "skirt_extrusion_width"
    };
    for (const char *key : skirt_keys)
        set(key, have_skirt);
    set("skirt_height", have_skirt && config.opt_enum<DraftShield>("draft_shield") != dsEnabled);
    set("perimeter_extruder", P || have_brim);
    set("perimeter_extrusion_width", P || have_brim);
    set("perimeter_extrusion_spacing", P || have_brim);
    set("perimeter_extrusion_change_odd_layers", P || have_brim);
    set("support_material_extruder", have_support || have_skirt);
    set("support_material_speed", have_support || have_skirt || have_brim);

    const char *const support_keys[] = {
        "support_material_style", "support_material_extrusion_width",
        "support_material_contact_distance_type", "support_material_xy_spacing",
        "support_material_bottom_interface_layers", "support_material_interface_layers"
    };
    for (const char *key : support_keys)
        set(key, have_support);
    set("support_material_threshold", have_support && b("support_material_auto"));
    set("support_material_bottom_contact_distance", have_support && !soluble);
    set("support_material_contact_distance", have_support && !soluble);
    set("support_material_closing_radius", have_support && style == SupportMaterialStyle::smsSnug);

    const char *const tree_family_keys[] = {
        "support_tree_angle", "support_tree_angle_slow", "support_tree_branch_diameter",
        "support_tree_branch_diameter_angle", "support_tree_branch_diameter_double_wall",
        "support_tree_tip_diameter", "support_tree_branch_distance", "support_tree_top_rate",
        "support_tree_max_overhang"
    };
    for (const char *key : tree_family_keys)
        set(key, tree_family);

    const char *const iface_keys[] = {
        "support_material_interface_spacing", "support_material_interface_extruder",
        "support_material_interface_speed", "support_material_interface_angle_increment",
        "support_material_bottom_interface_expansion"
    };
    for (const char *key : iface_keys)
        set(key, have_iface);
    set("support_material_interface_layer_height", have_iface);
    set("support_material_interface_angle", have_iface);
    set("support_material_bottom_interface_pattern", have_iface);
    set("support_material_interface_contact_loops", have_iface);
    set("support_material_with_sheath", have_support && !organic);
    set("support_material_pattern", have_support && !organic);
    set("support_material_spacing", have_support && !organic);
    set("support_material_angle", have_support && !organic);
    set("support_material_angle_height", have_support && !organic);
    set("support_material_layer_height", have_support);

    // Contact gaps do not disable synchronization of intermediate support layers; only grid and snug use it.
    set("support_material_synchronize_layers", have_support &&
        (style == SupportMaterialStyle::smsGrid || style == SupportMaterialStyle::smsSnug));

    const bool raft_soluble = config.opt_enum<SupportZDistanceType>("raft_contact_distance_type") == zdNone;
    set("raft_contact_distance", raft && !raft_soluble);
    // The first layer foot and density apply to every support: the option, a raft or enforced layers.
    const bool support_foot = supp || raft || i("support_material_enforce_layers") > 0;
    set("raft_first_layer_density", support_foot);
    set("raft_first_layer_expansion", support_foot);
    set("raft_expansion", raft);
    set("first_layer_acceleration_over_raft", raft);
    set("first_layer_speed_over_raft", raft);
    set("raft_layer_height", raft);
    set("raft_interface_layer_height", raft);

    set("extrusion_width", default_width);
    set("extrusion_spacing", default_width);
    set("first_layer_extrusion_spacing", en("first_layer_extrusion_width"));
    set("first_layer_infill_extrusion_spacing", en("first_layer_infill_extrusion_width"));

    set("complete_objects_sort", seq);
    set("parallel_objects_step", seq);
    set("parallel_objects_step_max_z", parallel_ok);
    set("parallel_islands", parallel_ok);
    set("standby_temperature_delta", b("ooze_prevention"));
    set("wipe_tower", have_wipe);
    set("wipe_tower_per_object", !spiral);
    set("wipe_tower_object_x", wipe_ready && (whole || own_tower));
    set("wipe_tower_object_y", wipe_ready && (whole || own_tower));
    set("wipe_tower_extruder_toolchange", wipe_ready && i("wipe_tower_extruder") != 0);
    set("wipe_tower_x", have_wipe && b("wipe_tower") && !whole);
    set("wipe_tower_y", have_wipe && b("wipe_tower") && !whole);
    const char *const wipe_keys[] = {
        "wipe_tower_width", "wipe_tower_rotation_angle",
        "wipe_tower_brim_width", "wipe_tower_cone_angle", "wipe_tower_extra_spacing",
        "wipe_tower_bridging", "wipe_tower_brim", "wipe_tower_no_sparse_layers",
        "single_extruder_multi_material_priming", "wipe_tower_speed",
        "wipe_tower_wipe_starting_speed", "wipe_tower_extrusion_width", "wipe_tower_rest_in_middle"
    };
    for (const char *key : wipe_keys)
        set(key, wipe_ready);
    set("mmu_segmented_region_interlocking_depth", fl("mmu_segmented_region_max_width") != 0);

    set("avoid_crossing_curled_overhangs", !avoid);
    set("avoid_crossing_perimeters", !b("avoid_crossing_curled_overhangs"));
    set("avoid_crossing_perimeters_max_detour", avoid);
    set("avoid_crossing_not_first_layer", avoid);
    set("avoid_crossing_top", avoid);
    set("avoid_travel_island", avoid);
    set("avoid_travel_island_weight", avoid && b("avoid_travel_island"));
    set("enforce_retract_first_layer", b("only_retract_when_crossing_perimeters"));
    set("milling_after_z", b("milling_post_process"));
    set("milling_extra_size", b("milling_post_process"));

    // The travel deceleration and feature acceleration settings are defined by
    // their plugins, so the reference skips them; their activation rules are
    // covered by the plugin tests.
    set("perimeter_acceleration", default_accel && P);
    set("support_material_acceleration", default_accel && (have_support || have_brim || have_skirt));
    set("support_material_interface_acceleration", default_accel && have_iface);

    const bool percent_speed =
        config.option<ConfigOptionFloatOrPercent>("perimeter_speed")->percent ||
        config.option<ConfigOptionFloatOrPercent>("solid_infill_speed")->percent ||
        config.option<ConfigOptionFloatOrPercent>("bridge_speed")->percent ||
        config.option<ConfigOptionFloatOrPercent>("support_material_speed")->percent;
    set("default_speed", percent_speed);

    return out;
}

EnabledMap rule_enabled(const Orchestrator &orchestrator, const DynamicPrintConfig &config)
{
    EnabledMap out;
    const char *const owners[] = { core_gui_rules_owner, infill_gui_rules_owner };
    for (const char *owner : owners) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1, owner))
            out[application.target_key] = application.enabled;
    }
    return out;
}

// Mutate the owned config using a borrowed seeded generator, including tower eligibility and contour-tool states; no ownership transfers and option failures propagate.
void randomize(DynamicPrintConfig &config, std::mt19937 &rng)
{
    std::uniform_int_distribution<int> small(0, 3);
    std::uniform_int_distribution<int> coin(0, 1);
    std::uniform_int_distribution<int> percent(0, 3);
    const double densities[] = { 0.0, 15.0, 50.0, 60.0 };
    config.option<ConfigOptionInt>("perimeters")->value = small(rng);
    config.option<ConfigOptionInt>("top_solid_layers")->value = small(rng);
    config.option<ConfigOptionInt>("bottom_solid_layers")->value = small(rng);
    config.option<ConfigOptionInt>("solid_infill_every_layers")->value = small(rng);
    config.option<ConfigOptionInt>("skirts")->value = small(rng);
    config.option<ConfigOptionInt>("raft_layers")->value = small(rng);
    config.option<ConfigOptionInt>("support_material_interface_layers")->value = small(rng);
    config.option<ConfigOptionInt>("support_material_bottom_interface_layers")->value = small(rng);
    config.option<ConfigOptionInt>("support_material_enforce_layers")->value = small(rng);
    config.option<ConfigOptionPercent>("fill_density")->value = densities[percent(rng)];
    config.option<ConfigOptionPercent>("thin_perimeters")->value = percent(rng) == 0 ? 0.0 : 100.0;
    config.option<ConfigOptionPercent>("perimeter_overlap")->value = percent(rng) == 0 ? 100.0 : 50.0;
    config.option<ConfigOptionPercent>("external_perimeter_overlap")->value = percent(rng) == 0 ? 100.0 : 20.0;
    config.option<ConfigOptionFloat>("brim_width")->value = coin(rng) ? 5.0 : 0.0;
    config.option<ConfigOptionFloat>("brim_width_interior")->value = coin(rng) ? 3.0 : 0.0;
    config.option<ConfigOptionFloat>("solid_infill_below_area")->value = coin(rng) ? 5.0 : 0.0;
    config.option<ConfigOptionFloatOrPercent>("small_perimeter_speed")->value = coin(rng) ? 10.0 : 0.0;
    config.option<ConfigOptionFloat>("max_volumetric_speed")->value = coin(rng) ? 5.0 : 0.0;
    config.option<ConfigOptionFloat>("parallel_objects_step")->value = coin(rng) ? 1.0 : 0.0;
    config.option<ConfigOptionInt>("wipe_tower_extruder")->value = small(rng);
    for (const char *key : { "first_layer_extrusion_width", "external_perimeter_extrusion_width",
                             "perimeter_extrusion_width", "infill_extrusion_width",
                             "solid_infill_extrusion_width", "top_infill_extrusion_width",
                             "support_material_extrusion_width", "skirt_extrusion_width" })
        config.option<ConfigOptionFloatOrPercent>(key)->value = coin(rng) ? 0.0 : 0.45;
    config.option<ConfigOptionFloatOrPercent>("default_acceleration")->value = coin(rng) ? 1000.0 : 0.0;
    config.option<ConfigOptionFloatOrPercent>("infill_anchor_max")->value = coin(rng) ? 5.0 : 0.0;
    config.option("first_layer_extrusion_width")->set_enabled(coin(rng) != 0);
    config.option("first_layer_infill_extrusion_width")->set_enabled(coin(rng) != 0);
    config.option("overhangs_flow_ratio")->set_enabled(coin(rng) != 0);
    for (const char *key : { "spiral_vase", "external_perimeters_first", "perimeter_loop",
                             "overhangs", "perimeter_reverse", "thin_walls", "only_one_perimeter_top",
                             "support_material", "support_material_auto", "ooze_prevention",
                              "wipe_tower", "wipe_tower_per_object", "parallel_islands", "avoid_crossing_perimeters",
                             "avoid_crossing_curled_overhangs", "avoid_travel_island",
                             "milling_post_process", "infill_dense", "gap_fill_enabled",
                             "infill_filled_bottom", "infill_filled_solid", "infill_filled_top",
                             "external_perimeters_first_force" })
        config.option<ConfigOptionBool>(key)->value = coin(rng) != 0;

    // Dynamic configs may store an enum as ConfigOptionEnum<T> or as
    // ConfigOptionEnumGeneric, so write through the virtual int accessor.
    const auto set_enum = [&config](const char *key, int value) { config.option(key)->set_int(value); };
    set_enum("perimeter_generator", coin(rng) ? int(PerimeterGeneratorType::Arachne) : int(PerimeterGeneratorType::Classic));
    set_enum("support_material_style", small(rng) % 5);
    set_enum("fuzzy_skin", coin(rng) ? int(FuzzySkinType::External) : int(FuzzySkinType::None));
    set_enum("seam_slope_type", coin(rng) ? int(SeamScarfType::External) : int(SeamScarfType::None));
    set_enum("infill_connection", coin(rng) ? int(InfillConnection::icNotConnected) : int(InfillConnection::icConnected));
    set_enum("ensure_vertical_shell_thickness", coin(rng) ? int(EnsureVerticalShellThickness::Enabled) : int(EnsureVerticalShellThickness::Disabled));
    set_enum("draft_shield", coin(rng) ? int(dsEnabled) : int(dsDisabled));
    set_enum("support_material_contact_distance_type", coin(rng) ? int(zdNone) : int(zdFilament));
    set_enum("raft_contact_distance_type", coin(rng) ? int(zdNone) : int(zdFilament));
    set_enum("seam_position", coin(rng) ? int(spCost) : int(spAligned));
    config.set_deserialize_strict("top_fill_pattern", coin(rng) ? "smooth" : "rectilinear");
    config.set_deserialize_strict("print_plan_builder", coin(rng) ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer");
    config.set_deserialize_strict("bottom_fill_pattern", coin(rng) ? "smooth" : "rectilinear");
    // solid_fill_pattern is a plugin-defined enum storing plugin ids.
    config.set_deserialize_strict("solid_fill_pattern", coin(rng) ? "smooth" : "rectilinear");
    set_enum("support_material_top_interface_pattern", coin(rng) ? int(InfillPattern::ipSmooth) : int(InfillPattern::ipRectilinear));
    set_enum("support_material_bottom_interface_pattern", coin(rng) ? int(InfillPattern::ipSmooth) : int(InfillPattern::ipRectilinear));
}

} // namespace

TEST_CASE("FFF print GUI rules match independent activation logic", "[plugins][gui-rules][fff-print]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    bool has_fff_rules = false;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        has_fff_rules = has_fff_rules ||
            (rule.scope == RAW_PRESET_TYPE_FFF_PRINT && rule.owner == core_gui_rules_owner);
    REQUIRE(has_fff_rules);

    // The rules are registered once: a second initialization must not add any.
    const size_t rule_count = orchestrator.gui_rules().rules().size();
    slic3r_api::GuiRulesCorePlugin::register_gui_rules_core_plugin(orchestrator.handle());
    CHECK(orchestrator.gui_rules().rules().size() == rule_count);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const EnabledMap expected = reference_enabled(config);
    const EnabledMap actual = rule_enabled(orchestrator, config);
    REQUIRE(actual.size() == expected.size());
    for (const auto &entry : expected) {
        const auto found = actual.find(entry.first);
        REQUIRE(found != actual.end());
        INFO("key " << entry.first);
        CHECK(found->second == entry.second);
    }
}

TEST_CASE("FFF print GUI rules match independent activation logic on a seeded fuzz",
          "[plugins][gui-rules][fff-print][fuzz]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &orchestrator = Orchestrator::instance();

    std::mt19937 rng(0x5eed1234u);
    for (int iteration = 0; iteration < 800; ++iteration) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        randomize(config, rng);

        const EnabledMap expected = reference_enabled(config);
        const EnabledMap actual = rule_enabled(orchestrator, config);
        REQUIRE(actual.size() == expected.size());
        for (const auto &entry : expected) {
            const auto found = actual.find(entry.first);
            REQUIRE(found != actual.end());
            INFO("iteration " << iteration << " key " << entry.first);
            REQUIRE(found->second == entry.second);
        }
    }
}

// Spell out the expected activation of each support setting group for every style, with support on and off.
// The tree and organic styles share the support_tree_* settings; interfaces and the layer height are common to all
// styles; the sparse base fill settings serve grid, snug, tree and filled but not organic (its body is the branch sheath).
// Evaluate all registered activation reasons with automatic sparse selection; organic branch controls stay editable.
TEST_CASE("Auto support toolpath selectors keep every provider setting editable",
          "[plugins][gui-rules][support-toolpath][support-toolpath-auto]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &orchestrator = Orchestrator::instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"support_material", "1"}, {"support_material_style", "organic"},
        {"exclusive_group_1350_support_toolpath_sparse_plugin", SLIC3R_SELECTOR_AUTO_VALUE}});
    const char *const key = "support_tree_branch_diameter_double_wall";
    REQUIRE(config.has(key));
    bool owned_auto_rule = false;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        if (rule.target_key == key && rule.owner == "support.toolpath.branch_sheath")
            for (const Orchestrator::PluginGuiRuleClause &clause : rule.clauses)
                if (clause.condition == RAW_GUI_RULE_CONDITION_STRING_EQUALS &&
                    clause.condition_key == "exclusive_group_1350_support_toolpath_sparse_plugin" &&
                    clause.condition_string_value == SLIC3R_SELECTOR_AUTO_VALUE)
                    owned_auto_rule = true;
    REQUIRE(owned_auto_rule);
    bool found = false;
    for (const Orchestrator::GuiRuleApplication &application :
         orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1)) {
        if (application.target_key == key) {
            found = true;
            CHECK(application.enabled);
        }
    }
    REQUIRE(found);
}

TEST_CASE("Support setting groups follow the support style", "[plugins][gui-rules][fff-print][support]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &orchestrator = Orchestrator::instance();

    const char *const tree_keys[] = { "support_tree_angle", "support_tree_branch_diameter", "support_tree_top_rate",
                                      "support_tree_max_overhang" };
    const char *const interface_keys[] = { "support_material_interface_layer_height", "support_material_interface_angle",
                                           "support_material_bottom_interface_pattern",
                                           "support_material_interface_contact_loops" };
    const char *const base_fill_keys[] = { "support_material_with_sheath", "support_material_pattern",
                                           "support_material_spacing", "support_material_angle",
                                           "support_material_angle_height" };

    struct StyleCase { SupportMaterialStyle style; bool tree_settings; bool base_fill; bool synchronization; };
    const StyleCase cases[] = {
        { SupportMaterialStyle::smsGrid,    false, true,  true  },
        { SupportMaterialStyle::smsSnug,    false, true,  true  },
        { SupportMaterialStyle::smsTree,    true,  true,  false },
        { SupportMaterialStyle::smsOrganic, true,  false, false },
        { SupportMaterialStyle::smsFilled,  false, true,  false },
    };
    for (const StyleCase &style_case : cases) {
        for (const bool support_on : { true, false }) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.option<ConfigOptionBool>("support_material")->value = support_on;
            config.option<ConfigOptionInt>("raft_layers")->value = 0;
            config.option<ConfigOptionInt>("support_material_enforce_layers")->value = 0;
            config.option<ConfigOptionInt>("support_material_interface_layers")->value = 2;
            config.option("support_material_style")->set_int(int(style_case.style));
            const EnabledMap enabled = rule_enabled(orchestrator, config);

            INFO("style " << int(style_case.style) << " support " << support_on);
            for (const char *key : tree_keys)
                CHECK(enabled.at(key) == (support_on && style_case.tree_settings));
            for (const char *key : interface_keys)
                CHECK(enabled.at(key) == support_on);
            for (const char *key : base_fill_keys)
                CHECK(enabled.at(key) == (support_on && style_case.base_fill));
            CHECK(enabled.at("support_material_layer_height") == support_on);
            CHECK(enabled.at("support_material_synchronize_layers") == (support_on && style_case.synchronization));
        }
    }

    // Organic style with a raft only: the raft counts as support for the common settings, but no organic branch
    // is built (support off, no enforced layers), so the tree settings stay inactive and the base fill settings
    // stay active.
    DynamicPrintConfig raft_config = DynamicPrintConfig::full_print_config();
    raft_config.option<ConfigOptionBool>("support_material")->value = false;
    raft_config.option<ConfigOptionInt>("raft_layers")->value = 1;
    raft_config.option<ConfigOptionInt>("support_material_enforce_layers")->value = 0;
    raft_config.option<ConfigOptionInt>("support_material_interface_layers")->value = 2;
    raft_config.option("support_material_style")->set_int(int(SupportMaterialStyle::smsOrganic));
    const EnabledMap raft_enabled = rule_enabled(orchestrator, raft_config);
    for (const char *key : tree_keys)
        CHECK_FALSE(raft_enabled.at(key));
    for (const char *key : interface_keys)
        CHECK(raft_enabled.at(key));
    for (const char *key : base_fill_keys)
        CHECK(raft_enabled.at(key));
    CHECK(raft_enabled.at("support_material_layer_height"));

    // The first layer settings apply when support, a raft or enforced layers exist, and only then.
    struct FootCase { bool support; int raft; int enforce; bool expected; };
    for (const FootCase &foot_case : { FootCase{ false, 0, 0, false }, FootCase{ true, 0, 0, true }, FootCase{ false, 1, 0, true },
                                       FootCase{ false, 0, 3, true } }) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.option<ConfigOptionBool>("support_material")->value = foot_case.support;
        config.option<ConfigOptionInt>("raft_layers")->value = foot_case.raft;
        config.option<ConfigOptionInt>("support_material_enforce_layers")->value = foot_case.enforce;
        const EnabledMap enabled = rule_enabled(orchestrator, config);
        INFO("support " << foot_case.support << " raft " << foot_case.raft << " enforce " << foot_case.enforce);
        CHECK(enabled.at("raft_first_layer_expansion") == foot_case.expected);
        CHECK(enabled.at("raft_first_layer_density") == foot_case.expected);
    }
}

// Compare owned tower activation maps across enable/sequential/vase/parallel/tool combinations, requiring parallel-independent eligibility; no GUI code is loaded and assertion/configuration failures remain visible.
TEST_CASE("Core tower rules distinguish global and instance controls", "[plugins][gui-rules][wipe-tower]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Orchestrator &orchestrator = Orchestrator::instance();
    for (unsigned int state = 0; state < 64; ++state) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        const bool enabled = (state & 1) != 0;
        const bool own = (state & 2) != 0;
        const bool sequential = (state & 4) != 0;
        const bool vase = (state & 8) != 0;
        const bool parallel = (state & 16) != 0;
        const bool contour_tool = (state & 32) != 0;
        config.option<ConfigOptionBool>("wipe_tower")->value = enabled;
        config.option<ConfigOptionBool>("wipe_tower_per_object")->value = own;
        config.set_deserialize_strict("print_plan_builder", sequential ? "ordering.plan_builder.by_object" : "ordering.plan_builder.by_layer");
        config.option<ConfigOptionBool>("spiral_vase")->value = vase;
        config.option<ConfigOptionBool>("parallel_islands")->value = false;
        config.option<ConfigOptionFloat>("parallel_objects_step")->value = parallel ? 5. : 0.;
        config.option<ConfigOptionInt>("wipe_tower_extruder")->value = contour_tool ? 2 : 0;
        const EnabledMap actual = rule_enabled(orchestrator, config);
        const EnabledMap expected = reference_enabled(config);
        CAPTURE(state);
        REQUIRE(actual == expected);
        CHECK(actual.at("wipe_tower_per_object") == !vase);
        const bool possible = !vase;
        CHECK(actual.at("wipe_tower") == possible);
        CHECK(actual.at("wipe_tower_object_x") == (possible && (enabled || own) && ((sequential && !parallel) || own)));
        CHECK(actual.at("wipe_tower_object_y") == actual.at("wipe_tower_object_x"));
        CHECK(actual.at("wipe_tower_x") == (possible && enabled && (!sequential || parallel)));
        CHECK(actual.at("wipe_tower_y") == actual.at("wipe_tower_x"));
        CHECK(actual.at("wipe_tower_extruder_toolchange") == (possible && (enabled || own) && contour_tool));
    }
}

TEST_CASE("GUI rules preserve clauses, scope and owner", "[plugins][gui-rules][abi]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    raw_gui_rule_clause clauses[1] = { raw_gui_rule_clause_init() };
    clauses[0].condition = RAW_GUI_RULE_CONDITION_BOOL_TRUE;
    clauses[0].condition_key = "spiral_vase";

    const char *const test_owner = "gui_rule_abi_test_owner";
    raw_gui_rule rule = raw_gui_rule_init();
    rule.target_key = "fill_aligned_z";
    rule.scope = RAW_PRESET_TYPE_FFF_PRINT;
    rule.clauses = clauses;
    rule.clause_count = 1;

    CHECK(orchestrator.gui_rules().add(&rule, test_owner) == Orchestrator::GuiRuleRegistration::Added);
    CHECK(orchestrator.gui_rules().add(&rule, test_owner) == Orchestrator::GuiRuleRegistration::Duplicate);

    bool found = false;
    size_t stored_index = 0;
    for (size_t rule_idx = 0; rule_idx < orchestrator.gui_rules().rules().size(); ++rule_idx) {
        const Orchestrator::PluginGuiRule &stored = orchestrator.gui_rules().rules()[rule_idx];
        if (stored.owner != test_owner)
            continue;
        found = true;
        stored_index = rule_idx;
        CHECK(stored.target_key == "fill_aligned_z");
        CHECK(stored.scope == RAW_PRESET_TYPE_FFF_PRINT);
        REQUIRE(stored.clauses.size() == 1);
        CHECK(stored.clauses[0].condition == RAW_GUI_RULE_CONDITION_BOOL_TRUE);
        CHECK(stored.clauses[0].condition_key == "spiral_vase");
    }
    CHECK(found);

    // The rule is attached to the activation group of its setting.
    const Orchestrator::SettingActivation *activation = orchestrator.gui_rules().setting_activation_for("fill_aligned_z");
    REQUIRE(activation != nullptr);
    CHECK(activation->setting_key == "fill_aligned_z");
    CHECK(std::find(activation->rule_indices.begin(), activation->rule_indices.end(), stored_index) !=
          activation->rule_indices.end());

    const size_t stored_count = orchestrator.gui_rules().rules().size();
    const auto expect_invalid = [&orchestrator, stored_count](raw_gui_rule &candidate, const char *reason) {
        INFO(reason);
        CHECK(orchestrator.gui_rules().add(&candidate) == Orchestrator::GuiRuleRegistration::Invalid);
        CHECK(orchestrator.gui_rules().rules().size() == stored_count);
    };

    // Descriptor level validation: an unknown enum value must be rejected
    // instead of being stored and silently ignored by the evaluator.
    raw_gui_rule invalid = rule;
    invalid.target_key = nullptr;
    expect_invalid(invalid, "null target_key");

    invalid = rule;
    invalid.target_key = "";
    expect_invalid(invalid, "empty target_key");

    invalid = rule;
    invalid.target_index = -3;
    expect_invalid(invalid, "invalid target_index");

    invalid = rule;
    invalid.scope = static_cast<raw_option_preset_type>(RAW_PRESET_TYPE_COUNT + 10);
    expect_invalid(invalid, "unknown scope");

    invalid = rule;
    invalid.clause_count = 0;
    expect_invalid(invalid, "no condition clause");

    invalid = rule;
    invalid.clauses = nullptr;
    expect_invalid(invalid, "null clause array");

    invalid = rule;
    invalid.struct_size = sizeof(uint32_t);
    expect_invalid(invalid, "truncated struct_size");

    // A real plugin compiled against an older descriptor may allocate only
    // struct_size bytes. The rejection path must not read target_key from
    // memory the caller never provided: only struct_size is readable here,
    // so a regression is caught by sanitizer builds.
    std::vector<uint32_t> short_storage(1, 0);
    raw_gui_rule *short_rule = reinterpret_cast<raw_gui_rule *>(short_storage.data());
    short_rule->struct_size = sizeof(uint32_t);
    expect_invalid(*short_rule, "descriptor allocated shorter than its struct_size");

    // Clause level validation.
    raw_gui_rule_clause invalid_clause = clauses[0];
    invalid = rule;
    invalid.clauses = &invalid_clause;

    invalid_clause.condition = RAW_GUI_RULE_CONDITION_NONE;
    expect_invalid(invalid, "condition NONE");

    invalid_clause = clauses[0];
    invalid_clause.condition = static_cast<raw_gui_rule_condition>(99);
    expect_invalid(invalid, "unknown condition");

    invalid_clause = clauses[0];
    invalid_clause.condition_key = nullptr;
    expect_invalid(invalid, "null clause condition_key");

    invalid_clause = clauses[0];
    invalid_clause.condition_key = "";
    expect_invalid(invalid, "empty clause condition_key");

    invalid_clause = clauses[0];
    invalid_clause.condition_index = -3;
    expect_invalid(invalid, "invalid clause condition_index");

    invalid_clause = clauses[0];
    invalid_clause.condition = RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN;
    invalid_clause.condition_float_value = std::numeric_limits<double>::quiet_NaN();
    expect_invalid(invalid, "non-finite float condition");

    invalid_clause = clauses[0];
    invalid_clause.condition = RAW_GUI_RULE_CONDITION_INT_EQUALS;
    invalid_clause.condition_float_value = std::numeric_limits<double>::quiet_NaN();
    expect_invalid(invalid, "non-finite int condition");

    invalid_clause = clauses[0];
    invalid_clause.condition = RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS;
    invalid_clause.condition_float_value = std::numeric_limits<double>::infinity();
    expect_invalid(invalid, "infinite int condition");

    // The public C relay must distinguish added, duplicate and invalid.
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    raw_gui_rule relay = rule;
    relay.target_key = "gui_rule_c_relay_target";
    CHECK(orchestrator_add_gui_rule(handle, &relay) == 1);
    CHECK(orchestrator_add_gui_rule(handle, &relay) == 0);
    relay.scope = static_cast<raw_option_preset_type>(RAW_PRESET_TYPE_COUNT + 10);
    CHECK(orchestrator_add_gui_rule(handle, &relay) == -1);
    CHECK(orchestrator_add_gui_rule(handle, nullptr) == -1);
}

TEST_CASE("GUI activation rules attach to settings and queue unknown ones", "[plugins][gui-rules][abi]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_BOOL_TRUE;
    clause.condition_key = "spiral_vase";

    raw_gui_rule pending = raw_gui_rule_init();
    pending.target_key = "gui_rule_pending_test_target";
    pending.scope = RAW_PRESET_TYPE_FFF_PRINT;
    pending.clauses = &clause;
    pending.clause_count = 1;
    CHECK(orchestrator.gui_rules().add(&pending, "gui_rule_pending_owner") ==
          Orchestrator::GuiRuleRegistration::Added);
    // Unknown settings have no activation group and stay queued for a warning.
    CHECK(orchestrator.gui_rules().setting_activation_for("gui_rule_pending_test_target") == nullptr);
    CHECK(orchestrator.gui_rules().warn_pending_rules() >= 1);
}

TEST_CASE("GUI rule INT conditions compare the rounded float operand", "[plugins][gui-rules][abi]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    raw_gui_rule_clause int_equals = raw_gui_rule_clause_init();
    int_equals.condition = RAW_GUI_RULE_CONDITION_INT_EQUALS;
    int_equals.condition_key = "solid_infill_every_layers";
    int_equals.condition_float_value = 2.4;

    const char *const int_owner = "gui_rule_int_owner";
    raw_gui_rule equals_rule = raw_gui_rule_init();
    equals_rule.target_key = "fill_aligned_z";
    equals_rule.scope = RAW_PRESET_TYPE_FFF_PRINT;
    equals_rule.clauses = &int_equals;
    equals_rule.clause_count = 1;
    CHECK(orchestrator.gui_rules().add(&equals_rule, int_owner) == Orchestrator::GuiRuleRegistration::Added);

    raw_gui_rule_clause int_not_equals = int_equals;
    int_not_equals.condition = RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS;

    raw_gui_rule not_equals_rule = equals_rule;
    not_equals_rule.target_key = "infill_every_layers";
    not_equals_rule.clauses = &int_not_equals;
    CHECK(orchestrator.gui_rules().add(&not_equals_rule, int_owner) == Orchestrator::GuiRuleRegistration::Added);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled_for = [&orchestrator, &config, int_owner](const char *target_key) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1, int_owner))
            if (application.target_key == target_key)
                return application.enabled;
        return false;
    };

    config.option<ConfigOptionInt>("solid_infill_every_layers")->value = 2;
    CHECK(enabled_for("fill_aligned_z"));
    CHECK_FALSE(enabled_for("infill_every_layers"));

    config.option<ConfigOptionInt>("solid_infill_every_layers")->value = 3;
    CHECK_FALSE(enabled_for("fill_aligned_z"));
    CHECK(enabled_for("infill_every_layers"));
}

TEST_CASE("Page filter restricts GUI rule evaluation", "[plugins][gui-rules][filter]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto evaluate_map = [&orchestrator, &config](int32_t current_index,
                                                       const std::vector<OptionKeyIdx> &filter,
                                                       const char *owner = core_gui_rules_owner) {
        EnabledMap out;
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, current_index, owner, filter))
            out[application.target_key + "#" + std::to_string(application.target_index)] = application.enabled;
        return out;
    };

    // A filtered refresh only visits the rules of the displayed settings and
    // returns the same state as the full scan for those settings.
    const std::vector<OptionKeyIdx> page = {
        OptionKeyIdx{ "fill_aligned_z", -1 },
        OptionKeyIdx{ "infill_every_layers", -1 },
    };
    const EnabledMap full = evaluate_map(-1, {});
    const EnabledMap filtered = evaluate_map(-1, page);
    CHECK_FALSE(filtered.empty());
    CHECK(filtered.size() <= page.size());
    for (const auto &entry : filtered) {
        const auto found = full.find(entry.first);
        REQUIRE(found != full.end());
        CHECK(found->second == entry.second);
    }

    // The per-extruder pass only keeps fields of the current index: a field of
    // index 0 is not evaluated while refreshing extruder 1.
    const std::vector<OptionKeyIdx> extruder_zero = { OptionKeyIdx{ "fill_aligned_z", 0 } };
    CHECK(evaluate_map(1, extruder_zero).empty());
    CHECK_FALSE(evaluate_map(-1, extruder_zero).empty());

    // A rule whose setting has no definition is queued and only reachable
    // through the full scan.
    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_BOOL_TRUE;
    clause.condition_key = "spiral_vase";

    raw_gui_rule unattached = raw_gui_rule_init();
    unattached.target_key = "gui_rule_filter_pending_target";
    unattached.scope = RAW_PRESET_TYPE_FFF_PRINT;
    unattached.clauses = &clause;
    unattached.clause_count = 1;
    CHECK(orchestrator.gui_rules().add(&unattached, "gui_rule_filter_owner") ==
          Orchestrator::GuiRuleRegistration::Added);

    config.set_key_value("gui_rule_filter_pending_target", new ConfigOptionBool(false));
    const std::vector<OptionKeyIdx> pending_page = { OptionKeyIdx{ "gui_rule_filter_pending_target", -1 } };
    const EnabledMap filtered_pending = evaluate_map(-1, pending_page, "gui_rule_filter_owner");
    CHECK(filtered_pending.empty());

    const auto full_pending = [&orchestrator, &config]() {
        EnabledMap out;
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1, "gui_rule_filter_owner"))
            out[application.target_key] = application.enabled;
        return out;
    }();
    CHECK(full_pending.find("gui_rule_filter_pending_target") != full_pending.end());
}
TEST_CASE("GUI rules activate perimeters for every perimeter generator", "[plugins][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    REQUIRE(PrintConfigDef::instance().get("step_perimeter_plugin") != nullptr);

    // The generated used-config-key rules must keep the wall count editable for
    // every implementation that reads it, not only for the one that declared it
    // first.
    for (const char *plugin_id : {"perimeter.generator.classic", "perimeter.generator.arachne", "perimeter.generator.simple"}) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize("step_perimeter_plugin", plugin_id);

        bool found = false;
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1)) {
            if (application.target_key != "perimeters")
                continue;
            found = true;
            INFO("generator " << plugin_id);
            CHECK(application.enabled);
        }
        INFO("generator " << plugin_id);
        CHECK(found);
    }
}
