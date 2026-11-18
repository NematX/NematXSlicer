///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
GuiRulesCore.cpp
================

Built-in activation rules for the host settings that live in the core
print configuration, published as the always-on `gui_rules.core` plugin. Each
entry answers one question: under which configured features is this setting
relevant? Clauses reference core option keys by name and the enum selectors that
plugins mirror through GuiRuleHelpers.hpp. The static assertions below keep
those mirrored selector values equal to the host enums.

The shared conjunctions (has_top, have_solid, have_support, ...) describe
feature families; plugin generators build equivalent local copies for the
settings they own, so both sides must stay in sync.
*/

#include "GuiRulesCore.hpp"

#include <initializer_list>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace slic3r_api { namespace GuiRulesCorePlugin {

namespace {

const char *k_gui_rules_core_id = "gui_rules.core";
const char *k_no_dependencies[] = { nullptr };

} // namespace

namespace {

using slic3r_api::all_of;
using slic3r_api::any_of;
using slic3r_api::GuiCondition;
using slic3r_api::GuiRule;

using slic3r_api::disabled;
using slic3r_api::enabled;
using slic3r_api::false_;
using slic3r_api::float_equals;
using slic3r_api::float_greater_or_equal;
using slic3r_api::float_less_than;
using slic3r_api::int_equals;
using slic3r_api::int_not_equals;
using slic3r_api::non_zero;
using slic3r_api::percent;
using slic3r_api::string_equals;
using slic3r_api::true_;
using slic3r_api::vector_empty;

// The rule conditions reference the built-in option enums (perimeter
// generator, support style, ...) declared in Slic3r.
using namespace Slic3r;

// Keep the selector values mirrored by GuiRuleHelpers.hpp in sync with the host
// enums; plugins read the mirrored values instead of the configuration headers.
static_assert(int(Slic3r::PerimeterGeneratorType::Classic) == 0, "perimeter generator selector changed");
static_assert(int(Slic3r::PerimeterGeneratorType::Arachne) == 1, "perimeter generator selector changed");
static_assert(int(Slic3r::InfillPattern::ipSmooth) == 17, "infill pattern selector changed");

// Shared conjunctions used by several rules below.
const GuiCondition has_top = any_of({
    non_zero("top_solid_layers"),
    all_of({ non_zero("perimeters"), true_("spiral_vase") }),
    int_equals("solid_infill_every_layers", 1)
});
const GuiCondition has_bottom = any_of({
    non_zero("bottom_solid_layers"),
    int_equals("solid_infill_every_layers", 1)
});
const GuiCondition has_solid = any_of({
    non_zero("top_solid_layers"),
    all_of({ non_zero("perimeters"), true_("spiral_vase") }),
    int_equals("solid_infill_every_layers", 1),
    non_zero("bottom_solid_layers"),
    all_of({ non_zero("fill_density"), non_zero("solid_infill_every_layers") }),
    all_of({ non_zero("fill_density"), non_zero("solid_infill_below_area") })
});
const GuiCondition have_support = any_of({
    true_("support_material"),
    non_zero("raft_layers")
});
// A support foot exists: the support option, a raft, or enforced support layers (which build support without the option).
const GuiCondition has_support_foot = any_of({
    true_("support_material"),
    non_zero("raft_layers"),
    non_zero("support_material_enforce_layers")
});
const GuiCondition have_brim = any_of({
    non_zero("brim_width"),
    non_zero("brim_width_interior")
});
const GuiCondition have_interface = any_of({
    all_of({ true_("support_material"), non_zero("support_material_interface_layers") }),
    all_of({ true_("support_material"), non_zero("support_material_bottom_interface_layers") }),
    all_of({ non_zero("raft_layers"), non_zero("support_material_interface_layers") }),
    all_of({ non_zero("raft_layers"), non_zero("support_material_bottom_interface_layers") })
});
// The tree and organic styles are both served by the tree support family, so they share the
// support_tree_* settings. The family runs for the support option or for enforced layers.
const GuiCondition has_tree_family = any_of({
    all_of({ any_of({ int_equals("support_material_style", int(SupportMaterialStyle::smsTree)),
                      int_equals("support_material_style", int(SupportMaterialStyle::smsOrganic)) }),
             true_("support_material") }),
    all_of({ any_of({ int_equals("support_material_style", int(SupportMaterialStyle::smsTree)),
                      int_equals("support_material_style", int(SupportMaterialStyle::smsOrganic)) }),
             non_zero("support_material_enforce_layers") })
});
// The merged grid shares the overhang limit with the tree family but keeps its own layer placement.
const GuiCondition has_grid_merged = all_of({
    int_equals("support_material_style", int(SupportMaterialStyle::smsGridMerged)),
    any_of({ true_("support_material"), non_zero("support_material_enforce_layers") })
});

// base AND NOT(organic supports): NOT(style == organic AND (support OR enforce > 0)).
// The organic body is the branch sheath (the loop that walls an organic branch section), so the
// sparse fill settings (the infill pattern of the support body) do not apply to it; the tree style
// still builds its base slabs (horizontal bands of support between two Z levels) as sparse fills.
GuiCondition without_organic_supports(const GuiCondition &base)
{
    return all_of({
        base,
        any_of({
            int_not_equals("support_material_style", int(SupportMaterialStyle::smsOrganic)),
            all_of({
                false_("support_material"),
                int_equals("support_material_enforce_layers", 0)
            })
        })
    });
}

} // namespace

GuiRulesCore &GuiRulesCore::instance(orchestrator_handle *orch)
{
    static GuiRulesCore s_instance(orch);
    return s_instance;
}

const char *GuiRulesCore::id_impl() const noexcept
{
    return k_gui_rules_core_id;
}

const char *GuiRulesCore::name_impl() const noexcept
{
    return "Built-in GUI rules";
}

const char *GuiRulesCore::description_impl() const noexcept
{
    return "Activation rules of the built-in print settings.";
}

slicing_step_t GuiRulesCore::step_impl() const noexcept
{
    return STEP_NONE;
}

const char *const *GuiRulesCore::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t GuiRulesCore::priority_impl() const noexcept
{
    return 0;
}

// Register owned rules with the borrowed orchestrator; tower controls distinguish global positions and instance offsets without parallel-mode restrictions, and registration failures propagate.
void GuiRulesCore::inilialize_impl(storage_handle *) const
{
    // Register one FFF activation rule: all targets share the FFF scope and
    // every disjunct becomes an activation reason.
    const auto add_rule = [this](std::initializer_list<const char *> target_keys, const GuiCondition &condition) {
        GuiRule::activate(std::vector<std::string>(target_keys.begin(), target_keys.end()))
            .when(condition)
            .scope(RAW_PRESET_TYPE_FFF_PRINT)
            .add_to(m_orchestrator);
    };

    // ----- perimeters ---------------------------------------------------
    add_rule({
        "extra_perimeters",
        "external_perimeters_first",
        "external_perimeter_extrusion_width", "external_perimeter_extrusion_spacing",
        "external_perimeter_extrusion_change_odd_layers", "overhangs", "seam_position",
        "staggered_inner_seams", "perimeter_generator",
        "small_perimeter_speed", "spiral_vase",
        "seam_notch_all", "seam_notch_inner", "seam_notch_outer",
        "no_perimeter_unsupported_algo", "only_one_perimeter_top"
    }, non_zero("perimeters"));

    add_rule({ "overhangs_dynamic_speed" }, all_of({ non_zero("perimeters"), true_("overhangs") }));
    add_rule({ "small_perimeter_min_length", "small_perimeter_max_length" },
             non_zero("small_perimeter_speed"));

    add_rule({ "wall_transition_length", "wall_transition_filter_deviation", "wall_transition_angle",
               "wall_distribution_count", "min_feature_size", "min_bead_width" },
             all_of({ non_zero("perimeters"), int_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) }));

    const GuiCondition not_arachne = any_of({ false_("perimeters"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) });
    add_rule({ "overhangs_extrusion_spacing" }, not_arachne);
    add_rule({ "perimeter_loop" },
             all_of({ non_zero("perimeters"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) }));


    // external perimeter / seam slope
    add_rule({ "seam_slope_type" }, any_of({
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop") }),
        all_of({ true_("external_perimeters_first"), non_zero("perimeters"), int_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) })
    }));
    add_rule({ "seam_slope_min_height", "seam_slope_max_length" }, any_of({
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop"), int_not_equals("seam_slope_type", int(SeamScarfType::None)) }),
        all_of({ true_("external_perimeters_first"), non_zero("perimeters"), int_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)), int_not_equals("seam_slope_type", int(SeamScarfType::None)) })
    }));
    add_rule({ "external_perimeters_first_force" }, any_of({
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop"), false_("perimeters") }),
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) })
    }));
    add_rule({ "external_perimeters_nothole", "external_perimeters_hole" }, any_of({
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop"), false_("perimeters"),
                 false_("external_perimeters_first_force") }),
        all_of({ true_("external_perimeters_first"), false_("perimeter_loop"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)),
                 false_("external_perimeters_first_force") })
    }));
    // FLOAT_* clauses compare get_float(), which is a fraction for percent
    // options, so a full overlap reads 1.0. Comparing against 100 would never
    // match: perimeter_bonding requires both overlaps at their maximum.
    add_rule({ "perimeter_bonding" }, any_of({
        all_of({ true_("external_perimeters_first"), false_("perimeters"),
                 float_equals("perimeter_overlap", 1.0),
                 float_equals("external_perimeter_overlap", 1.0) }),
        all_of({ true_("external_perimeters_first"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)),
                 float_equals("perimeter_overlap", 1.0),
                 float_equals("external_perimeter_overlap", 1.0) })
    }));


    // overhangs
    // The loop direction post-process reverses overhangs for every perimeter generator, unless perimeter_reverse
    // already reverses every loop.
    const GuiCondition can_reverse = all_of({ non_zero("perimeters"), true_("overhangs"), false_("perimeter_reverse") });
    add_rule({ "overhangs_reverse" }, can_reverse);
    add_rule({ "overhangs_reverse_threshold" },
             all_of({ can_reverse, true_("overhangs_reverse") }));
    add_rule({ "overhangs_speed_enforce" }, any_of({
        all_of({ non_zero("perimeters"), true_("overhangs"), false_("perimeter_loop") }),
        all_of({ non_zero("perimeters"), true_("overhangs"), int_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) })
    }));
    add_rule({ "overhangs_speed", "overhangs_width_speed", "overhangs_flow_ratio" },
             all_of({ non_zero("perimeters"), true_("overhangs") }));
    add_rule({ "overhangs_width", "overhangs_dynamic_flow" },
             all_of({ non_zero("perimeters"), true_("overhangs"), enabled("overhangs_flow_ratio") }));

    // perimeter details



    const GuiCondition seam_cost = all_of({ non_zero("perimeters"), int_equals("seam_position", int(spCost)) });
    add_rule({ "seam_angle_cost", "seam_travel_cost", "seam_visibility" }, seam_cost);
    add_rule({ "perimeter_loop_seam" }, any_of({
        all_of({ true_("perimeter_loop"), false_("perimeters") }),
        all_of({ true_("perimeter_loop"), int_not_equals("perimeter_generator", int(PerimeterGeneratorType::Arachne)) })
    }));
    add_rule({ "seam_notch_angle" }, any_of({
        all_of({ non_zero("perimeters"), non_zero("seam_notch_all") }),
        all_of({ non_zero("perimeters"), non_zero("seam_notch_inner") }),
        all_of({ non_zero("perimeters"), non_zero("seam_notch_outer") })
    }));

    // ----- gap fill -----------------------------------------------------
    // fuzzy skin
    add_rule({ "fuzzy_skin_thickness", "fuzzy_skin_point_dist" },
             int_not_equals("fuzzy_skin", int(FuzzySkinType::None)));

    // ----- infill -------------------------------------------------------
    add_rule({ "fill_aligned_z", "infill_connection",
               "solid_infill_every_layers", "solid_infill_below_area",
               "solid_infill_below_layer_area", "solid_infill_below_width",
               "infill_extruder" }, non_zero("fill_density"));
    add_rule({ "infill_anchor_max" },
             all_of({ non_zero("fill_density"), int_not_equals("infill_connection", int(icNotConnected)) }));
    add_rule({ "infill_anchor" }, all_of({
        non_zero("fill_density"),
        int_not_equals("infill_connection", int(icNotConnected)),
        non_zero("infill_anchor_max")
    }));

    // fill_density is a percent option: get_float() returns a fraction, so
    // the 50% threshold is 0.5.
    add_rule({ "infill_dense" }, float_less_than("fill_density", 0.5));
    add_rule({ "infill_dense_algo" }, all_of({
        true_("infill_dense"),
        float_less_than("fill_density", 0.5)
    }));
    add_rule({ "infill_every_layers", "infill_only_where_needed" }, any_of({
        all_of({ non_zero("fill_density"), false_("infill_dense") }),
        all_of({ non_zero("fill_density"), float_greater_or_equal("fill_density", 0.5) })
    }));

    add_rule({ "enforce_full_fill_volume", "external_infill_margin", "bridged_infill_margin",
               "solid_infill_extruder", "solid_infill_extrusion_width",
               "solid_infill_extrusion_spacing",
               "solid_infill_extrusion_change_odd_layers" }, has_solid);

    const GuiCondition has_infill_or_solid = any_of({ has_solid, non_zero("fill_density") });
    add_rule({ "infill_first" }, has_infill_or_solid);
    add_rule({ "fill_angle_cross", "fill_angle_follow_model", "fill_angle_increment",
               "fill_angle_template", "bridge_angle", "infill_extrusion_width",
               "infill_extrusion_spacing", "infill_extrusion_change_odd_layers" }, has_infill_or_solid);

    add_rule({ "fill_angle" },
             all_of({ has_infill_or_solid, vector_empty("fill_angle_template") }));

    add_rule({ "top_solid_min_thickness" }, any_of({
        all_of({ non_zero("top_solid_layers"), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("perimeters") }),
        all_of({ non_zero("top_solid_layers"), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("spiral_vase") }),
        all_of({ int_equals("solid_infill_every_layers", 1), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("perimeters") }),
        all_of({ int_equals("solid_infill_every_layers", 1), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("spiral_vase") })
    }));
    add_rule({ "bottom_solid_min_thickness" }, any_of({
        all_of({ non_zero("bottom_solid_layers"), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("perimeters") }),
        all_of({ non_zero("bottom_solid_layers"), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("spiral_vase") }),
        all_of({ int_equals("solid_infill_every_layers", 1), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("perimeters") }),
        all_of({ int_equals("solid_infill_every_layers", 1), int_not_equals("ensure_vertical_shell_thickness", int(EnsureVerticalShellThickness::Disabled)), false_("spiral_vase") })
    }));

    // solid / top / bottom surfaces
    add_rule({ "ironing", "infill_connection_top",
               "top_infill_extrusion_width", "top_infill_extrusion_spacing" }, has_top);
    add_rule({ "infill_connection_bottom" }, has_bottom);
    add_rule({"infill_connection_solid", "bridge_fill_pattern", "infill_connection_bridge"}, has_solid);

    const GuiCondition ironing_pp = all_of({ has_top, true_("ironing") });
    add_rule({ "ironing_type", "ironing_flowrate", "ironing_spacing", "ironing_angle" }, ironing_pp);

    const GuiCondition fill_smooth = any_of({
        all_of({ has_top, string_equals("top_fill_pattern", "smooth") }),
        all_of({ has_bottom, string_equals("bottom_fill_pattern", "smooth") }),
        all_of({ has_solid, string_equals("solid_fill_pattern", "smooth") }),
        all_of({ have_support, string_equals("support_material_top_interface_pattern", "smooth") }),
        all_of({ have_support, string_equals("support_material_bottom_interface_pattern", "smooth") })
    });
    add_rule({ "fill_smooth_width", "fill_smooth_distribution" }, fill_smooth);

    // ----- skirt and brim -----------------------------------------------
    add_rule({ "skirt_width", "skirt_brim", "skirt_distance", "skirt_distance_from_brim",
               "draft_shield", "min_skirt_length", "skirt_extrusion_width" }, non_zero("skirts"));
    add_rule({ "skirt_height" },
             all_of({ non_zero("skirts"), int_not_equals("draft_shield", int(dsEnabled)) }));

    const GuiCondition perim_or_brim = any_of({ non_zero("perimeters"), non_zero("brim_width"), non_zero("brim_width_interior") });
    add_rule({ "perimeter_extruder" }, perim_or_brim);
    add_rule({ "perimeter_extrusion_width", "perimeter_extrusion_spacing",
               "perimeter_extrusion_change_odd_layers" }, perim_or_brim);

    const GuiCondition support_or_skirt = any_of({ have_support, non_zero("skirts") });
    add_rule({ "support_material_extruder" }, support_or_skirt);

    const GuiCondition support_skirt_brim = any_of({ have_support, non_zero("skirts"), non_zero("brim_width"), non_zero("brim_width_interior") });
    add_rule({ "support_material_speed" }, support_skirt_brim);

    // ----- support material and raft ------------------------------------
    add_rule({ "support_material_style", "support_material_extrusion_width",
               "support_material_contact_distance_type", "support_material_xy_spacing",
               "support_material_contact_xy_spacing", "support_material_bottom_interface_layers",
               "support_material_interface_layers" }, have_support);

    add_rule({ "support_material_threshold" },
             all_of({ have_support, true_("support_material_auto") }));

    add_rule({ "support_material_bottom_contact_distance", "support_material_contact_distance" },
             all_of({ have_support,
                      int_not_equals("support_material_contact_distance_type", int(zdNone)) }));

    add_rule({ "support_material_closing_radius" }, all_of({ have_support, int_equals("support_material_style", int(SupportMaterialStyle::smsSnug)) }));

    add_rule({ "support_tree_angle", "support_tree_angle_slow", "support_tree_branch_diameter",
               "support_tree_branch_diameter_angle", "support_tree_branch_diameter_double_wall",
               "support_tree_tip_diameter", "support_tree_branch_distance", "support_tree_top_rate" },
             has_tree_family);
    // The overhang limit drives tree branch angles and the tolerated border of merged grid layers.
    add_rule({ "support_max_overhang" }, any_of({ has_tree_family, has_grid_merged }));

    add_rule({ "support_material_interface_spacing", "support_material_interface_extruder",
               "support_material_interface_speed",
               "support_material_interface_angle_increment",
               "support_material_bottom_interface_expansion" }, have_interface);
    // Interfaces are common to every support family.
    add_rule({ "support_material_interface_layer_height", "support_material_interface_angle",
               "support_material_bottom_interface_pattern",
               "support_material_interface_contact_loops" }, have_interface);
    add_rule({ "support_material_with_sheath", "support_material_pattern",
               "support_material_spacing", "support_material_angle",
               "support_material_angle_height" },
             without_organic_supports(have_support));
    // The layer height is the maximum slab height of the slab planner, used by every style.
    add_rule({ "support_material_layer_height" }, have_support);

    // Synchronization also retains object Z positions for non-soluble support, while contacts keep their separation gaps.
    // Only grid and snug expose it: the merged grid forces synchronization, while tree and organic use branch slopes.
    add_rule({ "support_material_synchronize_layers" }, all_of({ have_support, any_of({
        int_equals("support_material_style", int(SupportMaterialStyle::smsGrid)),
        int_equals("support_material_style", int(SupportMaterialStyle::smsSnug)) }) }));

    // The first layer of every support style and of the raft: its density and the expansion of its foot.
    add_rule({ "raft_first_layer_density", "raft_first_layer_expansion" }, has_support_foot);

    // raft
    add_rule({ "raft_contact_distance" }, all_of({
        non_zero("raft_layers"),
        int_not_equals("raft_contact_distance_type", int(zdNone))
    }));
    add_rule({ "raft_expansion", "first_layer_acceleration_over_raft",
               "first_layer_speed_over_raft", "raft_layer_height",
               "raft_interface_layer_height" }, non_zero("raft_layers"));

    // ----- extrusion widths ---------------------------------------------
    const GuiCondition default_width = any_of({
        all_of({ enabled("first_layer_extrusion_width"),
                 false_("first_layer_extrusion_width") }),
        all_of({ enabled("first_layer_infill_extrusion_width"),
                 false_("first_layer_infill_extrusion_width") }),
        all_of({ false_("perimeter_extrusion_width"), non_zero("perimeters") }),
        all_of({ false_("perimeter_extrusion_width"), non_zero("brim_width") }),
        all_of({ false_("perimeter_extrusion_width"), non_zero("brim_width_interior") }),
        all_of({ false_("external_perimeter_extrusion_width"), non_zero("perimeters") }),
        all_of({ false_("infill_extrusion_width"), has_infill_or_solid }),
        all_of({ false_("solid_infill_extrusion_width"), has_solid }),
        all_of({ false_("top_infill_extrusion_width"), has_top }),
        all_of({ false_("support_material_extrusion_width"), have_support }),
        all_of({ false_("skirt_extrusion_width"), non_zero("skirts") })
    });
    add_rule({ "extrusion_width", "extrusion_spacing" }, default_width);

    add_rule({ "first_layer_extrusion_spacing" }, enabled("first_layer_extrusion_width"));
    add_rule({ "first_layer_infill_extrusion_spacing" }, enabled("first_layer_infill_extrusion_width"));

    // ----- sequential printing, wipe tower, MMU -------------------------
    const GuiCondition by_object = string_equals("print_plan_builder", "ordering.plan_builder.by_object");
    const GuiCondition whole_objects = all_of({ by_object, float_equals("complete_objects_step", 0.) });
    add_rule({ "standby_temperature_delta" }, true_("ooze_prevention"));

    const GuiCondition tower_possible = false_("spiral_vase");
    add_rule({ "wipe_tower" }, tower_possible);
    add_rule({ "wipe_tower_per_object" }, false_("spiral_vase"));
    const GuiCondition wipe_ready = all_of({ tower_possible, any_of({ true_("wipe_tower"), true_("wipe_tower_per_object") }) });
    add_rule({ "wipe_tower_object_x", "wipe_tower_object_y" }, all_of({ wipe_ready,
        any_of({ whole_objects, true_("wipe_tower_per_object") }) }));
    add_rule({ "wipe_tower_extruder_toolchange" }, all_of({ wipe_ready, non_zero("wipe_tower_extruder") }));
    // A global tower is positioned for By layer or a nonzero object height step, not whole-object printing.
    add_rule({ "wipe_tower_x", "wipe_tower_y" }, all_of({ tower_possible, true_("wipe_tower"),
        any_of({ string_equals("print_plan_builder", "ordering.plan_builder.by_layer"), non_zero("complete_objects_step") }) }));
    add_rule({ "wipe_tower_width", "wipe_tower_rotation_angle",
               "wipe_tower_brim_width", "wipe_tower_cone_angle", "wipe_tower_extra_spacing",
               "wipe_tower_bridging", "wipe_tower_brim", "wipe_tower_no_sparse_layers",
               "single_extruder_multi_material_priming", "wipe_tower_speed",
               "wipe_tower_wipe_starting_speed", "wipe_tower_extrusion_width",
               "wipe_tower_rest_in_middle" }, wipe_ready);

    add_rule({ "mmu_segmented_region_interlocking_depth" },
             non_zero("mmu_segmented_region_max_width"));

    // ----- avoid crossing -----------------------------------------------
    add_rule({ "avoid_crossing_curled_overhangs" }, false_("avoid_crossing_perimeters"));
    add_rule({ "avoid_crossing_perimeters" }, false_("avoid_crossing_curled_overhangs"));
    add_rule({ "avoid_crossing_perimeters_max_detour", "avoid_crossing_not_first_layer",
               "avoid_crossing_top", "avoid_travel_island" },
             true_("avoid_crossing_perimeters"));
    add_rule({ "avoid_travel_island_weight" }, all_of({
        true_("avoid_crossing_perimeters"),
        true_("avoid_travel_island")
    }));
    add_rule({ "enforce_retract_first_layer" }, true_("only_retract_when_crossing_perimeters"));

    // ----- milling ------------------------------------------------------
    add_rule({ "milling_after_z", "milling_extra_size" },
             true_("milling_post_process"));

    // ----- accelerations -------------------------------------------------
    add_rule({ "perimeter_acceleration" },
             all_of({ non_zero("default_acceleration"), non_zero("perimeters") }));
    add_rule({ "support_material_acceleration" },
             all_of({ non_zero("default_acceleration"), any_of({ true_("support_material"), non_zero("raft_layers"), non_zero("brim_width"), non_zero("brim_width_interior"), non_zero("skirts") }) }));
    add_rule({ "support_material_interface_acceleration" }, all_of({ non_zero("default_acceleration"), have_interface }));

    // ----- speeds -------------------------------------------------------
    add_rule({ "default_speed" }, any_of({
        percent("perimeter_speed"),
        percent("solid_infill_speed"),
        percent("bridge_speed"),
        percent("support_material_speed")
    }));


}

void GuiRulesCore::run_impl(const plugin_run_context *) const
{
}

void register_gui_rules_core_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, GuiRulesCore::instance(orch).c_instance());
}

}} // namespace slic3r_api::GuiRulesCorePlugin
