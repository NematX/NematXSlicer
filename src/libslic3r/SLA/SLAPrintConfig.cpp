///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Lukáš Hejl @hejllukas, Tomáš Mészáros @tamasmeszaros, Oleksandra Iushchenko @YuSanka, Pavel Mikuš @Godrak, David Kocík @kocikdav, Enrico Turri @enricoturri1966, Filip Sykala @Jony01, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2023 Pedro Lamas @PedroLamas
///|/ Copyright (c) 2023 Mimoja @Mimoja
///|/ Copyright (c) 2020 - 2021 Sergey Kovalev @RandoMan70
///|/ Copyright (c) 2021 Niall Sheridan @nsheridan
///|/ Copyright (c) 2021 Martin Budden
///|/ Copyright (c) 2021 Ilya @xorza
///|/ Copyright (c) 2020 Paul Arden @ardenpm
///|/ Copyright (c) 2020 rongith
///|/ Copyright (c) 2019 Spencer Owen @spuder
///|/ Copyright (c) 2019 Stephan Reichhelm @stephanr
///|/ Copyright (c) 2018 Martin Loidl @LoidlM
///|/ Copyright (c) SuperSlicer 2018 Remi Durand @supermerill
///|/ Copyright (c) 2016 - 2017 Joseph Lenox @lordofhyphens
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2016 Vanessa Ezekowitz @VanessaE
///|/ Copyright (c) 2015 Alexander Rössler @machinekoder
///|/ Copyright (c) 2014 Petr Ledvina @ledvinap
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SLA/SLAPrintConfig.hpp"

#include <algorithm>

#include "PointUtils.hpp"

namespace Slic3r {


static t_config_enum_names enum_names_from_keys_map(const t_config_enum_values &enum_keys_map)
{
    t_config_enum_names names;
    int cnt = 0;
    for (const auto& kvp : enum_keys_map)
        cnt = std::max(cnt, kvp.second);
    cnt += 1;
    names.assign(cnt, "");
    for (const auto& kvp : enum_keys_map)
        names[kvp.second] = kvp.first;
    return names;
}

#define CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS(NAME, ...) \
    static const t_config_enum_values& enum_keys_map_##NAME() { \
        static const t_config_enum_values keys_map __VA_ARGS__; \
        return keys_map; \
    } \
    template<> const t_config_enum_values& ConfigOptionEnum<NAME>::get_enum_values() { return enum_keys_map_##NAME(); } \
    template<> const t_config_enum_names& ConfigOptionEnum<NAME>::get_enum_names() { \
        static const t_config_enum_names keys_names = enum_names_from_keys_map(enum_keys_map_##NAME()); \
        return keys_names; \
    }

CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS(SLADisplayOrientation, {
    { "landscape",      sladoLandscape},
    { "portrait",       sladoPortrait}
})
CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS(SLAPillarConnectionMode, {
    {"zigzag",          int(SLAPillarConnectionMode::zigzag)},
    {"cross",           int(SLAPillarConnectionMode::cross)},
    {"dynamic",         int(SLAPillarConnectionMode::dynamic)}
})
CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS(SLAMaterialSpeed, {
    {"slow",            slamsSlow},
    {"fast",            slamsFast},
    {"high_viscosity",  slamsHighViscosity}
})
CONFIG_OPTION_ENUM_DEFINE_STATIC_MAPS(SLASupportTreeType, {
    {"default", int(sla::SupportTreeType::Default)},
    {"branching",   int(sla::SupportTreeType::Branching)},
    //TODO: {"organic", int(sla::SupportTreeType::Organic)}
})

// Declare and initialize static caches of StaticPrintConfig derived classes.
#define PRINT_CONFIG_CACHE_ELEMENT_DEFINITION(r, data, CLASS_NAME) StaticPrintConfig::StaticCache<class Slic3r::CLASS_NAME> BOOST_PP_CAT(CLASS_NAME::s_cache_, CLASS_NAME);
#define PRINT_CONFIG_CACHE_ELEMENT_INITIALIZATION(r, data, CLASS_NAME) Slic3r::CLASS_NAME::initialize_cache();
#define PRINT_CONFIG_CACHE_INITIALIZE(CLASSES_SEQ) \
    BOOST_PP_SEQ_FOR_EACH(PRINT_CONFIG_CACHE_ELEMENT_DEFINITION, _, BOOST_PP_TUPLE_TO_SEQ(CLASSES_SEQ)) \
    int sla_print_config_static_initializer() { \
        /* Putting a trace here to avoid the compiler to optimize out this function. */ \
        /*BOOST_LOG_TRIVIAL(trace) << "Initializing StaticPrintConfigs";*/ \
        /* Tamas: alternative solution through a static volatile int. Boost log pollutes stdout and prevents tests from generating clean output */ \
        static volatile int ret = 1; \
        BOOST_PP_SEQ_FOR_EACH(PRINT_CONFIG_CACHE_ELEMENT_INITIALIZATION, _, BOOST_PP_TUPLE_TO_SEQ(CLASSES_SEQ)) \
        return ret; \
    }
PRINT_CONFIG_CACHE_INITIALIZE((
    SLAMaterialConfig, SLAPrintConfig, SLAPrintObjectConfig, SLAPrinterConfig, SLAFullPrintConfig))

void initialize_sla_print_config_cache()
{
    static const int initialized = sla_print_config_static_initializer();
    (void)initialized;
}

static Points to_points(const std::vector<Vec2d> &dpts)
{
    Points pts;
    pts.reserve(dpts.size());
    for (const Vec2d &v : dpts)
        pts.emplace_back(scale_i(v.x()), scale_i(v.y()));
    return pts;
}

Points get_bed_shape(const SLAPrinterConfig &cfg) { return to_points(cfg.bed_shape.get_values()); }

std::string get_sla_suptree_prefix(const DynamicPrintConfig &config)
{
    const auto *suptreetype = config.option<ConfigOptionEnum<sla::SupportTreeType>>("support_tree_type");
    std::string slatree = "";
    if (suptreetype) {
        auto ttype = static_cast<sla::SupportTreeType>(suptreetype->get_int());
        switch (ttype) {
        case sla::SupportTreeType::Branching: slatree = "branching"; break;
        case sla::SupportTreeType::Organic: slatree = "organic"; break;
        default:
            ;
        }
    }

    return slatree;
}

void init_sla_support_params(PrintConfigDef &definition, const std::string &prefix)
{
    ConfigOptionDef* def;

    def = definition.add(prefix + "support_head_front_diameter", coFloat, ptSLA);
    def->label = L("Pinhead front diameter");
    def->category = OptionCategory::support;
    def->tooltip = L("Diameter of the pointing side of the head");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.4));

    def = definition.add(prefix + "support_head_penetration", coFloat, ptSLA);
    def->label = L("Head penetration");
    def->category = OptionCategory::support;
    def->tooltip = L("How much the pinhead has to penetrate the model surface");
    def->sidetext = L("mm");
    def->mode = comAdvancedE | comPrusa;
    def->min = 0;
    def->set_default_value(new ConfigOptionFloat(0.2));

    def = definition.add(prefix + "support_head_width", coFloat, ptSLA);
    def->label = L("Pinhead width");
    def->category = OptionCategory::support;
    def->tooltip = L("Width from the back sphere center to the front sphere center");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 20;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add(prefix + "support_pillar_diameter", coFloat, ptSLA);
    def->label = L("Pillar diameter");
    def->category = OptionCategory::support;
    def->tooltip = L("Diameter in mm of the support pillars");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 15;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add(prefix + "support_small_pillar_diameter_percent", coPercent, ptSLA);
    def->label = L("Small pillar diameter percent");
    def->category = OptionCategory::support;
    def->tooltip = L("The percentage of smaller pillars compared to the normal pillar diameter "
                      "which are used in problematic areas where a normal pilla cannot fit.");
    def->sidetext = L("%");
    def->min = 1;
    def->max = 100;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionPercent(50));

    def = definition.add(prefix + "support_max_bridges_on_pillar", coInt, ptSLA);
    def->label = L("Max bridges on a pillar");
    def->tooltip = L(
        "Maximum number of bridges that can be placed on a pillar. Bridges "
        "hold support point pinheads and connect to pillars as small branches.");
    def->min = 0;
    def->max = 50;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionInt(prefix == "branching" ? 2 : 3));

    def = definition.add(prefix + "support_max_weight_on_model", coFloat, ptSLA);
    def->label = L("Max weight on model");
    def->category = OptionCategory::support;
    def->tooltip  = L(
        "Maximum weight of sub-trees that terminate on the model instead of the print bed. The weight is the sum of the lenghts of all "
        "branches emanating from the endpoint.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(10.));

    def = definition.add(prefix + "support_pillar_connection_mode", coEnum, ptSLA);
    def->label = L("Pillar connection mode");
    def->tooltip = L("Controls the bridge type between two neighboring pillars."
                            " Can be zig-zag, cross (double zig-zag) or dynamic which"
                            " will automatically switch between the first two depending"
                            " on the distance of the two pillars.");
    def->set_enum<SLAPillarConnectionMode>(
        ConfigOptionEnum<SLAPillarConnectionMode>::get_enum_names(),
        { L("Zig-Zag"), L("Cross"), L("Dynamic") });
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionEnum(SLAPillarConnectionMode::dynamic));

    def = definition.add(prefix + "support_buildplate_only", coBool, ptSLA);
    def->label = L("Support on build plate only");
    def->category = OptionCategory::support;
    def->tooltip = L("Only create support if it lies on a build plate. Don't create support on a print.");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));

    def = definition.add(prefix + "support_pillar_widening_factor", coFloat, ptSLA);
    def->label = L("Pillar widening factor");
    def->category = OptionCategory::support;

    def->tooltip  = 
        L("Merging bridges or pillars into another pillars can "
        "increase the radius. Zero means no increase, one means "
        "full increase. The exact amount of increase is unspecified and can "
        "change in the future.");

    def->min = 0;
    def->max = 1;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.5));

    def = definition.add(prefix + "support_base_diameter", coFloat, ptSLA);
    def->label = L("Support base diameter");
    def->category = OptionCategory::support;
    def->tooltip = L("Diameter in mm of the pillar base");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 30;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(4.0));

    def = definition.add(prefix + "support_base_height", coFloat, ptSLA);
    def->label = L("Support base height");
    def->category = OptionCategory::support;
    def->tooltip = L("The height of the pillar base cone");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add(prefix + "support_base_safety_distance", coFloat, ptSLA);
    def->label = L("Support base safety distance");
    def->category = OptionCategory::support;
    def->tooltip  = L(
        "The minimum distance of the pillar base from the model in mm. "
        "Makes sense in zero elevation mode where a gap according "
        "to this parameter is inserted between the model and the pad.");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 10;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1));

    def = definition.add(prefix + "support_critical_angle", coFloat, ptSLA);
    def->label = L("Critical angle");
    def->category = OptionCategory::support;
    def->tooltip = L("The default angle for connecting support sticks and junctions.");
    def->sidetext = L("°");
                    def->min = 0;
    def->max = 90;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(45));

    def = definition.add(prefix + "support_max_bridge_length", coFloat, ptSLA);
    def->label = L("Max bridge length");
    def->category = OptionCategory::support;
    def->tooltip = L("The max length of a bridge");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comAdvancedE | comPrusa;

    double default_val = 15.0;
    if (prefix == "branching")
        default_val = 5.0;

    def->set_default_value(new ConfigOptionFloat(default_val));

    def = definition.add(prefix + "support_max_pillar_link_distance", coFloat, ptSLA);
    def->label = L("Max pillar linking distance");
    def->category = OptionCategory::support;
    def->tooltip = L("The max distance of two pillars to get linked with each other."
                               " A zero value will prohibit pillar cascading.");
    def->sidetext = L("mm");
    def->min = 0;   // 0 means no linking
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(10.0));

    def = definition.add(prefix + "support_object_elevation", coFloat, ptSLA);
    def->label = L("Object elevation");
    def->category = OptionCategory::support;
    def->tooltip = L("How much the supports should lift up the supported object. "
                      "If \"Pad around object\" is enabled, this value is ignored.");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 150; // This is the max height of print on SL1
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(5.0));
}

void init_sla_params(PrintConfigDef &definition)
{
    definition.option_keys(RAW_PRESET_TYPE_SLA_MATERIAL_OVERRIDE).insert( {
        "branchingsupport_head_front_diameter",
        "branchingsupport_head_penetration",
        "branchingsupport_head_width",
        "branchingsupport_pillar_diameter",
        "sla_first_layer_size_compensation",
        "relative_correction_x",
        "relative_correction_y",
        "relative_correction_z",
        "support_head_front_diameter",
        "support_head_penetration",
        "support_head_width",
        "support_pillar_diameter",
        "support_points_density_relative",
    });

    definition.option_keys(RAW_PRESET_TYPE_SLA_PRINT).insert( {
        "print_version",
        "layer_height",
        "faded_layers",
        "print_custom_variables", // only for scripted widgets
        "supports_enable",
        "support_tree_type",

        "support_head_front_diameter",
        "support_head_penetration",
        "support_head_width",
        "support_pillar_diameter",
        "support_small_pillar_diameter_percent",
        "support_max_bridges_on_pillar",
        "support_max_weight_on_model",
        "support_pillar_connection_mode",
        "support_buildplate_only",
        "support_enforcers_only",
        "support_pillar_widening_factor",
        "support_base_diameter",
        "support_base_height",
        "support_base_safety_distance",
        "support_critical_angle",
        "support_max_bridge_length",
        "support_max_pillar_link_distance",
        "support_object_elevation",

        "branchingsupport_head_front_diameter",
        "branchingsupport_head_penetration",
        "branchingsupport_head_width",
        "branchingsupport_pillar_diameter",
        "branchingsupport_small_pillar_diameter_percent",
        "branchingsupport_max_bridges_on_pillar",
        "branchingsupport_max_weight_on_model",
        "branchingsupport_pillar_connection_mode",
        "branchingsupport_buildplate_only",
        "branchingsupport_pillar_widening_factor",
        "branchingsupport_base_diameter",
        "branchingsupport_base_height",
        "branchingsupport_base_safety_distance",
        "branchingsupport_critical_angle",
        "branchingsupport_max_bridge_length",
        "branchingsupport_max_pillar_link_distance",
        "branchingsupport_object_elevation",

        "support_points_density_relative",
        "support_points_minimal_distance",
        "slice_closing_radius",
        "slicing_mode",
        "pad_enable",
        "pad_wall_thickness",
        "pad_wall_height",
        "pad_brim_size",
        "pad_max_merge_distance",
        // "pad_edge_radius",
        "pad_wall_slope",
        "pad_object_gap",
        "pad_around_object",
        "pad_around_object_everywhere",
        "pad_object_connector_stride",
        "pad_object_connector_width",
        "pad_object_connector_penetration",
        "hollowing_enable",
        "hollowing_min_thickness",
        "hollowing_quality",
        "hollowing_closing_distance",
        "output_filename_format",
        "default_sla_print_profile",
        "compatible_printers",
        "compatible_printers_condition",
        "inherits"
    });

    definition.option_keys(RAW_PRESET_TYPE_SLA_MATERIAL).insert({
        "material_colour",
        "material_type",
        "initial_layer_height",
        "bottle_cost",
        "bottle_volume",
        "bottle_weight",
        "material_density",
        "filament_custom_variables", // only for scripted widgets
        "exposure_time",
        "initial_exposure_time",
        "material_correction",
        "material_correction_x",
        "material_correction_y",
        "material_correction_z",
        "material_notes",
        "material_vendor",
        "material_print_speed",
        "default_sla_material_profile",
        "compatible_prints", "compatible_prints_condition",
        "compatible_printers", "compatible_printers_condition", "inherits",

        // overriden options
        "material_ow_support_head_front_diameter",
        "material_ow_support_head_penetration",
        "material_ow_support_head_width",
        "material_ow_support_pillar_diameter",

        "material_ow_branchingsupport_head_front_diameter",
        "material_ow_branchingsupport_head_penetration",
        "material_ow_branchingsupport_head_width",
        "material_ow_branchingsupport_pillar_diameter",

        "material_ow_support_points_density_relative",

        "material_ow_relative_correction_x",
        "material_ow_relative_correction_y",
        "material_ow_relative_correction_z",
        "material_ow_sla_first_layer_size_compensation"
    });

    definition.option_keys(RAW_PRESET_TYPE_SLA_PRINTER).insert({
        "printer_technology",
        "bed_shape", "bed_custom_texture", "bed_custom_model", "max_print_height",
        "display_width", "display_height", "display_pixels_x", "display_pixels_y",
        "display_mirror_x", "display_mirror_y",
        "display_orientation",
        "fast_tilt_time", "slow_tilt_time", "high_viscosity_tilt_time", "area_fill",
        "relative_correction",
        "relative_correction_x",
        "relative_correction_y",
        "relative_correction_z",
        "absolute_correction",
        "sla_first_layer_size_compensation",
        "elephant_foot_min_width",
        "gamma_correction",
        "min_exposure_time", "max_exposure_time",
        "min_initial_exposure_time", "max_initial_exposure_time", "sla_output_precision",
        "output_format",
        "sla_output_precision",
        //FIXME the print host keys are left here just for conversion from the Printer preset to Physical Printer preset.
        "print_host", "printhost_apikey", "printhost_cafile", "printhost_port",
        "printer_custom_variables", // only for scripted widgets
        "printer_notes",
        "inherits",
        "thumbnails",
        "thumbnails_color",
        "thumbnail_color_mode",
        "thumbnails_with_bed",
        "thumbnails_format",
        "thumbnails_tag_format",
        "thumbnails_with_support",
    });


    ConfigOptionDef* def;

    // SLA Printer settings

    def = definition.add("display_width", coFloat, ptSLA);
    def->label = L("Display width");
    def->tooltip = L("Width of the display");
    def->min = 1;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(120.));

    def = definition.add("display_height", coFloat, ptSLA);
    def->label = L("Display height");
    def->tooltip = L("Height of the display");
    def->min = 1;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(68.));

    def = definition.add("display_pixels_x", coInt, ptSLA);
    def->full_label = L("Number of pixels in");
    def->label = L("X");
    def->tooltip = L("Number of pixels in X");
    def->min = 100;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionInt(2560));

    def = definition.add("display_pixels_y", coInt, ptSLA);
    def->label = L("Y");
    def->tooltip = L("Number of pixels in Y");
    def->min = 100;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionInt(1440));

    def = definition.add("display_mirror_x", coBool, ptSLA);
    def->full_label = L("Display horizontal mirroring");
    def->label = L("Mirror horizontally");
    def->tooltip = L("Enable horizontal mirroring of output images");
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionBool(true));

    def = definition.add("display_mirror_y", coBool, ptSLA);
    def->full_label = L("Display vertical mirroring");
    def->label = L("Mirror vertically");
    def->tooltip = L("Enable vertical mirroring of output images");
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));

    def = definition.add("display_orientation", coEnum, ptSLA);
    def->label = L("Display orientation");
    def->tooltip = L("Set the actual LCD display orientation inside the SLA printer."
                     " Portrait mode will flip the meaning of display width and height parameters"
                     " and the output images will be rotated by 90 degrees.");
    def->set_enum<SLADisplayOrientation>({
        { "landscape",  L("Landscape") },
        { "portrait",   L("Portrait") }
    });
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionEnum<SLADisplayOrientation>(sladoPortrait));

    def = definition.add("fast_tilt_time", coFloat, ptSLA);
    def->label = L("Fast");
    def->full_label = L("Fast tilt");
    def->tooltip = L("Time of the fast tilt");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(5.));

    def = definition.add("slow_tilt_time", coFloat, ptSLA);
    def->label = L("Slow");
    def->full_label = L("Slow tilt");
    def->tooltip = L("Time of the slow tilt");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(8.));

    def = definition.add("high_viscosity_tilt_time", coFloat, ptSLA);
    def->label = L("High viscosity");
    def->full_label = L("Tilt for high viscosity resin");
    def->tooltip = L("Time of the super slow tilt");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert;
    def->set_default_value(new ConfigOptionFloat(10.));

    def = definition.add("area_fill", coFloat, ptSLA);
    def->label = L("Area fill");
    def->tooltip = L("The percentage of the bed area. \nIf the print area exceeds the specified value, \nthen a slow tilt will be used, otherwise - a fast tilt");
    def->sidetext = L("%");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(50.));

    def = definition.add("relative_correction", coFloats, ptSLA);
    def->label = L("Printer scaling correction");
    def->full_label = L("Printer scaling correction");
    def->tooltip  = L("Printer scaling correction");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloats( { 1., 1.} ));

    def = definition.add("relative_correction_x", coFloat, ptSLA);
    def->label = L("Printer scaling correction in X axis");
    def->full_label = L("Printer scaling X axis correction");
    def->tooltip = L("Printer scaling correction in X axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("relative_correction_y", coFloat, ptSLA);
    def->label = L("Printer scaling correction in Y axis");
    def->full_label = L("Printer scaling Y axis correction");
    def->tooltip = L("Printer scaling correction in Y axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("relative_correction_z", coFloat, ptSLA);
    def->label = L("Printer scaling correction in Z axis");
    def->full_label = L("Printer scaling Z axis correction");
    def->tooltip = L("Printer scaling correction in Z axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("absolute_correction", coFloat, ptSLA);
    def->label = L("Printer absolute correction");
    def->full_label = L("Printer absolute correction");
    def->tooltip  = L("Will inflate or deflate the sliced 2D polygons according "
                      "to the sign of the correction.");
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.0));
    
    // Was first_layer_size_compensation, shared with FFF until the FFF setting
    // moved to the XY compensation plugin.
    def = definition.add("sla_first_layer_size_compensation", coFloat, ptSLA);
    def->label = L("First layer");
    def->full_label = L("First layer compensation");
    def->category = OptionCategory::slicing;
    def->tooltip = L("The first layer will be grown / shrunk in the XY plane by the configured value "
        "to compensate for the 1st layer squish aka an Elephant Foot effect. (should be negative = inwards = remove area)");
    def->sidetext = L("mm");
    def->mode = comAdvancedE | comSuSi | comPrusa; // just a rename & inverted of prusa 's elefant_foot
    def->set_default_value(new ConfigOptionFloat(0));

    def = definition.add("elephant_foot_min_width", coFloat, ptSLA);
    def->label = L("minimum width");
    def->category = OptionCategory::slicing;
    def->tooltip = L("Minimum width of features to maintain when doing the first layer compensation.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.2));

    def = definition.add("gamma_correction", coFloat, ptSLA);
    def->label = L("Printer gamma correction");
    def->full_label = L("Printer gamma correction");
    def->tooltip  = L("This will apply a gamma correction to the rasterized 2D "
                      "polygons. A gamma value of zero means thresholding with "
                      "the threshold in the middle. This behaviour eliminates "
                      "antialiasing without losing holes in polygons.");
    def->min = 0;
    def->max = 1;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));


    // SLA Material settings.

    def = definition.add("material_colour", coString, ptSLA);
    def->label = L("Color");
    def->tooltip = L("This is only used in the Slic3r interface as a visual help.");
    def->gui_type = ConfigOptionDef::GUIType::color;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionString("#29B2B2"));

    def = definition.add("material_type", coString, ptSLA);
    def->label = L("SLA material type");
    def->tooltip = L("SLA material type");
    def->gui_flags = "show_value";
    def->set_enum_values(ConfigOptionDef::GUIType::select_open,
        { "Tough", "Flexible", "Casting", "Dental", "Heat-resistant" });
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionString("Tough"));

    def = definition.add("initial_layer_height", coFloat, ptSLA);
    def->label = L("Initial layer height");
    def->tooltip = L("Initial layer height");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.3));

    def = definition.add("bottle_volume", coFloat, ptSLA);
    def->label = L("Bottle volume");
    def->tooltip = L("Bottle volume");
    def->sidetext = L("ml");
    def->min = 50;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1000.0));

    def = definition.add("bottle_weight", coFloat, ptSLA);
    def->label = L("Bottle weight");
    def->tooltip = L("Bottle weight");
    def->sidetext = L("kg");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add("material_density", coFloat, ptSLA);
    def->label = L("Density");
    def->tooltip = L("Density");
    def->sidetext = L("g/ml");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add("bottle_cost", coFloat, ptSLA);
    def->label = L("Cost");
    def->tooltip = L("Cost");
    def->sidetext = L("money/bottle");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.0));

    def = definition.add("faded_layers", coInt, ptSLA);
    def->label = L("Faded layers");
    def->tooltip = L("Number of the layers needed for the exposure time fade from initial exposure time to the exposure time");
    def->min = 3;
    def->max = 20;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionInt(10));

    def = definition.add("min_exposure_time", coFloat, ptSLA);
    def->label = L("Minimum exposure time");
    def->tooltip = L("Minimum exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0));

    def = definition.add("max_exposure_time", coFloat, ptSLA);
    def->label = L("Maximum exposure time");
    def->tooltip = L("Maximum exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(100));

    def = definition.add("exposure_time", coFloat, ptSLA);
    def->label = L("Exposure time");
    def->tooltip = L("Exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(10));

    def = definition.add("min_initial_exposure_time", coFloat, ptSLA);
    def->label = L("Minimum initial exposure time");
    def->tooltip = L("Minimum initial exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0));

    def = definition.add("max_initial_exposure_time", coFloat, ptSLA);
    def->label = L("Maximum initial exposure time");
    def->tooltip = L("Maximum initial exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(150));

    def = definition.add("initial_exposure_time", coFloat, ptSLA);
    def->label = L("Initial exposure time");
    def->tooltip = L("Initial exposure time");
    def->sidetext = L("s");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(15));

    def = definition.add("material_correction", coFloats, ptSLA);
    def->label = L("Correction for expansion");
    def->tooltip  = L("Correction for expansion");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloats({ 1., 1., 1. }));

    def = definition.add("material_correction_x", coFloat, ptSLA);
    def->full_label = L("Correction for expansion in X axis");
    def->tooltip = L("Correction for expansion in X axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("material_correction_y", coFloat, ptSLA);
    def->full_label = L("Correction for expansion in Y axis");
    def->tooltip = L("Correction for expansion in Y axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("material_correction_z", coFloat, ptSLA);
    def->full_label = L("Correction for expansion in Z axis");
    def->tooltip = L("Correction for expansion in Z axis");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("material_notes", coString, ptSLA);
    def->label = L("SLA print material notes");
    def->tooltip = L("You can put your notes regarding the SLA print material here.");
    def->multiline = true;
    def->full_width = true;
    def->height = 13;
    // TODO currently notes are the only way to pass data
    // for non-PrusaResearch printers. We therefore need to always show them 
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionString(""));

    def = definition.add("material_vendor", coString, ptSLA);
    def->set_default_value(new ConfigOptionString(L("(Unknown)")));
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("default_sla_material_profile", coString, ptSLA);
    def->nature = ConfigOptionNature::PresetMeta;
    def->label = L("Default SLA material profile");
    def->tooltip = L("Default print profile associated with the current printer profile. "
                   "On selection of the current printer profile, this print profile will be activated.");
    def->set_default_value(new ConfigOptionString());
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("sla_material_settings_id", coString, ptSLA);
    def->nature = ConfigOptionNature::Bookkeeping;
    def->set_default_value(new ConfigOptionString(""));
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("sla_material_settings_modified", coBool, ptSLA);
    def->nature = ConfigOptionNature::Bookkeeping;
    def->set_default_value(new ConfigOptionBool(false));
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("default_sla_print_profile", coString, ptSLA);
    def->nature = ConfigOptionNature::PresetMeta;
    def->label = L("Default SLA material profile");
    def->tooltip = L("Default print profile associated with the current printer profile. "
                   "On selection of the current printer profile, this print profile will be activated.");
    def->set_default_value(new ConfigOptionString());
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("sla_print_settings_modified", coBool, ptSLA);
    def->nature = ConfigOptionNature::Bookkeeping;
    def->set_default_value(new ConfigOptionBool(false));
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("sla_print_settings_id", coString, ptSLA);
    def->nature = ConfigOptionNature::Bookkeeping;
    def->set_default_value(new ConfigOptionString(""));
    def->cli = ConfigOptionDef::nocli;

    def = definition.add("supports_enable", coBool, ptSLA);
    def->label = L("Generate supports");
    def->category = OptionCategory::support;
    def->tooltip = L("Generate supports for the models");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(true));

    def = definition.add("support_tree_type", coEnum, ptSLA);
    def->label = L("Support tree type");
    def->tooltip = L("Support tree building strategy");
    def->set_enum<sla::SupportTreeType>(
        ConfigOptionEnum<sla::SupportTreeType>::get_enum_names(),
        { L("Default"),
    // TRN One of the "Support tree type"s on SLAPrintSettings : Supports
            L("Branching (experimental)") });
    // TODO: def->enum_def->labels[2] = L("Organic");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionEnum(sla::SupportTreeType::Default));

    init_sla_support_params(definition, "");
    init_sla_support_params(definition, "branching");

    def = definition.add("support_enforcers_only", coBool, ptSLA);
    def->label = L("Support only in enforced regions");
    def->category = OptionCategory::support;
    def->tooltip = L("Only create support if it lies in a support enforcer.");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));

    def = definition.add("support_points_density_relative", coInt, ptSLA);
    def->label = L("Support points density");
    def->category = OptionCategory::support;
    def->tooltip = L("This is a relative measure of support points density.");
    def->sidetext = L("%");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionInt(100));

    def = definition.add("support_points_minimal_distance", coFloat, ptSLA);
    def->label = L("Minimal distance of the support points");
    def->category = OptionCategory::support;
    def->tooltip = L("No support points will be placed closer than this threshold.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.));

    def = definition.add("pad_enable", coBool, ptSLA);
    def->label = L("Use pad");
    def->category = OptionCategory::pad;
    def->tooltip = L("Add a pad underneath the supported model");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(true));

    def = definition.add("pad_wall_thickness", coFloat, ptSLA);
    def->label = L("Pad wall thickness");
    def->category = OptionCategory::pad;
     def->tooltip = L("The thickness of the pad and its optional cavity walls.");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 30;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(2.0));

    def = definition.add("pad_wall_height", coFloat, ptSLA);
    def->label = L("Pad wall height");
    def->tooltip = L("Defines the pad cavity depth. Set zero to disable the cavity. "
                     "Be careful when enabling this feature, as some resins may "
                     "produce an extreme suction effect inside the cavity, "
                     "which makes peeling the print off the vat foil difficult.");
    def->category = OptionCategory::pad;
//     def->tooltip = L("");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 30;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.));
    
    def = definition.add("pad_brim_size", coFloat, ptSLA);
    def->label = L("Pad brim size");
    def->tooltip = L("How far should the pad extend around the contained geometry");
    def->category = OptionCategory::pad;
    //     def->tooltip = L("");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 30;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1.6));

    def = definition.add("pad_max_merge_distance", coFloat, ptSLA);
    def->label = L("Max merge distance");
    def->category = OptionCategory::pad;
     def->tooltip = L("Some objects can get along with a few smaller pads "
                      "instead of a single big one. This parameter defines "
                      "how far the center of two smaller pads should be. If they"
                      "are closer, they will get merged into one pad.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(50.0));

    // This is disabled on the UI. I hope it will never be enabled.
//    def = definition.add("pad_edge_radius", coFloat);
//    def->label = L("Pad edge radius");
//    def->category = OptionCategory::pad;
////     def->tooltip = L("");
//    def->sidetext = L("mm");
//    def->min = 0;
//    def->mode = comAdvancedE | comPrusa;
//    def->set_default_value(new ConfigOptionFloat(1.0));

    def = definition.add("pad_wall_slope", coFloat, ptSLA);
    def->label = L("Pad wall slope");
    def->category = OptionCategory::pad;
    def->tooltip = L("The slope of the pad wall relative to the bed plane. "
                     "90 degrees means straight walls.");
    def->sidetext = L("°");
    def->min = 45;
    def->max = 90;
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(90.0));

    def = definition.add("pad_around_object", coBool, ptSLA);
    def->label = L("Pad around object");
    def->category = OptionCategory::pad;
    def->tooltip = L("Create pad around object and ignore the support elevation");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));
    
    def = definition.add("pad_around_object_everywhere", coBool, ptSLA);
    def->label = L("Pad around object everywhere");
    def->category = OptionCategory::pad;
    def->tooltip = L("Force pad around object everywhere");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));

    def = definition.add("pad_object_gap", coFloat, ptSLA);
    def->label = L("Pad object gap");
    def->category = OptionCategory::pad;
    def->tooltip  = L("The gap between the object bottom and the generated "
                      "pad in zero elevation mode.");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 10;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(1));

    def = definition.add("pad_object_connector_stride", coFloat, ptSLA);
    def->label = L("Pad object connector stride");
    def->category = OptionCategory::pad;
    def->tooltip = L("Distance between two connector sticks which connect the object and the generated pad.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(10));

    def = definition.add("pad_object_connector_width", coFloat, ptSLA);
    def->label = L("Pad object connector width");
    def->category = OptionCategory::pad;
    def->tooltip  = L("Width of the connector sticks which connect the object and the generated pad.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.5));

    def = definition.add("pad_object_connector_penetration", coFloat, ptSLA);
    def->label = L("Pad object connector penetration");
    def->category = OptionCategory::pad;
    def->tooltip  = L(
        "How much should the tiny connectors penetrate into the model body.");
    def->sidetext = L("mm");
    def->min = 0;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.3));
    
    def = definition.add("hollowing_enable", coBool, ptSLA);
    def->label = L("Enable hollowing");
    def->category = OptionCategory::hollowing;
    def->tooltip = L("Hollow out a model to have an empty interior");
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionBool(false));
    
    def = definition.add("hollowing_min_thickness", coFloat, ptSLA);
    def->label = L("Wall thickness");
    def->category = OptionCategory::hollowing;
    def->tooltip  = L("Minimum wall thickness of a hollowed model.");
    def->sidetext = L("mm");
    def->min = 1;
    def->max = 10;
    def->mode = comSimpleAE | comPrusa;
    def->set_default_value(new ConfigOptionFloat(3.));
    
    def = definition.add("hollowing_quality", coFloat, ptSLA);
    def->label = L("Accuracy");
    def->category = OptionCategory::hollowing;
    def->tooltip  = L("Performance vs accuracy of calculation. Lower values may produce unwanted artifacts.");
    def->min = 0;
    def->max = 1;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.5));
    
    def = definition.add("hollowing_closing_distance", coFloat, ptSLA);
    def->label = L("Closing distance");
    def->category = OptionCategory::hollowing;
    def->tooltip  = L(
        "Hollowing is done in two steps: first, an imaginary interior is "
        "calculated deeper (offset plus the closing distance) in the object and "
        "then it's inflated back to the specified offset. A greater closing "
        "distance makes the interior more rounded. At zero, the interior will "
        "resemble the exterior the most.");
    def->sidetext = L("mm");
    def->min = 0;
    def->max = 10;
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(2.0));

    def = definition.add("material_print_speed", coEnum, ptSLA);
    def->label = L("Print speed");
    def->tooltip = L(
        "A slower printing profile might be necessary when using materials with higher viscosity "
        "or with some hollowed parts. It slows down the tilt movement and adds a delay before exposure.");
    def->set_enum<SLAMaterialSpeed>({
        { "slow",           L("Slow") },
        { "fast",           L("Fast") },
        { "high_viscosity", L("High viscosity") }
    });
    def->mode = comAdvancedE | comPrusa;
    def->set_default_value(new ConfigOptionEnum<SLAMaterialSpeed>(slamsFast));

    //def = definition.add("sla_archive_format", coString);
    //def->label = L("Format of the output SLA archive");
    //def->mode = comAdvancedE | comPrusa;
    //def->set_default_value(new ConfigOptionString("SL1"));

    def = definition.add("output_format", coEnum, ptSLA);
    def->label = L("Output Format");
    def->tooltip = L("Select the output format for this printer.");
    def->set_enum<OutputFormat>({
        {"SL1", L("Prusa SL1")},
        {"SL1_SVG", L("Prusa SL1 with SVG")},
        {"mCWS", L("Masked CWS")},
        {"AnyMono", L("Anycubic Mono")},
        {"AnyMonoX", L("Anycubic Mono X")},
        {"AnyMonoSE", L("Anycubic Mono SE")},
    });
    def->mode = comAdvancedE | comSuSi; // output_format should be preconfigured in profiles;
    def->set_default_value(new ConfigOptionEnum<OutputFormat>(ofSL1));

    def = definition.add("sla_output_precision", coFloat, ptSLA);
    def->label = L("SLA output precision");
    def->tooltip = L("Minimum resolution in nanometers");
    def->sidetext = L("mm");
    def->min = float(SCALING_FACTOR);
    def->mode = comExpert | comPrusa;
    def->set_default_value(new ConfigOptionFloat(0.001));

    // Declare retract values for material profile, overriding the print and printer profiles.
    for (const std::string &opt_key : definition.option_keys(RAW_PRESET_TYPE_SLA_MATERIAL_OVERRIDE)) {
        auto it_opt = definition.options.find(opt_key);
        assert(it_opt != definition.options.end());
        def = definition.add(std::string("material_ow_") + opt_key, it_opt->second.type, ptSLA);
        def->can_be_disabled = true;
        def->is_optional = true;
        def->label = it_opt->second.label;
        def->full_label = it_opt->second.full_label;
        def->tooltip = it_opt->second.tooltip;
        def->sidetext = it_opt->second.sidetext;
        def->min  = it_opt->second.min;
        def->max  = it_opt->second.max;
        def->mode = it_opt->second.mode;
        ConfigOption *default_opt = it_opt->second.default_value->clone();
        default_opt->set_can_be_disabled(true);
        def->set_default_value(default_opt);
        assert(!def->default_value->is_enabled());
    }
}


} // namespace Slic3r
