///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultBrimGenerator.hpp"
#include "libslic3r/NativeAdhesionLayerHelpers.hpp"
#include "AdhesionLayerHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_skirt_brim.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Brim.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"

/*
Default brim generator
======================

This plugin implements brim generation for STEP_SKIRT_BRIM. It reads first-layer
slices, brim settings and model patch volumes, builds temporary extrusion trees,
and publishes them in auxiliary layers tagged as brim. Geometry is supplied by
the helpers in Brim.hpp; the plugin decides which objects share a computation
and which object owns the published result.

Objects with identical brim settings can share global output. With
brim_per_object, each object owns its brim instead. Complete-object printing
uses one reusable pattern at the origin; normal printing generates positioned
output for every instance. Temporary changes to an object's instances are
restored by TemporaryObjectInstances when its scope ends, including on error.
Patch brim belongs to its source object even when normal brim is global.

The running occupied area prevents new brim from entering object slices,
negative patches or areas reserved by brim already generated. It is passed
between computations, so processing a later object must not discard the area
reserved by an earlier one.

Publication converts the centerlines into full-width coverage for the auxiliary
layer, then transfers the generated tree to that layer. Subsequent adhesion
consumers read its geometry and tag rather than infer its purpose from position.

Generated trees become owned by auxiliary layers. A per-copy brim
is moved back to object coordinates and marked for that physical instance;
sequential and parallel modes publish an unmarked shared local pattern.

Operation notes:
- generate_per_object_brim() / generate_global_brim()
- generate_brim_for_objects() :  classic brim geometry
- make_brim_patch() :  model patch volumes
- publish_object_brim() :  local geometry with its optional owner

The generation and publication branches are:

    generate_default_brim()
    |-- group_objects_by_brim_settings()
    |-- initial_unbrimmable_area() from first-layer slices and negative patches
    `-- walk compatible object groups; skip groups without outer/interior/patch work
        |-- generate_per_object_brim() for each object, when brim_per_object is set
        |   |-- complete_objects or parallel_objects_step > 0
        |   |   |-- temporarily select one origin instance and a local occupied area
        |   |   |-- generate_brim_for_objects() into an object-owned temporary tree
        |   |   |-- make_brim_patch() into the same tree
        |   |   `-- publish_object_brim() as one reusable origin pattern
        |   `-- otherwise, walk the object's saved instances
        |       |-- temporarily select one positioned instance
        |       |-- generate_brim_for_objects() with the shared occupied area
        |       |-- make_brim_patch() with that same occupied area
        |       |-- translate_adhesion_tree() back to object coordinates
        |       `-- publish_object_brim() with the original instance index
        `-- generate_global_brim(), otherwise
            |-- generate_brim_for_objects() for the compatible group
            |-- publish_brim_to_auxiliary_layer() on the print's auxiliary object
            |   `-- use a first layer from the group as the height/position reference
            `-- walk each object's instances for patch brim
                |-- make_brim_patch() with positioned patches and shared occupied area
                |-- translate_adhesion_tree() back to object coordinates
                `-- publish_object_brim() on the patch-owning object with its instance index

    generate_brim_for_objects()
    |-- when outer brim is enabled: make_brim_ears() or make_brim()
    `-- when interior brim is enabled: make_brim_interior()

    publish_object_brim()
    |-- skip an empty tree
    `-- publish_brim_to_auxiliary_layer() using the object's first layer

    publish_brim_to_auxiliary_layer()
    |-- brim_subject_from_extrusion() builds the full-width subject
    `-- publish_adhesion_extrusion_to_auxiliary_layer()
        `-- transfer the tree with a BRIM tag and reference-layer height/position
*/

namespace slic3r_api { namespace SkirtBrim { namespace DefaultBrimGeneratorPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_brim_generator_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace Slic3r;

const char *const k_no_dependencies[] = { nullptr };
const char *const k_group_id = "skirt_brim.brim";
const char *const k_settings_fragment_id = "skirt_brim.brim.default.settings";

const char *const k_defined_config_keys[] = {
    "brim_ears",
    "brim_ears_max_angle",
    "brim_ears_pattern",
    "brim_ears_detection_length",
    "brim_inside_holes",
    "brim_separation"
};

const key_value_string_pair_t k_brim_ear_patterns[] = {
    { "concentric", "Concentric" },
    { "rectilinear", "Rectilinear" }
};

const char *const k_brim_separation_aliases[] = { "brim_offset" };

const raw_used_config_key k_used_config_keys[] = {
    { "brim_width", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_width_interior", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_separation", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_inside_holes", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_per_object", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_ears", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_ears_max_angle", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_ears_pattern", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "brim_ears_detection_length", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_height", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "layer_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_direction", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "complete_objects", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "parallel_objects_step", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "arc_fitting", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "resolution_internal", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "raft_first_layer_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_interface_spacing", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "filament_max_overlap", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "support_material_interface_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

/*
Temporarily replace one object's instances while a legacy brim helper computes
a reusable brim pattern. The original instances are restored in the destructor,
so every early return or exception leaves PrintObject in the same state it had
before generation started.
*/
class TemporaryObjectInstances
{
public:
    explicit TemporaryObjectInstances(PrintObject &object);
    ~TemporaryObjectInstances();

    void set_single_origin_instance();
    void set_single_instance(const PrintInstance &instance);

private:
    PrintObject &m_object;
    PrintInstances m_saved_instances;
};

/*
Values owned by this plugin and resolved from one object's dynamic config.
Keeping them together makes the grouping comparison and the geometry call use
the same snapshot of configuration.
*/
struct DefaultBrimSettings
{
    bool use_ears = false;
    BrimGenerationParameters geometry;
};

// Read the five settings owned by this plugin from an object's API config view.
DefaultBrimSettings read_default_brim_settings(const PrintObject &object);

// Register the option definitions, layout fragment and activation rules.
void initialize_default_brim_settings(orchestrator_handle *orchestrator);

// Return true when an object has a model volume that explicitly adds/removes brim.
bool has_brim_patch(const PrintObject &object, ModelVolumeType brim_type);

// Return true when any object in a compatible group has the requested brim patch.
bool has_brim_patch(const std::vector<PrintObject *> &objects, ModelVolumeType brim_type);

// Group objects whose brim-affecting settings are identical.
std::vector<std::vector<PrintObject *>> group_objects_by_brim_settings(Slic3r::Print &print, bool &brim_per_object);

// Compare the optional first-layer extrusion width setting for grouping.
bool same_first_layer_extrusion_width(const PrintObjectConfig &lhs, const PrintObjectConfig &rhs);

// Compare all static and plugin-owned settings that change generated geometry.
bool same_brim_group_settings(const PrintObject &lhs, const PrintObject &rhs);

// Build the running occupied area used to keep generated brim away from objects and negative patches.
ExPolygons initial_unbrimmable_area(const std::vector<std::vector<PrintObject *>> &object_groups,
                                    bool brim_per_object);

// Convert centerline brim extrusions into the 2D subject owned by the auxiliary layer.
ExPolygons brim_subject_from_extrusion(const Slic3r::ExtrusionEntity &extrusion);

// Select the extrusion flow used by per-object brim generation.
Flow brim_flow_for_object(const Slic3r::Print &print, const PrintObject &object);

// Select the extrusion flow used by global brim generation.
Flow brim_flow_for_print(const Slic3r::Print &print);

// Append configured outer brim or ears, then interior brim, to the borrowed destination.
// Errors propagate and may leave prior output or forbidden-area updates in place.
void generate_brim_for_objects(const Slic3r::Print &print,
                               const Flow &flow,
                               const PrintObjectPtrs &objects,
                               ExPolygons &unbrimmable_area,
                               Slic3r::ExtrusionEntity &out);

// Cover borrowed brim paths in real-object local or global-auxiliary plate coordinates before moving the root into its owner's layer; optional provenance restricts a real instance, empty output returns false and geometry/allocation failures propagate.
bool publish_brim_to_auxiliary_layer(storage_handle *storage,
                                     orchestrator_handle *orchestrator,
                                     Slic3r::Print &print,
                                     const slic3r_api::Object &object,
                                     const Slic3r::Layer &reference_layer,
                                     Slic3r::ExtrusionEntity &brim,
                                     std::optional<uint32_t> owner_instance = std::nullopt);

// Borrow already-local brim and its source object; publish one layer with optional original instance index, moving extrusion ownership and throwing if the first layer or publication is unavailable.
void publish_object_brim(storage_handle *storage,
                         orchestrator_handle *orchestrator,
                         Slic3r::Print &print,
                         PrintObject &object,
                         Slic3r::ExtrusionEntity &brim,
                         std::optional<uint32_t> owner_instance = std::nullopt);

// Publish brim for each copy, or one shared local pattern in sequential/parallel mode.
// Patch projection uses a real model instance; temporary instance changes are restored if generation throws.
void generate_per_object_brim(storage_handle *storage,
                              orchestrator_handle *orchestrator,
                              Slic3r::Print &print,
                              PrintObject &object,
                              ExPolygons &unbrimmable_area);

// Generate the print-owned branch used when compatible objects share one global brim.
// Global geometry stays on the auxiliary object; each object patch is localized with its original owner index. Source instances stay unchanged and generation/publication failures propagate.
void generate_global_brim(storage_handle *storage,
                          orchestrator_handle *orchestrator,
                          Slic3r::Print &print,
                          const std::vector<PrintObject *> &object_group,
                          ExPolygons &unbrimmable_area);

// Generate all brim groups for one print.
void generate_default_brim(storage_handle *storage,
                           orchestrator_handle *orchestrator,
                           Slic3r::Print &print);

DefaultBrimSettings read_default_brim_settings(const PrintObject &object)
{
    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(&object));
    const Config config = object_view.config();

    DefaultBrimSettings settings;
    settings.use_ears = config.bool_or_default("brim_ears", false);
    settings.geometry.fill_enclosed_holes = config.bool_or_default("brim_inside_holes", false);
    settings.geometry.ear_max_angle_degrees = config.float_or_default("brim_ears_max_angle", 125.0);
    settings.geometry.ear_detection_length_mm = config.float_or_default("brim_ears_detection_length", 1.0);
    settings.geometry.ear_pattern = config.enum_or_default("brim_ears_pattern", 0) == 1 ?
        BrimEarPattern::Rectilinear : BrimEarPattern::Concentric;
    return settings;
}

void initialize_default_brim_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "brim_inside_holes";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Brim inside holes";
    def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
    def.tooltip = "Allow to create a brim over an island when it's inside a hole (or surrounded by an object)."
                  "\nIncompatible with brim_width_interior, as it enables it with brim_width width.";
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "brim_ears";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Brim ears";
    def.full_label = "Brim ears";
    def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
    def.tooltip = "Only draw brim over the sharp edges of the model.";
    def.mode = RAW_CONFIG_OPTION_MODE_SIM_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "brim_ears_max_angle";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Max angle";
    def.full_label = "Brim ear max angle";
    def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
    def.tooltip = "Maximum angle to let a brim ear appear. \nIf set to 0, no brim will be created. "
                  "\nIf set to ~178, brim will be created on everything but straight sections.";
    def.sidetext = "°";
    def.has_min = 1;
    def.min_value = 0.0;
    def.has_max = 1;
    def.max_value = 180.0;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "125";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "brim_ears_detection_length";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Detection radius";
    def.full_label = "Brim ear detection length";
    def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
    def.tooltip = "The geometry will be decimated before dectecting sharp angles. This parameter indicates "
                  "the minimum length of the deviation for the decimation.\n0 to deactivate";
    def.sidetext = "mm";
    def.has_min = 1;
    def.min_value = 0.0;
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "1";
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "brim_ears_pattern";
    def.type = RAW_CO_ENUM;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Pattern";
    def.full_label = "Ear pattern";
    def.category = RAW_OPTION_CATEGORY_INFILL;
    def.tooltip = "Pattern for the ear. The concentric is the default one. The rectilinear has a perimeter "
                  "around it, you can try it if the concentric has too many problems to stick to the build plate.";
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "concentric";
    def.enum_def.value_label_pairs.items = k_brim_ear_patterns;
    def.enum_def.value_label_pairs.count = uint32_t(std::size(k_brim_ear_patterns));
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "brim_separation";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_SUPPORT;
    def.category = RAW_OPTION_CATEGORY_SKIRT_BRIM;
    def.label = "Brim separation gap";
    def.tooltip = "Offset of brim from the printed object. Should be kept at 0 unless you encounter great difficulties to separate them."
                  "\nIt's subtracted to brim_width and brim_width_interior, so it has to be lower than them. The offset is applied after the first layer XY compensation (elephant foot).";
    def.sidetext = "mm";
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "0";
    def.aliases.items = k_brim_separation_aliases;
    def.aliases.size = uint32_t(std::size(k_brim_separation_aliases));
    orchestrator_create_option_def(orchestrator, &def);

    /*
    The fragment restores the same controls after they are removed from the
    static print.ui file. It also supplies the Simple-mode copy of brim_ears.
    */
    orchestrator_add_ui_fragment(
        orchestrator,
        "print.ui",
        k_settings_fragment_id,
        "page:Skirt & Brim\n"
        "group:Brim\n"
        "setting:insert$aftersetting$brim_width:brim_inside_holes\n"
        "line:insert$aftersetting$brim_width_interior:Brim ears\n"
        "setting:tags$Advanced$Expert$Prusa:label$_:sidetext_width$0:brim_ears\n"
        "setting:width$3:sidetext_width$1:brim_ears_max_angle\n"
        "setting:width$3:sidetext_width$3:brim_ears_detection_length\n"
        "setting:brim_ears_pattern\n"
        "end_line\n"
        "page:Support & Other\n"
        "group:Skirt & Brim\n"
        "setting:insert$aftersetting$brim_width:tags$Simple:sidetext_width$0:brim_ears\n",
        0);

    /*
    Registering a conjunction for one target mirrors the geometric
    preconditions without hardcoded option names in ConfigManipulation.
    */
    GuiRule::activate("brim_ears")
        .when_non_zero("brim_width")
        .add_to(orchestrator);
    GuiRule::activate("brim_inside_holes")
        .when_non_zero("brim_width")
        .when_false("brim_width_interior")
        .add_to(orchestrator);
    GuiRule::activate("brim_ears_max_angle")
        .when_true("brim_ears")
        .when_non_zero("brim_width")
        .add_to(orchestrator);
    GuiRule::activate("brim_ears_detection_length")
        .when_true("brim_ears")
        .when_non_zero("brim_width")
        .add_to(orchestrator);
    GuiRule::activate("brim_ears_pattern")
        .when_true("brim_ears")
        .when_non_zero("brim_width")
        .add_to(orchestrator);
    GuiRule::activate("brim_separation")
        .when(any_of({ non_zero("brim_width"), non_zero("brim_width_interior") }))
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator);
}

TemporaryObjectInstances::TemporaryObjectInstances(PrintObject &object) :
    m_object(object),
    m_saved_instances(ApiInternal::PrintObjectAccess::mutable_instances(object))
{
}

TemporaryObjectInstances::~TemporaryObjectInstances()
{
    ApiInternal::PrintObjectAccess::mutable_instances(m_object) = std::move(m_saved_instances);
}

void TemporaryObjectInstances::set_single_origin_instance()
{
    PrintInstances &instances = ApiInternal::PrintObjectAccess::mutable_instances(m_object);
    instances.clear();
    instances.emplace_back();
}

void TemporaryObjectInstances::set_single_instance(const PrintInstance &instance)
{
    PrintInstances &instances = ApiInternal::PrintObjectAccess::mutable_instances(m_object);
    instances.clear();
    instances.push_back(instance);
}

bool has_brim_patch(const PrintObject &object, const ModelVolumeType brim_type)
{
    for (const ModelVolume *volume : object.model_object()->volumes) {
        assert(volume != nullptr);
        if (volume != nullptr && volume->type() == brim_type)
            return true;
    }
    return false;
}

bool has_brim_patch(const std::vector<PrintObject *> &objects, const ModelVolumeType brim_type)
{
    for (const PrintObject *object : objects)
        if (object != nullptr && has_brim_patch(*object, brim_type))
            return true;
    return false;
}

std::vector<std::vector<PrintObject *>> group_objects_by_brim_settings(Slic3r::Print &print, bool &brim_per_object)
{
    std::vector<std::vector<PrintObject *>> object_groups;
    brim_per_object = false;

    for (PrintObject &object : print.objects()) {
        brim_per_object = brim_per_object || object.config().brim_per_object.value;
        bool added = false;
        for (std::vector<PrintObject *> &object_group : object_groups) {
            assert(!object_group.empty());
            if (same_brim_group_settings(*object_group.front(), object)) {
                object_group.push_back(&object);
                added = true;
                break;
            }
        }
        if (!added)
            object_groups.push_back({ &object });
    }

    return object_groups;
}

bool same_first_layer_extrusion_width(const PrintObjectConfig &lhs, const PrintObjectConfig &rhs)
{
    if (lhs.first_layer_extrusion_width.is_enabled() != rhs.first_layer_extrusion_width.is_enabled())
        return false;
    if (!lhs.first_layer_extrusion_width.is_enabled())
        return true;
    return lhs.first_layer_extrusion_width.value == rhs.first_layer_extrusion_width.value;
}

// Read one float setting; an absent plugin-owned setting reads as zero.
double brim_config_float(const PrintObjectConfig &config, const char *key)
{
    const ConfigOptionFloat *option = config.option<ConfigOptionFloat>(key);
    return option != nullptr ? option->value : 0.;
}

bool same_brim_group_settings(const PrintObject &lhs, const PrintObject &rhs)
{
    const PrintObjectConfig &lhs_config = lhs.config();
    const PrintObjectConfig &rhs_config = rhs.config();
    const DefaultBrimSettings lhs_settings = read_default_brim_settings(lhs);
    const DefaultBrimSettings rhs_settings = read_default_brim_settings(rhs);

    return lhs_settings.use_ears == rhs_settings.use_ears &&
           lhs_settings.geometry.fill_enclosed_holes == rhs_settings.geometry.fill_enclosed_holes &&
           lhs_settings.geometry.ear_max_angle_degrees == rhs_settings.geometry.ear_max_angle_degrees &&
           lhs_settings.geometry.ear_detection_length_mm == rhs_settings.geometry.ear_detection_length_mm &&
           lhs_settings.geometry.ear_pattern == rhs_settings.geometry.ear_pattern &&
           lhs_config.brim_per_object.value == rhs_config.brim_per_object.value &&
           brim_config_float(lhs_config, "brim_separation") == brim_config_float(rhs_config, "brim_separation") &&
           lhs_config.brim_width.value == rhs_config.brim_width.value &&
           lhs_config.brim_width_interior.value == rhs_config.brim_width_interior.value &&
           same_first_layer_extrusion_width(lhs_config, rhs_config);
}

ExPolygons initial_unbrimmable_area(const std::vector<std::vector<PrintObject *>> &object_groups,
                                    const bool brim_per_object)
{
    ExPolygons brim_area;
    bool needs_global_occupied_area =
        object_groups.size() > 1 || brim_per_object;
    if (!needs_global_occupied_area && !object_groups.empty()) {
        for (const std::vector<PrintObject *> &object_group : object_groups) {
            needs_global_occupied_area = (has_brim_patch(object_group, ModelVolumeType::BRIM_PATCH) ||
                                          has_brim_patch(object_group, ModelVolumeType::BRIM_NEGATIVE));
            if (needs_global_occupied_area) {
                break;
            }
        }
    }
    if (!needs_global_occupied_area)
        return brim_area;

    /*
    When object groups cannot share one brim, each group must treat every first
    layer slice that already exists on the bed as occupied. The copies are
    translated by instance shift because layer slices are stored in object-local
    coordinates.
    */
    for (const std::vector<PrintObject *> &object_group : object_groups) {
        for (const PrintObject *object : object_group) {
            if (object == nullptr)
                continue;
            if (object->layer_count() > 0) {
                const Slic3r::Layer &first_layer = object->layers().front();
                for (const PrintInstance &instance : object->instances()) {
                    const size_t first_idx = brim_area.size();
                    brim_area.insert(brim_area.end(), first_layer.lslices().begin(), first_layer.lslices().end());
                    for (size_t idx = first_idx; idx < brim_area.size(); ++idx)
                        brim_area[idx].translate(instance.shift.x(), instance.shift.y());
                }
            }

            /*
            Negative brim patches behave like occupied bed area: generated brim
            must keep away from them even though they are not real object slices.
            */
            if (has_brim_patch(*object, ModelVolumeType::BRIM_NEGATIVE)) {
                for (Slic3r::Polygon &polygon : object->get_brim_patch(ModelVolumeType::BRIM_NEGATIVE))
                    brim_area.push_back(Slic3r::ExPolygon(std::move(polygon)));
            }
        }
    }

    return brim_area;
}

ExPolygons brim_subject_from_extrusion(const Slic3r::ExtrusionEntity &extrusion)
{
    /*
    Brim paths are stored as centerlines, but auxiliary layers own printable
    area. Inflate every leaf by the width carried in its extrusion attributes,
    then union the polygons so region/island reconstruction receives a clean
    subject instead of a set of overlapping path covers.
    */
    Polygons coverage = extrusion.polygons_covered_by_width(float(SCALED_EPSILON));
    return coverage.empty() ? ExPolygons{} : union_ex(coverage);
}

Flow brim_flow_for_object(const Slic3r::Print &print, const PrintObject &object)
{
    std::set<extruder_index_t> extruders = print.object_extruders(PrintObjectPtrs{ const_cast<PrintObject *>(&object) });
    append(extruders, print.support_material_extruders());
    const int configured_extruder = print.print_region(0).config().perimeter_extruder.value;
    if (extruders.empty() && (configured_extruder <= 0 ||
        configured_extruder - 1 > (std::numeric_limits<extruder_index_t>::max)()))
        throw Slic3r::InvalidArgument("Brim flow resolved an invalid extruder setting");
    const extruder_index_t extruder_id = extruders.empty() ?
        static_cast<extruder_index_t>(configured_extruder - 1) : *extruders.begin();
    return print.brim_flow(extruder_id, object.config());
}

Flow brim_flow_for_print(const Slic3r::Print &print)
{
    std::set<extruder_index_t> extruders = print.object_extruders();
    append(extruders, print.support_material_extruders());
    const int configured_extruder = print.print_region(0).config().perimeter_extruder.value;
    if (extruders.empty() && (configured_extruder <= 0 ||
        configured_extruder - 1 > (std::numeric_limits<extruder_index_t>::max)()))
        throw Slic3r::InvalidArgument("Brim flow resolved an invalid extruder setting");
    const extruder_index_t extruder_id = extruders.empty() ?
        static_cast<extruder_index_t>(configured_extruder - 1) : *extruders.begin();
    return print.brim_flow(extruder_id, print.default_object_config());
}

void generate_brim_for_objects(const Slic3r::Print &print,
                               const Flow &flow,
                               const PrintObjectPtrs &objects,
                               ExPolygons &unbrimmable_area,
                               Slic3r::ExtrusionEntity &out)
{
    assert(!objects.empty());
    const PrintObjectConfig &brim_config = objects.front()->config();
    const DefaultBrimSettings brim_settings = read_default_brim_settings(*objects.front());

    if (brim_config.brim_width > 0) {
        if (brim_settings.use_ears)
            make_brim_ears(print, flow, objects, brim_settings.geometry, unbrimmable_area, out);
        else
            make_brim(print, flow, objects, brim_settings.geometry, unbrimmable_area, out);
    }

    if (brim_config.brim_width_interior > 0)
        make_brim_interior(print, flow, objects, unbrimmable_area, out);
}

void publish_object_brim(storage_handle *storage,
                         orchestrator_handle *orchestrator,
                         Slic3r::Print &print,
                         PrintObject &object,
                         Slic3r::ExtrusionEntity &brim,
                         std::optional<uint32_t> owner_instance)
{
    if (brim.empty())
        return;

    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(&object));
    if (object.layer_count() == 0 ||
        !publish_brim_to_auxiliary_layer(storage, orchestrator, print, object_view, object.layer(0), brim, owner_instance))
        throw RuntimeError("Default brim generator could not publish object brim auxiliary layer.");
}

bool publish_brim_to_auxiliary_layer(storage_handle *storage,
                                     orchestrator_handle *orchestrator,
                                     Slic3r::Print &print,
                                     const slic3r_api::Object &object,
                                     const Slic3r::Layer &reference_layer,
                                     Slic3r::ExtrusionEntity &brim,
                                     std::optional<uint32_t> owner_instance)
{
    if (storage == nullptr || brim.empty())
        return false;

    ExPolygons subject = brim_subject_from_extrusion(brim);
    if (subject.empty())
        return false;

    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::ExPolygonCollection subject_view(
        reinterpret_cast<const expolygon_collection_handle *>(&subject));

    return publish_adhesion_extrusion_to_auxiliary_layer(
        storage,
        orchestrator,
        print_view,
        object,
        subject_view,
        reference_layer.scaled_height(),
        reference_layer.scaled_print_z(),
        scale_to_layer_coord(reference_layer.slice_z),
        RAW_LAYER_ADHESION_KIND_BRIM,
        0,
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(&brim)), owner_instance);
}

void generate_per_object_brim(storage_handle *storage,
                              orchestrator_handle *orchestrator,
                              Slic3r::Print &print,
                              PrintObject &object,
                              ExPolygons &unbrimmable_area)
{
    const PrintObjectConfig &brim_config = object.config();
    const Flow flow = brim_flow_for_object(print, object);

    if (print_by_object(print.config())) {
        // PrintApply.cpp groups instances with identical transforms except for XY translation into one PrintObject.
        // Project the first real instance, then subtract its shift to obtain the shared local patch.
        const PrintInstance &first_instance = object.instances().front();
        Polygons patches = object.get_brim_patch(ModelVolumeType::BRIM_PATCH, &first_instance);
        for (Slic3r::Polygon &patch : patches)
            patch.translate(-first_instance.shift.x(), -first_instance.shift.y());
        /*
        Complete-object mode prints one full object at a time. The brim stored
        on the object is therefore a reusable pattern around an origin instance;
        placement happens later with the object instance transform.
        */
        ExPolygons local_unbrimmable_area;
        Slic3r::ExtrusionEntity object_brim(Slic3r::ExtrusionEntity::Children{}, true, true, false);
        TemporaryObjectInstances instances_guard(object);
        instances_guard.set_single_origin_instance();
        generate_brim_for_objects(print, flow, PrintObjectPtrs{ &object }, local_unbrimmable_area, object_brim);
        make_brim_patch(print, flow, patches, local_unbrimmable_area, object_brim);
        publish_object_brim(storage, orchestrator, print, object, object_brim);
        return;
    }

    unbrimmable_area = union_ex(unbrimmable_area);
    const PrintInstances copies = object.instances();
    TemporaryObjectInstances instances_guard(object);

    /*
    In normal multi-object mode, every copy lives in bed coordinates and must
    avoid every other occupied area. Generate one temporary tree per copy, then
    publish it as a local auxiliary brim layer restricted to that physical copy.
    */
    for (uint32_t index = 0; index < copies.size(); ++index) {
        const PrintInstance &instance = copies[index];
        const Point shift = instance.shift;
        instances_guard.set_single_instance(instance);
        Slic3r::ExtrusionEntity entity_brim(Slic3r::ExtrusionEntity::Children{}, true, true, false);
        generate_brim_for_objects(print, flow, PrintObjectPtrs{ &object }, unbrimmable_area, entity_brim);
        make_brim_patch(
            print, flow, object.get_brim_patch(ModelVolumeType::BRIM_PATCH), unbrimmable_area, entity_brim);
        Slic3r::translate_adhesion_tree(entity_brim, -shift);
        publish_object_brim(storage, orchestrator, print, object, entity_brim, index);
    }
}

void generate_global_brim(storage_handle *storage,
                          orchestrator_handle *orchestrator,
                          Slic3r::Print &print,
                          const std::vector<PrintObject *> &object_group,
                          ExPolygons &unbrimmable_area)
{
    unbrimmable_area = union_ex(unbrimmable_area);
    const Flow flow = brim_flow_for_print(print);
    Slic3r::ExtrusionEntity global_brim(Slic3r::ExtrusionEntity::Children{}, true, true, false);
    generate_brim_for_objects(print, flow, object_group, unbrimmable_area, global_brim);
    if (!global_brim.empty()) {
        const PrintObject *reference_object = nullptr;
        for (const PrintObject *object : object_group) {
            if (object != nullptr && object->layer_count() > 0) {
                reference_object = object;
                break;
            }
        }
        if (reference_object == nullptr)
            throw RuntimeError("Default brim generator could not find a first layer for global brim.");

        const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
        if (!publish_brim_to_auxiliary_layer(storage,
                                             orchestrator,
                                             print,
                                             print_view.auxiliary_object(),
                                             reference_object->layer(0),
                                             global_brim))
            throw RuntimeError("Default brim generator could not publish global brim auxiliary layer.");
    }

    /*
    Brim patch volumes are object-local features. Even when normal brim is
    shared globally, patch brim is generated per instance and stored with the
    object that owns the patch.
    */
    for (PrintObject *object : object_group) {
        assert(object != nullptr);
        if (object == nullptr)
            continue;
        for (uint32_t index = 0; index < object->instances().size(); ++index) {
            const PrintInstance &instance = object->instances()[index];
            const Point shift = instance.shift;
            Slic3r::ExtrusionEntity patch_brim(Slic3r::ExtrusionEntity::Children{}, true, true, false);
            make_brim_patch(
                print, flow, object->get_brim_patch(ModelVolumeType::BRIM_PATCH, &instance), unbrimmable_area, patch_brim);
            Slic3r::translate_adhesion_tree(patch_brim, -shift);
            publish_object_brim(storage, orchestrator, print, *object, patch_brim, index);
        }
    }
}

void generate_default_brim(storage_handle *storage,
                           orchestrator_handle *orchestrator,
                           Slic3r::Print &print)
{
    bool brim_per_object = false;
    std::vector<std::vector<PrintObject *>> object_groups =
        group_objects_by_brim_settings(print, brim_per_object);
    ExPolygons unbrimmable_area = initial_unbrimmable_area(object_groups, brim_per_object);

    for (std::vector<PrintObject *> &object_group : object_groups) {
        assert(!object_group.empty());
        const PrintObjectConfig &brim_config = object_group.front()->config();
        const bool has_work =
            brim_config.brim_width > 0 ||
            brim_config.brim_width_interior > 0 ||
            has_brim_patch(object_group, ModelVolumeType::BRIM_PATCH);
        if (!has_work)
            continue;

        if (brim_config.brim_per_object) {
            for (PrintObject *object : object_group) {
                assert(object != nullptr);
                if (object != nullptr)
                    generate_per_object_brim(storage, orchestrator, print, *object, unbrimmable_area);
            }
        } else {
            generate_global_brim(storage, orchestrator, print, object_group, unbrimmable_area);
        }
    }
}

class DefaultBrimGenerator : public PluginBase
{
public:
    static DefaultBrimGenerator &instance(orchestrator_handle *orchestrator)
    {
        static DefaultBrimGenerator instance(orchestrator);
        return instance;
    }

    explicit DefaultBrimGenerator(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "skirt_brim.brim.default"; }
    const char *name_impl() const noexcept override { return "Default brim generator"; }
    const char *description_impl() const noexcept override
    {
        return "Generates the classic first-layer brim geometry.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_id; }
    slicing_step_t step_impl() const noexcept override { return STEP_SKIRT_BRIM; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 0; }
    const char *progress_message_format_impl() const noexcept override { return "Generating brim"; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys == nullptr)
            return int32_t(std::size(k_used_config_keys));
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
                keys[idx] = k_defined_config_keys[idx];
        return int32_t(std::size(k_defined_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        initialize_default_brim_settings(m_orchestrator);
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_skirt_brim *ctx = plugin_ctx_as_skirt_brim(run_ctx);
        assert(ctx != nullptr);
        assert(ctx->print != nullptr);
        if (ctx == nullptr || ctx->print == nullptr)
            return;

        generate_default_brim(
            run_ctx->plugin_storage,
            m_orchestrator,
            *reinterpret_cast<Slic3r::Print *>(ctx->print));
    }
};

} // namespace

void register_default_brim_generator_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, DefaultBrimGenerator::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultBrimGenerator::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Brim generator";
    definition.tooltip = "Selects which plugin generates brim extrusion around first-layer objects.";
    DefaultBrimGenerator::instance(orchestrator).register_exclusive_group(STEP_SKIRT_BRIM, definition);
}

}}} // namespace slic3r_api::SkirtBrim::DefaultBrimGeneratorPlugin
