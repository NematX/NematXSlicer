#include "StepGenerateGcode.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>
#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_firmware.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Printing/PrintingPlanFacts.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

/*
STEP_GCODE bridge
=================

The G-code step is a plugin boundary. Earlier pipeline steps build a
PrintingPlan, then this step creates the selected firmware session and invokes
every selected STEP_GCODE plugin, one after the other, by increasing priority.
All of them receive the same payload: the source Print, read-only
PrintingPlan, borrowed firmware session, and resolved final output path. The
built-in writer (priority 0) creates and publishes that file; plugins of higher
priority may then modify it before the host Reader consumes it.

The first plugin error or cancellation aborts the step, which is not marked as
done; the next attempt rebuilds the file. Progress units are recorded only after
every plugin succeeded, and a failure of the step resets the whole progress table,
so the next attempt starts from a fresh one. The Reader pass that follows this step
is a further host progress unit (k_gcode_read_progress_unit), completed by the
pipeline after the read.

Before crossing that plugin boundary, the step validates the process speed of
every geometric leaf and explicit E-only extrusion command, including those in
before/after scope events. It also rejects printable roles which cannot be
converted safely to the legacy marker vocabulary. Stationary-volume DEPOSIT
leaves additionally need exactly one point, positive printable flow and dimensions,
and no extrusion-axis request. This is the last common
point where the host can describe an invalid tree with its complete
PrintingPlan location; a firmware session sees only one borrowed tree at a
time and cannot recover that context.

File format decisions still belong to the selected plugin, so external writers
can replace the built-in implementation without changing the pipeline runner.

Representative direct calls (branches are not a timeline):

    run_step()
        |-- validate_pre()
        |-- create_firmware_session()
        |-- Detail::prepare_guarded_plugin_host_context()
        |-- plugin->setup()
        |-- plugin->setup_run()
        `-- plugin->run()
*/

namespace Slic3r::Steps::StepGenerateGcode {

// Copy producer, firmware and thumbnail selection rules into the borrowed host; thumbnails are encoded only during export.
// Firmware and declared writer groups show their selectors even with one provider.
// Encoders read singleton ids strictly, keep their stable key and independent settings, and use Notes when placement is empty.
// Print data and provider ownership are unchanged; conflicts assert, and allocation errors propagate.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits output;
    output.category = RAW_OPTION_CATEGORY_OUTPUT;
    output.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    output.invalidates_step = STEP_GCODE;
    output.show_with_one_provider = true;
    const int32_t output_registered = orchestrator.register_builtin_step_selector(STEP_GCODE, output);
    assert(output_registered == 1);
    (void)output_registered;
    Orchestrator::ExclusiveGroupDefinition writer;
    writer.traits = output;
    writer.traits.exclusive_group = "step_gcode_plugin";
    writer.traits.option_key = "step_gcode_plugin";
    writer.label = "G-code plugin";
    writer.line_label = "G-code plugin";
    writer.tooltip = "Selects how STEP_GCODE creates the output artifact.";
    writer.translation_domain = "Slic3r";
    const int32_t writer_group_registered = orchestrator.register_builtin_exclusive_group(STEP_GCODE, std::move(writer));
    assert(writer_group_registered == 1);
    (void)writer_group_registered;
    const int32_t firmware_registered = orchestrator.register_builtin_step_selector(GCODE_FIRMWARE, output);
    assert(firmware_registered == 1);
    (void)firmware_registered;
    Orchestrator::ExclusiveGroupDefinition firmware;
    firmware.traits = std::move(output);
    firmware.traits.exclusive_group = "gcode_firmware_plugin";
    firmware.traits.option_key = "gcode_firmware_plugin";
    firmware.label = "G-code firmware";
    firmware.tooltip = "Selects the firmware session used by PrintingPlan output writers.";
    firmware.translation_domain = "Slic3r";
    const int32_t firmware_group_registered = orchestrator.register_builtin_exclusive_group(GCODE_FIRMWARE, std::move(firmware));
    assert(firmware_group_registered == 1);
    (void)firmware_group_registered;
    StepSelectorTraits thumbnail;
    thumbnail.category = RAW_OPTION_CATEGORY_OUTPUT;
    thumbnail.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    thumbnail.invalidates_step = STEP_NONE;
    thumbnail.read_with_one_provider = true;
    thumbnail.provider_settings_rules = false;
    thumbnail.ui_target_file = "print.ui";
    const int32_t thumbnail_registered = orchestrator.register_builtin_step_selector(GCODE_THUMBNAIL, thumbnail);
    assert(thumbnail_registered == 1);
    (void)thumbnail_registered;
    thumbnail.exclusive_group = "thumbnail_encoder_plugin";
    thumbnail.option_key = "thumbnails_format";
    thumbnail.strict_selection = true;
    Orchestrator::ExclusiveGroupDefinition encoder;
    encoder.traits = std::move(thumbnail);
    encoder.label = "Thumbnail encoding";
    encoder.tooltip = "Selects the encoding used for thumbnails embedded in G-code files.";
    encoder.translation_domain = "Slic3r";
    const int32_t encoder_registered = orchestrator.register_builtin_exclusive_group(GCODE_THUMBNAIL, std::move(encoder));
    assert(encoder_registered == 1);
    (void)encoder_registered;
}

namespace {

constexpr size_t MaximumReportedValidationErrors = 10;

/* One direct EPropertySpeed declaration encountered from a root to a leaf. */
struct SpeedDeclaration
{
    std::vector<size_t> child_path;
    float value = -1.f;
};

/*
State inherited while walking one extrusion tree.

Positive finite speeds replace the effective value. Non-positive values are
the public inheritance sentinel and therefore leave it unchanged. Non-finite
values are retained separately: even if another node provides a usable speed,
the malformed declaration must still be reported for every geometric leaf
whose path contains it.
*/
struct SpeedTraversalState
{
    std::optional<float> effective_speed;
    std::optional<raw_extrusion_role> effective_role;
    std::optional<ExtrusionAttributes> effective_attributes;
    bool has_extrusion_axis = false;
    std::vector<SpeedDeclaration> declarations;
    bool has_non_finite_speed = false;
};

/*
Machine tool selected while walking the complete PrintingPlan.

Requested speed is lexical and resets for each event or extrusion tree, but a
tool change is machine state and survives the end of that tree. The source text
is retained so an E-only command can explain whether its tool came from a tool
group boundary or an earlier semantic TOOLCHANGE command.
*/
struct ActiveToolState
{
    std::optional<extruder_index_t> tool_id;
    std::string source = "no tool has been selected yet";
};

/*
Location and source context for one independently serialized extrusion tree.

The firmware resets inherited requested state before every PrintingExtrusion
and every before/after event root. Keeping the root path here lets the same
walker validate both kinds while optional pointers describe only context that
actually exists at that scope.
*/
struct PrintingTreeLocation
{
    std::string root_path;
    const Printing::PrintingLayerGroup *layer_group = nullptr;
    const Printing::PrintingToolGroup *tool_group = nullptr;
    const Printing::PrintingExtrusion *extrusion = nullptr;
};

/*
Accumulate every failure count while retaining only a bounded number of full
diagnostics. A damaged plan may contain thousands of related leaves; the total
still tells the developer the blast radius without creating an unusable GUI
message or log entry.
*/
struct SpeedValidationReport
{
    explicit SpeedValidationReport(const Orchestrator &orchestrator) :
        runtime_orchestrator(reinterpret_cast<const orchestrator_handle *>(&orchestrator)) {}
    const orchestrator_handle *runtime_orchestrator;
    size_t error_count = 0;
    size_t speed_error_count = 0;
    size_t extrusion_e_only_error_count = 0;
    size_t scope_event_error_count = 0;
    size_t missing_root_count = 0;
    size_t unsupported_role_error_count = 0;
    size_t fan_error_count = 0;
    std::vector<std::string> details;

    void add_speed_error(std::string detail)
    {
        ++error_count;
        ++speed_error_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }

    void add_extrusion_e_only_error(std::string detail)
    {
        ++error_count;
        ++extrusion_e_only_error_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }

    void add_scope_event_error(std::string detail)
    {
        ++error_count;
        ++scope_event_error_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }

    void add_missing_root(std::string detail)
    {
        ++error_count;
        ++missing_root_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }

    void add_unsupported_role_error(std::string detail)
    {
        ++error_count;
        ++unsupported_role_error_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }

    void add_fan_error(std::string detail)
    {
        ++error_count;
        ++fan_error_count;
        if (details.size() < MaximumReportedValidationErrors)
            details.push_back(std::move(detail));
    }
};

/* Return a readable value while preserving NaN and infinity signs. */
std::string speed_value_text(float value);

/* Return the number of DefaultExtruder instances created by the firmware. */
size_t configured_extruder_count(const Print &print);

/* Return true when marker serialization can safely convert this raw role. */
bool is_supported_legacy_marker_role(raw_extrusion_role role);

/* Format the zero-based path used by PrintingPlan and the debugger. */
std::string extrusion_tree_path(const PrintingTreeLocation &location,
                                const std::vector<size_t> &child_path);

/* Describe the concrete tree node without relying on RTTI class names. */
const char *extrusion_entity_type(const ExtrusionEntity &entity);

/* Find a source PrintObject's zero-based position in Print::objects(). */
std::optional<size_t> print_object_index(const Print &print, const PrintObject *object);

/* Append plan and optional source-object metadata shared by all failures. */
void append_plan_source_context(std::ostringstream &out,
                                const Print &print,
                                const PrintingTreeLocation &location);

/*
Build the full diagnostic for one geometric leaf. The source pointers stored in
PrintingPlan are optional in synthetic tests, so every source-derived field has
an explicit unavailable form instead of dereferencing a missing context.
*/
std::string geometric_speed_error(const Print &print,
                                  const ExtrusionEntity &entity,
                                  const PrintingTreeLocation &location,
                                  const std::vector<size_t> &child_path,
                                  const SpeedTraversalState &state);

/* Explain a geometric role which has no safe legacy marker conversion. */
std::string unsupported_role_error(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   const std::vector<size_t> &child_path,
                                   raw_extrusion_role role);

/* Append every lexical speed declaration used to resolve one movement. */
void append_speed_declarations(std::ostringstream &out,
                               const PrintingTreeLocation &location,
                               const SpeedTraversalState &state);

/* Explain every invalid input of one explicit E-only extrusion command. */
std::string e_only_extrusion_error(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   const std::vector<size_t> &child_path,
                                   const SpeedTraversalState &state,
                                   const ActiveToolState &active_tool,
                                   size_t extruder_count,
                                   double extrusion_delta);

/* Validate inherited speeds, tools and stationary-volume leaf requirements in one tree. */
// Borrow one plan tree and its source location, carrying lexical attributes and
// speed into children and persistent tool selection into following roots. Append
// diagnostics for malformed movements without changing geometry or throwing for
// validation failures; the caller owns the tree, traversal path and report.
void validate_extrusion_tree_movements(const Print &print,
                                       const ExtrusionEntity &entity,
                                       const PrintingTreeLocation &location,
                                       std::vector<size_t> &child_path,
                                       const SpeedTraversalState &parent_state,
                                       ActiveToolState &active_tool,
                                       size_t extruder_count,
                                       SpeedValidationReport &report);

/* Validate one scope-event root at its exact position in serialization order. */
void validate_scope_event_movements(const Print &print,
                                    const ExtrusionEntity &event_root,
                                    const std::string &root_path,
                                    const Printing::PrintingLayerGroup *layer_group,
                                    const Printing::PrintingToolGroup *tool_group,
                                    ActiveToolState &active_tool,
                                    size_t extruder_count,
                                    SpeedValidationReport &report);

/* Convert the bounded report into the public validate_pre() diagnostic. */
std::string speed_validation_message(const SpeedValidationReport &report);

/*
Own the raw firmware session returned through the plugin ABI.

The provider allocates the session, but STEP_GCODE may fail before or during
file publication. Keeping ownership in this stack object guarantees that the
provider destructor is called once on every path.
*/
class FirmwareInstanceOwner
{
public:
    FirmwareInstanceOwner() = default;
    ~FirmwareInstanceOwner();

    FirmwareInstanceOwner(const FirmwareInstanceOwner &) = delete;
    FirmwareInstanceOwner &operator=(const FirmwareInstanceOwner &) = delete;

    raw_gcode_firmware_instance *mutable_instance();
    const raw_gcode_firmware_instance *instance() const;

private:
    raw_gcode_firmware_instance m_instance = {};
};

/*
Receive the writer's text source path through the C payload.

The receiver lives on the STEP_GCODE stack, so the callback cannot outlive the
Print it updates. It accepts one publication and copies the borrowed UTF-8
text immediately; opening and parsing the file belongs to the following host
phase.
*/
class ReaderSourcePathReceiver
{
public:
    explicit ReaderSourcePathReceiver(Print &print);
    static int32_t publish_callback(void *context, const char *path) noexcept;

private:
    int32_t publish(const char *path) noexcept;

    Print &m_print;
    bool m_published = false;
};

// Select the GCODE_FIRMWARE provider from the print configuration and ask it
// to create a session for this export.
// Supply the Print and script processor as borrowed inputs.
// Store the returned instance in owner, which is responsible for destroying it,
// and validate its C interface before the STEP_GCODE writer uses it.
void create_firmware_session(Orchestrator &orchestrator,
                             Print &print,
                             const raw_gcode_script_processor *script_processor,
                             FirmwareInstanceOwner &owner);

// Store the plan-derived print facts (tool changes, initial and used tools with their filament types, layer counts) in the
// Print statistics. Expects the finalized PrintingPlan of the Print; leaves every measured value untouched.
void publish_plan_facts(Print &print);

ReaderSourcePathReceiver::ReaderSourcePathReceiver(Print &print) : m_print(print) {}

int32_t ReaderSourcePathReceiver::publish_callback(void *context, const char *path) noexcept
{
    ReaderSourcePathReceiver *receiver = static_cast<ReaderSourcePathReceiver *>(context);
    return receiver == nullptr ? 0 : receiver->publish(path);
}

int32_t ReaderSourcePathReceiver::publish(const char *path) noexcept
{
    if (m_published || path == nullptr || path[0] == '\0')
        return 0;
    try {
        m_print.set_gcode_reader_source_path(path);
        m_published = true;
        return 1;
    } catch (...) {
        return 0;
    }
}

std::string speed_value_text(const float value)
{
    if (std::isnan(value))
        return "NaN";
    if (std::isinf(value))
        return value > 0.f ? "+infinity" : "-infinity";

    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << value;
    return out.str();
}

size_t configured_extruder_count(const Print &print)
{
    const PrintConfig &config = print.config();
    return std::max<size_t>({
        1,
        config.nozzle_diameter.size(),
        config.filament_diameter.size(),
        config.extruder_offset.size(),
        config.extrusion_multiplier.size()
    });
}

bool is_supported_legacy_marker_role(const raw_extrusion_role role)
{
    // Process-only roles never request a TYPE marker and therefore keep their
    // existing firmware behavior without entering the legacy converter.
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) ||
        RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
        RAW_EXTRUSION_ROLE_IS_RETRACT(role) ||
        RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
        return true;

    constexpr uint32_t known_mask = (uint32_t(1) << 16) - 1;
    constexpr uint32_t base_mask =
        uint32_t(RAW_EXTRUSION_ROLE_PERIMETER) |
        uint32_t(RAW_EXTRUSION_ROLE_INFILL) |
        uint32_t(RAW_EXTRUSION_ROLE_SUPPORT) |
        uint32_t(RAW_EXTRUSION_ROLE_SKIRT) |
        uint32_t(RAW_EXTRUSION_ROLE_WIPE_TOWER) |
        uint32_t(RAW_EXTRUSION_ROLE_TRAVEL) |
        uint32_t(RAW_EXTRUSION_ROLE_MILL) |
        uint32_t(RAW_EXTRUSION_ROLE_MIXED);
    const uint32_t bits = static_cast<uint32_t>(role);
    const uint32_t base = bits & base_mask;
    if ((bits & ~known_mask) != 0 ||
        (bits != 0 && (base == 0 || (base & (base - 1)) != 0)))
        return false;

    // The legacy converter accepts all overhang-perimeter and wipe-tower
    // refinements. Other printable roles have one exact canonical bit shape.
    if (base == RAW_EXTRUSION_ROLE_WIPE_TOWER ||
        (base == RAW_EXTRUSION_ROLE_PERIMETER &&
         RAW_EXTRUSION_ROLE_IS_BRIDGE(role)))
        return true;

    switch (role) {
    case RAW_EXTRUSION_ROLE_NONE:
    case RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER:
    case RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER:
    case RAW_EXTRUSION_ROLE_INTERNAL_INFILL:
    case RAW_EXTRUSION_ROLE_SOLID_INFILL:
    case RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL:
    case RAW_EXTRUSION_ROLE_IRONING_INFILL:
    case RAW_EXTRUSION_ROLE_BRIDGE_INFILL:
    case RAW_EXTRUSION_ROLE_INTERNAL_BRIDGE_INFILL:
    case RAW_EXTRUSION_ROLE_GAP_FILL:
    case RAW_EXTRUSION_ROLE_THIN_WALL:
    case RAW_EXTRUSION_ROLE_SKIRT:
    case RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL:
    case RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL_INTERFACE:
    case RAW_EXTRUSION_ROLE_MILLING:
        return true;
    default:
        return false;
    }
}

std::string extrusion_tree_path(const PrintingTreeLocation &location,
                                const std::vector<size_t> &child_path)
{
    std::ostringstream out;
    out << location.root_path;
    for (const size_t child_idx : child_path)
        out << ".children[" << child_idx << "]";
    return out.str();
}

const char *extrusion_entity_type(const ExtrusionEntity &entity)
{
    if (entity.child_count() > 0 && entity.is_loop())
        return "loop";
    if (entity.child_count() > 0 && entity.is_continuous())
        return "multipath";
    if (entity.child_count() > 0)
        return "collection";
    if (entity.is_nop())
        return "nop";
    if (entity.polyline_or_null() != nullptr)
        return "path";
    return "empty";
}

std::optional<size_t> print_object_index(const Print &print, const PrintObject *object)
{
    if (object == nullptr)
        return std::nullopt;
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx)
        if (&print.objects()[object_idx] == object)
            return object_idx;
    return std::nullopt;
}

void append_plan_source_context(std::ostringstream &out,
                                const Print &print,
                                const PrintingTreeLocation &location)
{
    out << "Plan context: print_z=";
    if (location.layer_group != nullptr)
        out << std::setprecision(12) << unscaled(location.layer_group->print_z) << " mm";
    else
        out << "unavailable";
    out << ", extruder_id=";
    if (location.tool_group != nullptr)
        out << location.tool_group->extruder_id;
    else
        out << "unavailable";
    out << ", source_role=";
    if (location.extrusion != nullptr)
        out << role_to_code(location.extrusion->sregion_island_role);
    else
        out << "unavailable (scope event)";
    out << "\n";

    if (location.extrusion == nullptr) {
        out << "Source context: unavailable for a before/after scope event\n";
        return;
    }

    const Printing::PrintingExtrusion &extrusion = *location.extrusion;
    const LayerRegionIsland *region_island = extrusion.region_island;
    const LayerRegion *source_region = region_island != nullptr && !region_island->regions().empty() ?
        *region_island->regions().begin() : nullptr;
    const Layer *source_layer = source_region != nullptr ? source_region->layer() : nullptr;
    const PrintObject *source_object = source_layer != nullptr ? source_layer->object() : nullptr;
    const ModelObject *model_object = source_object != nullptr ? source_object->model_object() : nullptr;
    const std::optional<size_t> source_object_idx = print_object_index(print, source_object);

    out << "Source context: ";
    if (region_island == nullptr) {
        out << "region_island=unavailable, layer=unavailable, object=unavailable";
    } else {
        out << "region_count=" << region_island->regions().size()
            << ", island_extruder_id=" << region_island->extruder_id();
        if (source_layer != nullptr)
            out << ", layer_id=" << source_layer->id();
        else
            out << ", layer=unavailable";
        if (source_object != nullptr) {
            out << ", print_object_index=";
            if (source_object_idx)
                out << *source_object_idx;
            else
                out << "unavailable";
            out << ", object_name="
                << std::quoted(model_object != nullptr ? model_object->name : std::string("<unnamed>"));
        } else {
            out << ", object=unavailable";
        }
    }
    out << ", printing_catalog_id=" << extrusion.printing_catalog_id;
    const Printing::PrintingPlan *plan = print.printing_plan();
    if (extrusion.printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE) {
        const Printing::PrintingObjectInstance *instance =
            plan == nullptr ? nullptr : plan->object_instance(extrusion.printing_catalog_id);
        if (instance == nullptr) {
            out << ", catalog_instance=invalid";
        } else if (instance->object != nullptr) {
            out << ", catalog_instance_idx=" << instance->object_instance_idx;
            if (source_object != nullptr && instance->object != source_object)
                out << " (invalid; catalog object differs from source object)";
            else if (source_object != nullptr &&
                     instance->object_instance_idx >= source_object->instances().size())
                out << " (invalid; object has " << source_object->instances().size()
                    << " instances)";
        } else {
            out << ", catalog_instance=synthetic";
        }
    }
    out << "\n";
}

std::string geometric_speed_error(const Print &print,
                                  const ExtrusionEntity &entity,
                                  const PrintingTreeLocation &location,
                                  const std::vector<size_t> &child_path,
                                  const SpeedTraversalState &state)
{
    std::ostringstream out;
    out << "PrintingPlan location: " << extrusion_tree_path(location, child_path) << "\n";
    append_plan_source_context(out, print, location);

    const ArcPolyline *polyline = entity.polyline_or_null();
    const size_t point_count = polyline != nullptr ? polyline->size() : 0;
    const size_t segment_count = point_count > 0 ? point_count - 1 : 0;
    out << "Entity: id=" << entity.get_id() << ", type=" << extrusion_entity_type(entity)
        << ", points=" << point_count << ", segments=" << segment_count
        << ", length=" << std::setprecision(12) << unscaled(entity.length()) << " mm";
    if (polyline != nullptr && !polyline->empty()) {
        out << ", start=(" << unscaled(polyline->front().x()) << ", "
            << unscaled(polyline->front().y());
        if (polyline->has_z_offset())
            out << ", z_offset=" << unscaled(polyline->z_offset(0));
        out << ") mm, end=(" << unscaled(polyline->back().x()) << ", "
            << unscaled(polyline->back().y());
        if (polyline->has_z_offset())
            out << ", z_offset=" << unscaled(polyline->z_offset(point_count - 1));
        out << ") mm, contains_arcs=" << (polyline->has_arc() ? "yes" : "no");
    }
    if (state.effective_role)
        out << ", effective_role=" << *state.effective_role;
    else
        out << ", effective_role=unavailable";
    out << ", movement=geometric";
    out << "\n";

    append_speed_declarations(out, location, state);

    if (state.has_non_finite_speed)
        out << "Reason: a speed declaration on this path is NaN or infinite."
               " Every explicit speed must be finite.\n";
    else if (state.declarations.empty())
        out << "Reason: neither this geometric leaf nor any parent declares a speed.\n";
    else
        out << "Reason: every speed declaration on this path requests inheritance,"
               " but no parent provides a finite positive speed.\n";
    return out.str();
}

std::string unsupported_role_error(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   const std::vector<size_t> &child_path,
                                   const raw_extrusion_role role)
{
    std::ostringstream out;
    out << "PrintingPlan location: " << extrusion_tree_path(location, child_path) << "\n";
    append_plan_source_context(out, print, location);
    out << "Entity: id=" << entity.get_id()
        << ", type=" << extrusion_entity_type(entity) << "\n"
        << "Reason: unsupported effective extrusion role raw=" << role
        << " has no safe legacy G-code marker conversion.\n";
    return out.str();
}

void append_speed_declarations(std::ostringstream &out,
                               const PrintingTreeLocation &location,
                               const SpeedTraversalState &state)
{
    out << "Speed declarations:";
    if (state.declarations.empty()) {
        out << " none on the leaf or its parents\n";
    } else {
        out << "\n";
        for (const SpeedDeclaration &declaration : state.declarations)
            out << "  - " << extrusion_tree_path(location, declaration.child_path)
                << ": speed_mm_per_s=" << speed_value_text(declaration.value)
                << (std::isfinite(declaration.value) && declaration.value <= 0.f ? " (inherit)" : "")
                << "\n";
    }
}

std::string e_only_extrusion_error(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   const std::vector<size_t> &child_path,
                                   const SpeedTraversalState &state,
                                   const ActiveToolState &active_tool,
                                   const size_t extruder_count,
                                   const double extrusion_delta)
{
    std::ostringstream out;
    out << "PrintingPlan location: " << extrusion_tree_path(location, child_path) << "\n";
    append_plan_source_context(out, print, location);
    out << "Entity: id=" << entity.get_id() << ", type=" << extrusion_entity_type(entity)
        << ", movement=E-only special extrusion, extrusion_delta_mm="
        << std::setprecision(std::numeric_limits<double>::max_digits10) << extrusion_delta << "\n";
    out << "Active tool: ";
    if (active_tool.tool_id)
        out << *active_tool.tool_id;
    else
        out << "unavailable";
    out << ", source=" << active_tool.source
        << ", configured_tool_count=" << extruder_count << "\n";
    append_speed_declarations(out, location, state);

    out << "Reason:";
    if (!std::isfinite(extrusion_delta))
        out << " the requested E delta is NaN or infinite;";
    if (!active_tool.tool_id)
        out << " no active tool is available;";
    else if (*active_tool.tool_id >= extruder_count)
        out << " active tool " << *active_tool.tool_id << " is outside the configured range;";
    if (state.has_non_finite_speed)
        out << " a speed declaration on this path is NaN or infinite;";
    else if (std::isfinite(extrusion_delta) && extrusion_delta != 0.0 &&
             (!state.effective_speed || !std::isfinite(*state.effective_speed) ||
              *state.effective_speed <= 0.f)) {
        if (state.declarations.empty())
            out << " neither this E-only event nor any parent declares a speed;";
        else
            out << " every speed declaration requests inheritance without a finite positive parent speed;";
    }
    out << " the firmware cannot safely emit this command.\n";
    return out.str();
}

// Walk the borrowed tree with inherited process state and persistent tool state;
// append movement diagnostics, including deposits outside extrusion roots, to the
// caller's report without changing geometry.
void validate_extrusion_tree_movements(const Print &print,
                                       const ExtrusionEntity &entity,
                                       const PrintingTreeLocation &location,
                                       std::vector<size_t> &child_path,
                                       const SpeedTraversalState &parent_state,
                                       ActiveToolState &active_tool,
                                       const size_t extruder_count,
                                       SpeedValidationReport &report)
{
    SpeedTraversalState state = parent_state;
    if (const ExtrusionPropertyFan *fan = entity.get_property<ExtrusionPropertyFan>()) {
        if (!fan->is_valid(report.runtime_orchestrator))
            report.add_fan_error(extrusion_tree_path(location, child_path) +
                ": invalid EPropertyFan pair: unregistered mode or invalid active percentage.\n");
    }
    if (const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>()) {
        state.declarations.push_back(SpeedDeclaration{child_path, speed->speed_mm_per_s});
        if (!std::isfinite(speed->speed_mm_per_s)) {
            state.has_non_finite_speed = true;
        } else if (speed->speed_mm_per_s > 0.f) {
            state.effective_speed = speed->speed_mm_per_s;
        }
    }
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>()) {
        state.effective_role = attributes->role;
        state.effective_attributes = *attributes;
    }
    if (entity.get_property<ExtrusionPropertyExtrusionAxis>() != nullptr)
        state.has_extrusion_axis = true;

    const ExtrusionPropertySpecialCommand *special_command =
        entity.get_property<ExtrusionPropertySpecialCommand>();
    if (special_command != nullptr &&
        special_command->command_code() == ExtrusionPropertySpecialCommand::Code::TOOLCHANGE) {
        // Semantic tool changes are persistent machine state. Process them on
        // node entry, just like ExtrusionWriterVisitor, so following siblings
        // and later event roots observe the selected tool.
        if (!std::isfinite(special_command->extra_data) || special_command->extra_data < 0.0 ||
            special_command->extra_data > double((std::numeric_limits<extruder_index_t>::max)()) ||
            std::trunc(special_command->extra_data) != special_command->extra_data) {
            active_tool.tool_id.reset();
            active_tool.source = extrusion_tree_path(location, child_path) +
                " contains an invalid TOOLCHANGE value";
        } else {
            active_tool.tool_id = static_cast<extruder_index_t>(special_command->extra_data);
            active_tool.source = extrusion_tree_path(location, child_path) +
                " SpecialCommand::TOOLCHANGE";
        }
    }

    if (special_command != nullptr &&
        special_command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION) {
        const double extrusion_delta = special_command->extra_data;
        const bool invalid_delta = !std::isfinite(extrusion_delta);
        const bool invalid_tool = !active_tool.tool_id || *active_tool.tool_id >= extruder_count;
        const bool invalid_speed = state.has_non_finite_speed ||
            (std::isfinite(extrusion_delta) && extrusion_delta != 0.0 &&
             (!state.effective_speed || !std::isfinite(*state.effective_speed) ||
              *state.effective_speed <= 0.f));
        if (invalid_delta || invalid_tool || invalid_speed) {
            const std::string detail = e_only_extrusion_error(
                print, entity, location, child_path, state, active_tool,
                extruder_count, extrusion_delta);
            if (location.extrusion == nullptr)
                report.add_scope_event_error(detail);
            else
                report.add_extrusion_e_only_error(detail);
        }
    }

    if (special_command != nullptr &&
        special_command->command_code() == ExtrusionPropertySpecialCommand::Code::DEPOSIT) {
        const ArcPolyline *deposit_point = entity.polyline_or_null();
        const bool valid_role = state.effective_role &&
            is_supported_legacy_marker_role(*state.effective_role) &&
            !RAW_EXTRUSION_ROLE_IS_TRAVEL(*state.effective_role) &&
            !RAW_EXTRUSION_ROLE_IS_WIPE(*state.effective_role) &&
            !RAW_EXTRUSION_ROLE_IS_RETRACT(*state.effective_role) &&
            !RAW_EXTRUSION_ROLE_IS_UNRETRACT(*state.effective_role);
        const bool valid_flow = state.effective_attributes &&
            std::isfinite(state.effective_attributes->mm3_per_mm) && state.effective_attributes->mm3_per_mm > 0.0 &&
            std::isfinite(state.effective_attributes->width) && state.effective_attributes->width > 0.f &&
            std::isfinite(state.effective_attributes->height) && state.effective_attributes->height > 0.f;
        if (location.extrusion == nullptr || entity.child_count() != 0 ||
            deposit_point == nullptr || deposit_point->size() != 1 ||
            !std::isfinite(special_command->extra_data) || special_command->extra_data <= 0.0 ||
            !valid_role || !valid_flow || state.has_extrusion_axis ||
            state.has_non_finite_speed || !state.effective_speed ||
            !std::isfinite(*state.effective_speed) || *state.effective_speed <= 0.f ||
            !active_tool.tool_id || *active_tool.tool_id >= extruder_count) {
            std::ostringstream out;
            out << "PrintingPlan location: " << extrusion_tree_path(location, child_path) << "\n";
            append_plan_source_context(out, print, location);
            out << "DEPOSIT needs one point, no children or extrusion-axis request, finite positive volume, "
                   "printable flow/width/height, a printable role, positive speed and an active tool, "
                   "inside a PrintingExtrusion root rather than an event scope.\n";
            append_speed_declarations(out, location, state);
            if (location.extrusion == nullptr)
                report.add_scope_event_error(out.str());
            else
                report.add_extrusion_e_only_error(out.str());
        }
    }

    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        child_path.push_back(child_idx);
        validate_extrusion_tree_movements(
            print, entity.child(child_idx), location, child_path, state,
            active_tool, extruder_count, report);
        child_path.pop_back();
    }

    const ArcPolyline *polyline = entity.polyline_or_null();
    const bool geometric_movement = entity.child_count() == 0 &&
        polyline != nullptr && polyline->size() >= 2;

    // Retraction commands and EPropertyExtrusionAxis are intentionally outside
    // this validation. Their effective speed depends on configured retraction
    // policy and runtime extrusion-axis state rather than only on this tree.
    if (!geometric_movement)
        return;
    if (state.effective_role &&
        !is_supported_legacy_marker_role(*state.effective_role)) {
        report.add_unsupported_role_error(unsupported_role_error(
            print, entity, location, child_path, *state.effective_role));
    }
    if (!state.has_non_finite_speed && state.effective_speed &&
        std::isfinite(*state.effective_speed) && *state.effective_speed > 0.f)
        return;

    const std::string detail = geometric_speed_error(print, entity, location, child_path, state);
    if (location.extrusion == nullptr)
        report.add_scope_event_error(detail);
    else
        report.add_speed_error(detail);
}

void validate_scope_event_movements(const Print &print,
                                    const ExtrusionEntity &event_root,
                                    const std::string &root_path,
                                    const Printing::PrintingLayerGroup *layer_group,
                                    const Printing::PrintingToolGroup *tool_group,
                                    ActiveToolState &active_tool,
                                    const size_t extruder_count,
                                    SpeedValidationReport &report)
{
    // Requested properties restart for every write_event() call. Active tool
    // state is deliberately shared with the caller because firmware tool
    // selection survives between independently serialized roots.
    const PrintingTreeLocation location{root_path, layer_group, tool_group, nullptr};
    std::vector<size_t> child_path;
    validate_extrusion_tree_movements(print, event_root, location, child_path,
                                      SpeedTraversalState{}, active_tool,
                                      extruder_count, report);
}

std::string speed_validation_message(const SpeedValidationReport &report)
{
    std::ostringstream out;
    out << "STEP_GCODE cannot generate G-code because the PrintingPlan contains "
        << report.error_count << " validation " << (report.error_count == 1 ? "error" : "errors") << ".\n";
    if (report.speed_error_count > 0)
        out << report.speed_error_count << " geometric extrusion "
            << (report.speed_error_count == 1 ? "leaf has" : "leaves have")
            << " no valid effective speed. A geometric leaf must define or inherit a finite speed"
               " greater than zero.\n";
    if (report.extrusion_e_only_error_count > 0)
        out << report.extrusion_e_only_error_count << " E-only special extrusion "
            << (report.extrusion_e_only_error_count == 1 ? "event is" : "events are")
            << " invalid. A non-zero E delta needs a finite positive inherited speed and every"
               " command needs a configured active tool.\n";
    if (report.scope_event_error_count > 0)
        out << report.scope_event_error_count << " movement-producing before/after event "
            << (report.scope_event_error_count == 1 ? "entity is" : "entities are")
            << " invalid. Each event tree has independent inherited process state, while the"
               " active tool follows PrintingPlan serialization order.\n";
    if (report.missing_root_count > 0)
        out << report.missing_root_count << " PrintingExtrusion "
            << (report.missing_root_count == 1 ? "has" : "entries have") << " no root.\n";
    if (report.unsupported_role_error_count > 0)
        out << report.unsupported_role_error_count << " geometric extrusion "
            << (report.unsupported_role_error_count == 1 ? "leaf has" : "leaves have")
            << " an unsupported effective role which cannot be converted to a legacy marker.\n";
    if (report.fan_error_count > 0)
        out << report.fan_error_count << " invalid fan request pair(s). Active percentages must be finite"
               " and in [0,100], and modes must belong to the fan runtime-ID domain.\n";
    out << "All PrintingPlan indices below are zero-based.\n";
    for (size_t detail_idx = 0; detail_idx < report.details.size(); ++detail_idx)
        out << "\n[" << detail_idx + 1 << "/" << report.error_count << "]\n"
            << report.details[detail_idx];
    if (report.error_count > report.details.size())
        out << "\n" << report.error_count - report.details.size()
            << " additional validation errors were omitted.\n";
    return out.str();
}

FirmwareInstanceOwner::~FirmwareInstanceOwner()
{
    // A provider may publish owning fields before producing an invalid table.
    // Destroy whenever the required pointers are available so validation
    // failures do not leak that partially published session.
    if (m_instance.session != nullptr && m_instance.vtable != nullptr &&
        m_instance.vtable->struct_size >=
            offsetof(raw_gcode_firmware_vtable, destroy) + sizeof(gcode_firmware_destroy_fn) &&
        m_instance.vtable->destroy != nullptr)
        m_instance.vtable->destroy(m_instance.session);
}

raw_gcode_firmware_instance *FirmwareInstanceOwner::mutable_instance()
{
    return &m_instance;
}

const raw_gcode_firmware_instance *FirmwareInstanceOwner::instance() const
{
    return &m_instance;
}

void create_firmware_session(Orchestrator &orchestrator,
                             Print &print,
                             const raw_gcode_script_processor *script_processor,
                             FirmwareInstanceOwner &owner)
{
    Plugin *firmware_plugin = selected_or_active_plugin_for_step(
        orchestrator, GCODE_FIRMWARE, &print.full_print_config());
    if (firmware_plugin == nullptr)
        throw RuntimeError("No active G-code firmware plugin is available.");

    std::atomic_bool execution_error { false };
    plugin_host_context host_context =
        Detail::prepare_guarded_plugin_host_context(
            orchestrator, GCODE_FIRMWARE, *firmware_plugin, print, execution_error);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(
        GCODE_FIRMWARE, firmware_plugin, &host_context);
    run_ctx_gcode_firmware payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.script_processor = script_processor;
    run_context.data = &payload;

    firmware_plugin->setup(run_context, 1);
    Detail::throw_if_plugin_stopped(orchestrator, print, GCODE_FIRMWARE, *firmware_plugin, execution_error, "setup");
    firmware_plugin->setup_run(run_context);
    Detail::throw_if_plugin_stopped(orchestrator, print, GCODE_FIRMWARE, *firmware_plugin, execution_error, "setup_run");
    firmware_plugin->run(run_context);

    // Transfer ownership before checking cancellation or table completeness;
    // the provider may allocate a session before reporting its error.
    *owner.mutable_instance() = payload.instance;
    Detail::throw_if_plugin_stopped(orchestrator, print, GCODE_FIRMWARE, *firmware_plugin, execution_error, "run");

    // Constructing the borrowed view validates struct sizes and every required
    // callback without invoking firmware code.
    const slic3r_api::GCodeFirmwareView validated(owner.instance());
    (void)validated;
}

// The facts come from the plan, not from the text, so they exist even when no Reader result is requested. The used tools
// are sorted by index; their filament types are sorted alphabetically and joined by "," without removing duplicates, as
// the {printing_filament_types} file name variable has always been.
void publish_plan_facts(Print &print)
{
    const Printing::PlanFacts facts = Printing::summarize_plan(print.printing_plan());
    const PrintConfig &config = print.config();
    PrintStatistics &statistics = print.print_statistics();
    statistics.total_toolchanges = facts.toolchanges;
    statistics.initial_extruder_id = facts.initial_tool;
    statistics.initial_filament_type = config.filament_type.get_at(size_t(facts.initial_tool));
    statistics.printing_extruders = facts.used_tools;
    std::vector<std::string> filament_types;
    for (extruder_index_t tool : facts.used_tools)
        filament_types.push_back(config.filament_type.get_at(size_t(tool)));
    std::sort(filament_types.begin(), filament_types.end());
    statistics.printing_filament_types.clear();
    for (size_t index = 0; index < filament_types.size(); ++index)
        statistics.printing_filament_types += (index == 0 ? "" : ",") + filament_types[index];
    statistics.object_layer_count = Printing::object_layer_count(print);
    statistics.total_layer_count = facts.layer_count;
    statistics.max_print_z = unscaled(facts.max_print_z);
}

} // namespace

void clear(Print &print)
{
    // A failed or skipped writer must not leave the next preview phase pointing
    // at text produced by an earlier export.
    print.reset_gcode_reader_source_path();
    print.print_statistics().clear();
}

void prepare(Print &) {}

bool validate_pre(const Orchestrator &orchestrator, const Print &print, std::string *error)
{
    if (error != nullptr)
        error->clear();

    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr) {
        if (error != nullptr)
            *error = "STEP_GCODE requires the PrintingPlan produced by STEP_ORDERING, but the Print has no plan.";
        return false;
    }

    SpeedValidationReport report(orchestrator);
    const size_t extruder_count = configured_extruder_count(print);
    ActiveToolState active_tool;
    if (plan->events.has_before())
        validate_scope_event_movements(print, plan->events.before(), "plan.events.before",
                                       nullptr, nullptr, active_tool, extruder_count, report);
    for (size_t group_idx = 0; group_idx < plan->groups.size(); ++group_idx) {
        const Printing::PrintingGroup &group = plan->groups[group_idx];
        const std::string group_path = "groups[" + std::to_string(group_idx) + "]";
        if (group.events.has_before())
            validate_scope_event_movements(print, group.events.before(),
                                           group_path + ".events.before", nullptr, nullptr,
                                           active_tool, extruder_count, report);
        for (size_t layer_idx = 0; layer_idx < group.layers.size(); ++layer_idx) {
            const Printing::PrintingLayerGroup &layer_group = group.layers[layer_idx];
            const std::string layer_path = group_path + ".layers[" +
                std::to_string(layer_idx) + "]";
            if (layer_group.events.has_before())
                validate_scope_event_movements(print, layer_group.events.before(),
                                               layer_path + ".events.before", &layer_group,
                                               nullptr, active_tool, extruder_count, report);
            for (size_t tool_group_idx = 0; tool_group_idx < layer_group.tool_groups.size(); ++tool_group_idx) {
                const Printing::PrintingToolGroup &tool_group = layer_group.tool_groups[tool_group_idx];
                const std::string tool_group_path = layer_path + ".tool_groups[" +
                    std::to_string(tool_group_idx) + "]";
                // begin_tool_group() establishes the intended tool before its
                // ordered events. A semantic TOOLCHANGE inside a tree may then
                // replace this state for following commands.
                active_tool.tool_id = tool_group.extruder_id;
                active_tool.source = tool_group_path + ".extruder_id";
                if (tool_group.events.has_before())
                    validate_scope_event_movements(print, tool_group.events.before(),
                                                   tool_group_path + ".events.before",
                                                   &layer_group, &tool_group, active_tool,
                                                   extruder_count, report);
                for (size_t extrusion_idx = 0; extrusion_idx < tool_group.extrusions.size(); ++extrusion_idx) {
                    const Printing::PrintingExtrusion &extrusion = tool_group.extrusions[extrusion_idx];
                    const PrintingTreeLocation location{
                        tool_group_path + ".extrusions[" +
                            std::to_string(extrusion_idx) + "].root",
                        &layer_group,
                        &tool_group,
                        &extrusion
                    };
                    if (extrusion.root == nullptr) {
                        std::ostringstream detail;
                        detail << "PrintingPlan location: " << extrusion_tree_path(location, {}) << "\n";
                        append_plan_source_context(detail, print, location);
                        detail << "Entity: unavailable because the PrintingExtrusion has no root.\n"
                               << "Speed declarations: unavailable because there is no extrusion tree.\n"
                               << "Reason: the PrintingExtrusion has no root extrusion tree.\n";
                        report.add_missing_root(detail.str());
                        continue;
                    }

                    std::vector<size_t> child_path;
                    validate_extrusion_tree_movements(
                        print, *extrusion.root, location, child_path,
                        SpeedTraversalState{}, active_tool, extruder_count, report);
                }
                if (tool_group.events.has_after())
                    validate_scope_event_movements(print, tool_group.events.after(),
                                                   tool_group_path + ".events.after",
                                                   &layer_group, &tool_group, active_tool,
                                                   extruder_count, report);
            }
            if (layer_group.events.has_after())
                validate_scope_event_movements(print, layer_group.events.after(),
                                               layer_path + ".events.after", &layer_group,
                                               nullptr, active_tool, extruder_count, report);
        }
        if (group.events.has_after())
            validate_scope_event_movements(print, group.events.after(),
                                           group_path + ".events.after", nullptr, nullptr,
                                           active_tool, extruder_count, report);
    }
    if (plan->events.has_after())
        validate_scope_event_movements(print, plan->events.after(), "plan.events.after",
                                       nullptr, nullptr, active_tool, extruder_count, report);

    if (report.error_count == 0)
        return true;
    if (error != nullptr)
        *error = speed_validation_message(report);
    return false;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

// Validate the plan, build the firmware session and the Reader association once,
// then run every selected STEP_GCODE plugin in priority order on one payload.
// Returns the Reader plugin ID chosen by the firmware. Any validation, plugin
// error or cancellation throws; no plugin is then recorded as completed. The host
// Reader unit is not completed here: the pipeline does it after the read.
std::string run_step(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    if (path.empty())
        throw RuntimeError("G-code generation plugin needs a non-empty output path.");

    // Run the host-owned validation before constructing plugins or touching a
    // staging file. RuntimeError is required here instead of the debug-only
    // validate_or_report helper so Release builds expose the same diagnosis.
    std::string validation_error;
    if (!validate_pre(orchestrator, print, &validation_error))
        throw RuntimeError(validation_error);

    const std::vector<Plugin *> plugins = selected_or_active_plugins_for_step(
        orchestrator, STEP_GCODE, &print.full_print_config());
    if (plugins.empty())
        throw RuntimeError("No active G-code generation plugin is available.");

    // The G-code step consumes the finalized PrintingPlan created by the
    // ordering and editing stages.
    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr)
        throw RuntimeError("G-code generation needs a finalized PrintingPlan.");

    // The host facade validates the Reader identity selected by the firmware.
    // Firmware sessions execute that Reader through their plugin-side helper.
    GCodeScriptProcessor scripts(print, orchestrator);
    GCodeReaderService readers(orchestrator, &print);
    FirmwareInstanceOwner firmware;
    create_firmware_session(orchestrator, print, scripts.c_processor(), firmware);
    const slic3r_api::GCodeFirmwareView firmware_view(firmware.instance());
    const std::string reader_plugin_id = readers.validated_reader_id(
        firmware_view.reader_plugin_id());
    ReaderSourcePathReceiver reader_source(print);
    run_ctx_generate_gcode payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<const printing_plan_handle *>(plan);
    payload.firmware = firmware.instance();
    payload.output_path = path.c_str();
    payload.reader_source_context = &reader_source;
    payload.publish_reader_source_path = &ReaderSourcePathReceiver::publish_callback;

    // Plugins run one after the other, by increasing priority, on the same
    // payload and firmware session: the writer creates the file, the following
    // ones may modify it. The first error or cancellation aborts the step.
    for (Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;

        std::atomic_bool execution_error { false };
        plugin_host_context host_context =
            Detail::prepare_guarded_plugin_host_context(orchestrator, STEP_GCODE, *plugin, print, execution_error);
        plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_GCODE, plugin, &host_context);
        run_context.data = &payload;

        plugin->setup(run_context, 1);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_GCODE, *plugin, execution_error, "setup");
        plugin->setup_run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_GCODE, *plugin, execution_error, "setup_run");
        plugin->run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, print, STEP_GCODE, *plugin, execution_error, "run");
    }
    // Everything that can still fail runs before the progress units are recorded;
    // the units are then marked in plan order. A failure of the step resets the whole
    // progress table (see run_step_if_requested), so no unit stays half consumed.
    publish_plan_facts(print);
    for (const Plugin *plugin : plugins)
        if (plugin != nullptr)
            print.pipeline_progress().complete_plugin(STEP_GCODE, plugin->get_id());
    return reader_plugin_id;
}

} // namespace Slic3r::Steps::StepGenerateGcode
