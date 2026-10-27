///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas, Filip Sykala @Jony01, Oleksandra Iushchenko @YuSanka, Vojtěch Král @vojtechkral
///|/ Copyright (c) BambuStudio 2023 manch1n @manch1n
///|/ Copyright (c) SuperSlicer 2022 Remi Durand @supermerill
///|/ Copyright (c) 2019 Bryan Smith
///|/ Copyright (c) 2017 Eyal Soha @eyal0
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2017 Joseph Lenox @lordofhyphens
///|/
///|/ ported from lib/Slic3r/Print.pm:
///|/ Copyright (c) Prusa Research 2016 - 2018 Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) Slic3r 2011 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 - 2013 Mark Hindess
///|/ Copyright (c) 2013 Devin Grady
///|/ Copyright (c) 2012 - 2013 Mike Sheldrake @mesheldrake
///|/ Copyright (c) 2012 Henrik Brix Andersen @henrikbrixandersen
///|/ Copyright (c) 2012 Michael Moon
///|/ Copyright (c) 2011 Richard Goodwin
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
Print represents one FFF slicing job. It owns the applied configuration, sliced
objects and processing state. Applying a model updates that state and marks
affected results for recomputation; the pipeline then produces geometry, a
printing plan and the G-code used by the preview. Exporting a file consumes
those results rather than transferring ownership of the sliced objects.

The model describes source geometry and placements. PrintObject holds the
corresponding sliced data; the printing plan places generated extrusion paths
in print order. Placement checks and measured statistics serve different parts
of that lifecycle and must not be confused with generated geometry.

Print owns effective configuration, sliced objects and pipeline state. Its
placement checks borrow model instances and use config-derived square tower
envelopes; instance-tower offsets follow model translation, not rotation.
Tool ordering supports preview ticks and legacy export. Statistics store
measured totals including the wipe-tower share for sidebar and placeholders.
Validation includes print-level setting requirements on selected providers.
*/
#ifndef slic3r_Print_hpp_
#define slic3r_Print_hpp_

#include <atomic>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "DataTreeFwd.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Flow.hpp"
#include "GCode/ThumbnailData.hpp"
#include "GCode/ToolOrdering.hpp"
#include "libslic3r.h"
#include "Point.hpp"
#include "PluginProperty.hpp"
#include "PrintBase.hpp"
#include "Config/FFFPrintConfig.hpp"
#include "PrintRecords.hpp"
#include "Api/host/ExportInputs.hpp"
#include "Api/plugin/c/slic3r_slicing_step.h"
#include "Steps/PipelineProgress.hpp"

namespace Slic3r {

class GCodeGenerator;
class Plugin;
struct GCodeProcessorResult;
struct ConflictResult;
namespace Printing {
struct PrintingPlan;
}
namespace ApiInternal {
struct PrintAccess;
}
namespace Steps {
enum class Checkpoint : uint8_t;
}

struct PrintStatistics
{
    PrintStatistics() { clear(); }
    // PrintEstimatedStatistics::ETimeMode::Normal -> time
    std::map<uint8_t, double>       estimated_print_time;
    std::map<uint8_t, std::string>  estimated_print_time_str;
    double                          total_used_filament;
    std::vector<std::pair<size_t, double>> color_extruderid_to_used_filament; // id -> mm (length)
    double                          total_extruded_volume;
    double                          total_cost;
    int                             total_toolchanges;
    double                          total_weight;
    double                          total_wipe_tower_cost;
    double                          total_wipe_tower_filament; // filament length in mm
    double                          total_wipe_tower_filament_weight; // grams
    double                          total_wipe_tower_volume; // mm3
    std::vector<std::pair<size_t, double>> color_extruderid_to_used_weight;
    std::vector<extruder_index_t>   printing_extruders;
    extruder_index_t                initial_extruder_id;
    std::string                     initial_filament_type;
    std::string                     printing_filament_types;
    std::map<extruder_index_t, double> filament_stats; // extruder id -> volume in mm3
    std::vector<std::pair<coord_t, float>> _layer_area_stats; // print_z to area

    // Z-distinct layers of the objects only (the "objects layers count" of the exported statistics).
    int                             object_layer_count;
    // Every PrintingLayerGroup of the PrintingPlan, auxiliary layers included (the "total layers count" of the exported statistics).
    int                             total_layer_count;
    // True when the Reader saw a positive filament volume extruded with a support or support interface role.
    bool                            has_support;

    std::atomic_bool is_computing_gcode;
    // True only after the Reader result of the current G-code has been fully converted into the measured totals above.
    // It is the sole signal that the measured values are usable; zero totals never mean "unavailable".
    std::atomic_bool statistics_available;

    // Copy measured print statistics, including tower shares, into a placeholder config.
    DynamicConfig           config() const;
    // Config with the statistics keys populated with placeholder strings.
    static DynamicConfig    placeholders();
    // Replace the print statistics placeholders in the path.
    std::string             finalize_output_path(const std::string &path_in) const;

    // Reset all measured totals, per-tool values and export-progress state.
    void clear() {
        estimated_print_time.clear();
        estimated_print_time_str.clear();
        total_used_filament    = 0.;
        total_extruded_volume  = 0.;
        total_cost             = 0.;
        total_toolchanges      = 0;
        total_weight           = 0.;
        total_wipe_tower_cost = 0.;
        total_wipe_tower_filament = 0.;
        total_wipe_tower_filament_weight = 0.;
        total_wipe_tower_volume = 0.;
        initial_extruder_id    = EXTRUDER_ID_UNKNOWN;
        initial_filament_type.clear();
        printing_filament_types.clear();
        filament_stats.clear();
        printing_extruders.clear();
        color_extruderid_to_used_filament.clear();
        color_extruderid_to_used_weight.clear();
        _layer_area_stats.clear();
        object_layer_count     = 0;
        total_layer_count      = 0;
        has_support            = false;
        is_computing_gcode = false;
        statistics_available = false;
    }

    static const std::string FilamentUsedG;
    static const std::string FilamentUsedGMask;
    static const std::string TotalFilamentUsedG;
    static const std::string TotalFilamentUsedGMask;
    static const std::string TotalFilamentUsedGValueMask;
    static const std::string FilamentUsedCm3;
    static const std::string FilamentUsedCm3Mask;
    static const std::string FilamentUsedMm;
    static const std::string FilamentUsedMmMask;
    static const std::string FilamentCost;
    static const std::string FilamentCostMask;
    static const std::string TotalFilamentCost;
    static const std::string TotalFilamentCostMask;
    static const std::string TotalFilamentCostValueMask;
    static const std::string TotalFilamentUsedWipeTower;
    static const std::string TotalFilamentUsedWipeTowerValueMask;
};

struct ConflictResult
{
    std::string _objName1;
    std::string _objName2;
    double      _height;
    const void* _obj1; // nullptr means wipe tower
    const void* _obj2;
    int         layer = -1;
    ConflictResult(const std::string& objName1, const std::string& objName2, double height, const void* obj1, const void* obj2)
        : _objName1(objName1), _objName2(objName2), _height(height), _obj1(obj1), _obj2(obj2)
    {}
    ConflictResult(const std::string& objName1, const std::string& objName2, double height, const void* obj1, const void* obj2, int layer_id)
        : _objName1(objName1), _objName2(objName2), _height(height), _obj1(obj1), _obj2(obj2), layer(layer_id)
    {}
    ConflictResult() = default;
};

using ConflictResultOpt = std::optional<ConflictResult>;

// The complete print tray with possibly multiple objects.
class Print : public PrintBase
{
private: // Prevents erroneous use by other classes.
    // Bool indicates if supports of PrintObject are top-level contour.
    typedef std::pair<PrintObject *, bool>         PrintObjectInfo;

public:
    Print();
    ~Print() override;

    PrinterTechnology	technology() const noexcept override { return ptFFF; }

    // Methods, which change the state of Print / PrintObject / PrintRegion.
    // The following methods are synchronized with process() and Orchestrator::export_gcode(),
    // so that slicing and exporting may be called from a background thread.
    // In case the following methods need to modify data processed by slicing or exporting,
    // a cancellation callback is executed to stop the background processing before the operation.
    void                clear() override;
    bool                empty() const override { return m_objects.empty(); }
    // List of existing PrintObject IDs, to remove notifications for non-existent IDs.
    std::vector<ObjectID> print_object_ids() const override;

    ApplyStatus         apply(const Model &model, DynamicPrintConfig config) override;
    void set_task(const TaskParams &params) override;
    // Slice the print and prepare tool ordering for preview ticks and legacy export.
    void process() override;
    void finalize() override;

    // methods for handling state
    // Canonical pipeline snapshots are copied under state_mutex.
    bool                is_step_done(slicing_step_t step) const;
    PrintStateBase::StateWithTimeStamp step_state_with_timestamp(slicing_step_t step) const;
    PrintStateBase::StateWithWarnings  step_state_with_warnings(slicing_step_t step) const;
    // Returns true if the last step was finished with success.
    bool                finished() const override { return this->is_step_done(STEP_GCODE); }

    bool                has_infinite_skirt() const;
    bool                has_skirt() const;
    bool                has_brim() const;

    // After apply(), validate placement/settings and selected providers using retained group descriptions; return an owned status/message pair.
    // Append to borrowed warnings when supplied and refresh cached clearance contours without changing registrations.
    // Geometry, plugin-resolution, config-comparison and allocation failures may propagate; no ownership is retained.
    std::pair<PrintValidationError, std::string> validate(std::vector<std::string>* warnings = nullptr) const override;
    Flow                brim_flow(extruder_index_t extruder_id, const PrintObjectConfig &brim_config) const;
    Flow                skirt_flow(extruder_index_t extruder_id, bool first_layer=false) const;
    coord_t             get_min_first_layer_height() const;
    coord_t             get_object_first_layer_height(const PrintObject& object) const;

    // get the extruders of these objects
    std::set<extruder_index_t> object_extruders(const PrintObjectPtrs &objects, coord_t z = -1) const;
    // get all extruders from the list of objects in this print ( same as print.object_extruders(print.objects()) )
    std::set<extruder_index_t> object_extruders(coord_t z = -1) const;
    std::set<extruder_index_t> support_material_extruders(coord_t z = -1) const;
    // Collect object/support tools at scaled Z, adding the global tower contour tool only for all-layer queries.
    std::set<extruder_index_t> extruders(coord_t z = -1) const;
    // Effective first-layer bed temperature across every tool used at Z=0.
    int32_t first_layer_bed_temperature() const;
    double              max_allowed_layer_height() const;
    bool                has_support_material() const;
    // Make sure the background processing has no access to this model_object during this call!
    void                auto_assign_extruders(ModelObject* model_object) const;

    const PrintConfig&          config() const { return m_config; }
    const PrintRecordStore&     records() const { return m_records; }
    PrintRecordStore&           records() { return m_records; }
    const PrintObjectConfig&    default_object_config() const { return m_default_object_config; }
    const PrintRegionConfig&    default_region_config() const { return m_default_region_config; }

    PrintObjectCRefs            objects() const { return make_ref_view<PrintObject>(m_objects); }
    PrintObjectRefs             objects() { return make_ref_view<PrintObject>(m_objects); }
    const PrintObject&          object(size_t idx) const { return *m_objects[idx]; }
    PrintObject&                object(size_t idx) { return *m_objects[idx]; }
    const PrintObject *get_print_object_by_model_object_id(ObjectID object_id) const;
    // PrintObject by its ObjectID, to be used to uniquely bind slicing warnings to their source PrintObjects
    // in the notification center.
    const PrintObject *get_object(ObjectID object_id) const;
    // How many of PrintObject::copies() over all print objects are there?
    // If zero, then the print is empty and the print shall not be executed.
    uint16_t                    num_object_instances() const;
    // Sort the PrintObjects by their increasing Z, likely useful for avoiding colisions on Deltas during sequential prints.
    std::vector<const PrintInstance*> sort_object_instances_by_max_z() const;
    // Sort the PrintObjects by their increasing Y, likely useful for avoiding colisions on printer with a x-bar during sequential prints.
    std::vector<const PrintInstance*> sort_object_instances_by_max_y() const;
    // Produce a vector of PrintObjects in the order of their respective ModelObjects in print.model().
    std::vector<const PrintInstance*> sort_object_instances_by_model_order() const;

    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const std::optional<ExtrusionEntityCollection>& skirt_first_layer() const;
    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const ExtrusionEntityCollection& skirt() const;
    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const ExtrusionEntityCollection& brim() const;
    // Convex hull of the 1st layer extrusions, for bed leveling and placing the initial purge line.
    // It encompasses the object extrusions, support extrusions, skirt and brim, but not wipe towers.
    // It does NOT encompass user extrusions generated by custom G-code,
    // therefore it does NOT encompass the initial purge line.
    // It does NOT encompass MMU/MMU2 starting (wipe) areas.
    const Polygon&                   first_layer_convex_hull() const { return m_first_layer_convex_hull; }
    // Compute an export envelope from the cached first-layer hull and current global/instance tower corners.
    Polygon                          first_layer_convex_hull_with_wipe_towers() const;

    const PrintStatistics&      print_statistics() const { return m_print_statistics; }
    PrintStatistics&            print_statistics() { return m_print_statistics; }
    const std::optional<ConflictResult>& conflict_result() const { return m_conflict_result; }
    std::time_t                 timestamp_last_change() const { return m_timestamp_last_change; }

    // Wipe tower support.
    // Test eligibility for any configured global or instance tower.
    bool                        has_wipe_tower() const;
    // Test global-tower eligibility outside complete-object/vase modes, independent of parallel scheduling.
    bool                        has_global_wipe_tower() const;
    const std::vector<ToolOrdering> &tool_orderings() const { return m_tool_orderings; }
    const Printing::PrintingPlan *printing_plan() const { return m_printing_plan.get(); }
    Printing::PrintingPlan &mutable_printing_plan();
    void reset_printing_plan();
    // Release only the live plan and keep the Checkpoint::Ordering copy, which
    // restores a fresh working plan when the pipeline resumes.
    void release_live_printing_plan();
    // Control whether Checkpoint::Ordering is available: when enabled, a deep
    // copy of the plan is kept after STEP_ORDERING so plan editors can replay
    // without re-ordering. Disabling it releases the copy.
    void set_keep_ordering_checkpoint(bool keep);
    bool keep_ordering_checkpoint() const { return m_keep_ordering_checkpoint; }
    // Store, query, restore or release the Checkpoint::Ordering plan copy.
    // Restoring clones the copy so it can be restored again later.
    bool save_ordering_checkpoint();
    bool has_ordering_checkpoint() const;
    bool restore_ordering_checkpoint();
    void clear_ordering_checkpoint();
    // Records and main-object properties, not geometry: None before LayerHeight prepare/plugins; Slicing after successful, noncancelled PostSlicing.
    void save_metadata_checkpoint(Steps::Checkpoint checkpoint);
    // Both operations require caller-owned execution or stopped workers; neither acquires a lock.
    bool has_metadata_checkpoint(Steps::Checkpoint checkpoint) const;
    // Path of the finished text stream used by GCodeReader and GCodeViewer.
    // The producer owns the file; Print owns only this copied UTF-8 path.
    const std::string &gcode_reader_source_path() const { return m_gcode_reader_source_path; }
    // Return the reader selected by the firmware for the published export source.
    const std::string &gcode_reader_plugin_id() const { return m_gcode_reader_plugin_id; }
    // Own the readable export path and its selected reader identity until invalidation.
    void set_gcode_reader_source_path(std::string path, std::string reader_plugin_id = {});
    // Discard the published source path and reader identity together.
    void reset_gcode_reader_source_path();

    /*
    One export request, from its deposit by the caller to its consumption by the STEP_EXPORT runner.
    It is single-use: the runner moves it out of the Print when it starts, so a later export needs a
    new request. config is a private copy of full_print_config() plus the request-only entries
    (output_path, output_path_suggestion); it never takes part in Print::apply(). The optional suggestion directory and
    filename base let the caller influence the exporter's proposed path without setting its destination. exporter_id lets the
    caller name the exporter of this request without touching the configuration. thumbnails renders
    the images the exporter asks for, and stays empty when no graphical renderer exists (command line). inputs asks the
    requester for the values that the exporter needs (see ExportInputForm); it stays empty when the requester cannot be
    asked, which fails an exporter that asks.
    */
    struct ExportRequest
    {
        DynamicPrintConfig          config;
        ThumbnailsGeneratorCallback thumbnails;
        ExportInputsCallback        inputs;
        // Existing directory to use when forming the exporter's proposed destination; empty keeps the model-derived directory.
        std::string output_path_suggestion_directory;
        // Base name used when forming the exporter's proposed destination; empty keeps the automatic name.
        std::string output_path_suggestion_filename_base;
        // Fold accents in the suggested filename for outputs intended for legacy firmware.
        bool output_path_suggestion_ascii_fallback = false;
        // ID of the G-code exporter to use for this request only; empty uses the `gcode_exporter_plugin` setting of config.
        // The setting itself is never changed by it.
        std::string exporter_id;

        // Destination of the export, or an empty string when none was set.
        std::string output_path() const
        {
            const ConfigOptionString *option = config.option<ConfigOptionString>("output_path");
            return option == nullptr ? std::string() : option->value;
        }
        void        set_output_path(const std::string &path) { config.set_key_value("output_path", new ConfigOptionString(path)); }
    };
    // Copy full_print_config() under state_mutex() (which the caller must not hold) into a new request
    // without destination. The request is owned by the caller until request_export().
    ExportRequest make_export_request(ThumbnailsGeneratorCallback thumbnails) const;
    // Queue STEP_EXPORT with this request. Takes state_mutex() (the caller must not hold it), replaces
    // any previous unconsumed request and discards the previous result. It never arms the plugin
    // cleanup, so a later apply() does not stop the worker because of it. The request lives until
    // take_export_request() or clear().
    void request_export(ExportRequest request);
    // Move the pending request out of the Print under state_mutex(); empty when none was deposited
    // or when it was already taken.
    std::optional<ExportRequest> take_export_request();
    // Path of the G-code reference when STEP_GCODE is done, or an empty string when the reference is not
    // ready (never produced, invalidated or being regenerated). The step state is read under state_mutex(); the path
    // is only safe to read from the thread that produces or consumes the reference (see Print.cpp).
    std::string ready_gcode_reference_path() const;
    /*
    What the exporter reported about its export: the local file it wrote and a label to display. Either may be empty;
    an exporter that writes no local file reports an empty local_path.
    */
    struct ExportResult
    {
        std::string local_path;
        std::string label;
    };
    // Result reported by the last export, or an empty result. Orchestrator::export_print() empties it when it starts, then it is recorded as soon as the exporter reports it, before
    // the step returns, so that a caller can clean up a file already published even when the step fails or is cancelled
    // afterwards. Valid until the next export, request_export() or clear(); an invalidation of STEP_GCODE leaves it.
    ExportResult export_result() const;
    // Record the result under state_mutex() (locked here). The host calls it while the exporter runs, when it reports; it replaces the previous result.
    void         set_export_result(ExportResult result);

    const PrintObject *auxiliary_object() const { return m_auxiliary_object.get(); }
    PrintObject &mutable_auxiliary_object();
    void reset_auxiliary_object();

    std::string                 output_filename(const std::string &filename_base = std::string()) const override;
    // Format FFF output placeholders with the G-code exporter extension rather than the binary preference.
    std::string                 output_filename(const std::string &filename_base,
                                                 const std::string &default_extension) const override;

    // Intern a complete snapshot. Concurrent registrations/readers are allowed;
    // apply() and clear() must run with workers stopped. Does not activate it.
    const PrintRegion &register_print_region(const PrintRegionConfig &config);
    // Complete regional overrides using this Print's current defaults. Invalid
    // keys/types return nullptr without modifying the registry.
    const PrintRegion *register_print_region_config(const ConfigBase &config);
    // Rebuild only the active catalogue in deterministic object order.
    void rebuild_active_print_regions();

    size_t                      num_print_regions() const throw() { return m_print_regions.size(); }
    PrintRegionCRefs            print_regions() const  { return make_ref_view<PrintRegion, PrintRegionPtrs>(m_print_regions); }
    const PrintRegion&          print_region(size_t idx) const  { return *m_print_regions[idx]; }

    const Polygons& get_sequential_print_clearance_contours() const { return m_sequential_print_clearance_contours; }
//TODO: decide to use this one or the printconfig one.
    // Check clearance between instance envelopes, including their configured towers.
    static bool sequential_print_horizontal_clearance_valid(const Print& print, Polygons* polygons = nullptr);

    //put this in public to be accessible for tests, it was in private before.
    bool                invalidate_state_by_config_options(const ConfigOptionResolver& new_config, const std::vector<t_config_option_key> &opt_keys);

    // Control thread only: callers hold state_mutex, which the GUI cancel
    // callback expects before it can stop and wait for the worker. Worker-side
    // paths must use request_replay_from_worker() instead.
    bool                request_replay_from(Steps::Checkpoint checkpoint);
    // Worker/idle only: refresh scheduling without self-cancellation.
    bool                request_replay_from_worker(Steps::Checkpoint checkpoint);
    bool                slices_ready() const;
    // Latest checkpoint whose data is valid: the checkpoint just before the
    // first requested step, moved back to Checkpoint::Print when its plan
    // (Ordering copy or live Plan) is missing.
    Steps::Checkpoint   current_checkpoint() const;
    // Pipeline-only: request every step after current_checkpoint() and put the
    // live plan in the state that checkpoint describes (restored Ordering copy,
    // or released plan for Checkpoint::Print and earlier).
    void                prepare_checkpoint_resume();
    /*
    One successful plugin setup() that may still hold caches for this Print. The
    entry is kept until a clear callback succeeds, so a failed cleanup can be
    retried before the owners it borrowed are replaced. setup_order is the host's
    monotonic setup counter, used to unwind participants in reverse order.
    */
    struct PluginCleanupParticipant
    {
        Plugin *plugin = nullptr;
        // Pipeline step whose setup created the state; also its cleanup boundary.
        slicing_step_t step = STEP_NONE;
        size_t setup_order = 0;
    };
    // Record one plugin setup so it can receive a clear callback later.
    void track_plugin_setup(Plugin &plugin, slicing_step_t step);
    // Ask every tracked plugin at or after first_invalid_step to release the
    // state it prepared. Returns false when at least one callback failed; failed
    // entries stay tracked for a retry.
    bool clear_plugin_state(slicing_step_t first_invalid_step);
    // Borrowed list used by the orchestrator to unwind plugins before their owners are replaced.
    const std::vector<PluginCleanupParticipant> &plugin_cleanup_participants() const
        { return m_plugin_cleanup_participants; }
    // Drop one participant after its clear callback succeeded.
    void erase_plugin_cleanup_participant(Plugin *plugin, slicing_step_t step);
    // Pipeline step currently running, or STEP_NONE outside the pipeline.
    slicing_step_t      running_step() const { return m_running_step.load(std::memory_order_acquire); }
    // Publishing a non-idle step starts a fresh execution attempt, including a retry of the same step.
    void                set_running_step(slicing_step_t step);
    // Atomically elect one reporter per attempt; only a non-idle step begin resets the claim, never cleanup.
    bool                claim_freeze_diagnostic() const { return !m_freeze_diagnostic_claimed.exchange(true, std::memory_order_relaxed); }
    // Checkpoint freeze contract: data owned by the steps up to
    // last_editor_step may be edited only by those steps, or outside the
    // pipeline (apply, tests). Host API debug asserts use this, for example
    // STEP_POST_SLICING for slices and STEP_FINALIZE_SLICING for the data tree.
    bool                running_step_may_edit(slicing_step_t last_editor_step) const;
    // Worker-owned or stopped-worker primitives; UI invalidation must use request_replay_from().
    void                mark_step_for_execution(slicing_step_t step);
    void                mark_step_and_dependents_for_execution(slicing_step_t step);
    bool                should_execute_step(slicing_step_t step) const;
    void                mark_step_executed(slicing_step_t step);
    void                reset_step_execution_plan_all();

    Steps::PipelineProgress       &pipeline_progress()       { return m_pipeline_progress; }
    const Steps::PipelineProgress &pipeline_progress() const { return m_pipeline_progress; }

    // Native G-code facade for warnings and statistics, not scheduling state.
    class StatusMonitor
    {
    private:
        Print& print;

    public:
        StatusMonitor(Print &print_mutable) : print(print_mutable) {}

        // need this extra method because active_step_add_warning is protected and so need the friend status, and Gcode has it.
        void active_step_add_warning(PrintStateBase::WarningLevel warning_level, const std::string &message, int message_id = 0)
        {
            print.active_step_add_warning(warning_level, message, message_id);
        }
        PrintStatistics &stats() { return print.m_print_statistics; }
        
    };

protected:
private:
    void                active_step_add_warning(PrintStateBase::WarningLevel warning_level, const std::string &message, int message_id = 0);
    void                add_step_warning(PrintStateBase::WarningLevel level, const std::string &message, int message_id);
    // Finalize the cached slicing-only first-layer hull from skirt coverage or first-layer islands.
    void                finalize_first_layer_convex_hull();
    // Non-cancelling scheduling invalidates downstream snapshots but retains the
    // live plan until plugins have released their borrowed handles.
    bool                request_steps_after(Steps::Checkpoint checkpoint);
    // Control paths already own the non-recursive state mutex.
    bool                mark_step_for_execution_unguarded(slicing_step_t step);
    // Queue one step without arming the plugin cleanup; state_mutex() must be held.
    bool                queue_step_unguarded(slicing_step_t step);
    bool                mark_step_and_dependents_for_execution_unguarded(slicing_step_t step);
    void                reset_step_execution_plan_all_unguarded();
    void                restore_metadata_checkpoint(Steps::Checkpoint checkpoint);
    void                forget_metadata_checkpoint_object(const PrintObject *object);

    // Islands of objects and their supports extruded at the 1st layer.
    Polygons            first_layer_islands() const;
    // Return 4 wipe tower corners in the world coordinates (shifted and rotated), including the wipe tower brim.
    // Collect scaled generic footprint corners for every configured tower.
    Points              first_layer_wipe_tower_corners() const;

    PrintConfig                             m_config;
    // Variable metadata shared by plugins and host producers for this Print.
    PrintRecordStore                        m_records;
    struct CheckpointMetadataSnapshot {
        PrintRecordStore::Snapshot records;
        std::map<const PrintObject *, PluginPropertyContainer> object_properties;
    };
    std::unique_ptr<CheckpointMetadataSnapshot> m_none_metadata_checkpoint;
    std::unique_ptr<CheckpointMetadataSnapshot> m_slicing_metadata_checkpoint;
    PrintObjectConfig                       m_default_object_config;
    PrintRegionConfig                       m_default_region_config;
    PrintObjectUPtrs                        m_objects;
    // Hidden owner for print-level auxiliary layers such as global skirt, brim
    // or wipe-tower helper geometry. It is not part of objects(), so normal
    // model slicing, validation and GUI object counts keep seeing only real
    // model-derived PrintObjects.
    std::unique_ptr<PrintObject>             m_auxiliary_object;
    // print regions are stored in PrintObjectRegions, here it's a shortcut
    // Historical ownership is separate from the active catalogue below.
    std::mutex m_region_registry_mutex;
    std::map<size_t, std::vector<std::unique_ptr<const PrintRegion>>> m_region_registry;
    PrintRegionPtrs                         m_print_regions;

    /*
    Deprecated read-through caches for old G-code and tests that still expect
    print.brim()/skirt() to return an ExtrusionEntityCollection reference.
    They are rebuilt from auxiliary layers on every accessor call and must
    never be used as the canonical storage for new brim/skirt output.
    */
    mutable std::optional<ExtrusionEntityCollection> m_legacy_skirt_first_layer_cache;
    mutable ExtrusionEntityCollection               m_legacy_skirt_cache;
    mutable ExtrusionEntityCollection               m_legacy_brim_cache;
    // Convex hull of the 1st layer extrusions.
    // It encompasses the object extrusions, support extrusions, skirt, brim, wipe tower.
    // It does NOT encompass user extrusions generated by custom G-code,
    // therefore it does NOT encompass the initial purge line.
    // It does NOT encompass MMU/MMU2 starting (wipe) areas.
    Polygon                                 m_first_layer_convex_hull;
    Points                                  m_skirt_convex_hull;

    // Following section will be consumed by the GCodeGenerator.
    std::vector<ToolOrdering>               m_tool_orderings;
    std::unique_ptr<Printing::PrintingPlan> m_printing_plan;
    // Checkpoint::Ordering copy. It owns its plan trees and metadata but
    // borrows source objects, layers and region-islands from this Print.
    std::unique_ptr<Printing::PrintingPlan> m_ordering_checkpoint;
    bool                                    m_keep_ordering_checkpoint = true;
    // Written by the pipeline thread between steps only; read by host API
    // debug checks while the step runs.
    std::atomic<slicing_step_t>             m_running_step{STEP_NONE};
    struct PipelineStepState {
        bool done = false;
        PrintStateBase::TimeStamp timestamp = 0;
        std::vector<PrintStateBase::Warning> warnings;
    };
    std::map<slicing_step_t, PipelineStepState> m_pipeline_state;
    mutable std::atomic_bool                m_freeze_diagnostic_claimed{false};
    std::string                             m_gcode_reader_source_path;
    std::string                             m_gcode_reader_plugin_id;
    // Pending export request and last reported export result, both guarded by state_mutex().
    std::optional<ExportRequest>            m_export_request;
    ExportResult                            m_export_result;

    // Estimated print time, filament consumed.
    PrintStatistics                         m_print_statistics;
    // time of last change, used by the gui to see if it needs to be updated
    std::time_t                             m_timestamp_last_change;

    // Cache to store sequential print clearance contours
    Polygons m_sequential_print_clearance_contours;

    // Steps that the next StepPipeline::run() should execute. The set is owned
    // by Print because config changes arrive here, while StepPipeline only
    // knows how to consume the plan in its fixed written order.
    std::set<slicing_step_t> m_steps_to_execute;
    std::vector<PluginCleanupParticipant>     m_plugin_cleanup_participants;
    // Tracked caches may belong to valid in-flight work; their presence alone must not trigger UI cleanup.
    std::atomic_bool                         m_plugin_cleanup_pending { false };
    size_t                                    m_next_plugin_setup_order = 0;

    Steps::PipelineProgress                    m_pipeline_progress;

    // To allow GCode to set the Print's GCodeExport step status.
    //friend class GCodeGenerator;
    // To allow GCodeProcessor to emit warnings.
    //friend class GCodeProcessor;
    // Allow PrintObject to access m_mutex and m_cancel_callback.
    friend class PrintObject;
    friend struct ApiInternal::PrintAccess;

    std::optional<ConflictResult> m_conflict_result;
};

} /* slic3r_Print_hpp_ */

#endif
