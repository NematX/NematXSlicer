///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Tomáš Mészáros @tamasmeszaros, Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Oleksandra Iushchenko @YuSanka, Lukáš Hejl @hejllukas, Filip Sykala @Jony01, Roman Beránek @zavorka, David Kocík @kocikdav
///|/ Copyright (c) BambuStudio 2023 manch1n @manch1n
///|/ Copyright (c) SuperSlicer 2023 Remi Durand @supermerill
///|/ Copyright (c) 2021 Martin Budden
///|/ Copyright (c) 2020 Paul Arden @ardenpm
///|/ Copyright (c) 2019 Thomas Moore
///|/ Copyright (c) 2019 Bryan Smith
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2014 Petr Ledvina @ledvinap
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

Print owns slicing state and validates placement. Tower envelopes are config-only
square-plus-brim polygons; instance towers follow ModelInstance translations
without rotation. They contribute to the export-time hull and owner clearance;
the cached first-layer hull contains only slicing geometry. Slicing also caches
tool ordering for preview ticks and legacy export. Statistics use measured
export totals and the wipe-tower share for sidebar and placeholders.
Validation checks print-level active settings against their selected plugin
group providers; object/region overrides are not inspected by that guard.

Representative direct calls (branches are not a timeline):

    Print::process()
        |-- slice()
        |-- this->sort_object_instances_by_model_order()
        |-- chain_print_object_instances()
        `-- assign_custom_gcodes()

    Print::validate()
        |-- Steps::unsupported_active_setting()
        |-- sequential_print_horizontal_clearance_valid()
        |-- sequential_print_vertical_clearance_valid()
        |-- PrintObject::update_layer_height_profile()
        |-- generate_object_layers()
        `-- PrintRegion::collect_object_printing_extruders()
*/
#include "Print.hpp"
#include "WipeTowerFootprint.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <limits>
#include <string>
#include <unordered_set>

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/log/trivial.hpp>
#include <boost/regex.hpp>

#include <oneapi/tbb/parallel_for.h>

#include "BoundingBox.hpp"
#include "NativeAdhesionLayerHelpers.hpp"
#include "Api/internal/PrintAccess.hpp"
#include "Api/internal/PrintObjectAccess.hpp"
#include "Api/host/Orchestrator.hpp"
#include "Api/host/Plugin.hpp"
#include "Brim.hpp"
#include "BuildVolume.hpp"
#include <clipper/clipper_z.hpp>
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "Extruder.hpp"
#include "ExtrusionEntityVisitors.hpp"
#include "Fill/FillBase.hpp"
#include "Flow.hpp"
#include "format.hpp"
#include "GCode.hpp"
#include "GCode/ConflictChecker.hpp"
#include "GCode/GCodeProcessor.hpp"
#include "GCode/ToolOrdering.hpp"
#include "Geometry/ConvexHull.hpp"
#include "I18N.hpp"
#include "PluginProperty.hpp"
#include "PrintObject.hpp"
#include "PrintObjectRegion.hpp"
#include "PrintRegion.hpp"
#include "Printing/PrintingPlan.hpp"
#include "ShortestPath.hpp"
#include "Steps/StepPipeline.hpp"
#include "Thread.hpp"
#include "Utils.hpp"

namespace Slic3r {

Print::Print() : m_pipeline_progress(this->status())
{
    // Create config hierarchy.
    m_default_object_config.parent = &m_config;
    m_default_region_config.parent = &m_default_object_config;
    this->reset_step_execution_plan_all();
}

Print::~Print()
{
    try {
        this->clear();
    } catch (const std::exception &error) {
        BOOST_LOG_TRIVIAL(error) << "Print cleanup failed during destruction: " << error.what();
    }
}

Printing::PrintingPlan &Print::mutable_printing_plan()
{
    // STEP_ORDERING owns this work copy for the current process() run. Create
    // it lazily so tests and legacy code that do not run ordering do not pay
    // for an empty plan allocation.
    if (!m_printing_plan)
        m_printing_plan = std::make_unique<Printing::PrintingPlan>();
    return *m_printing_plan;
}

void Print::reset_printing_plan()
{
    m_printing_plan.reset();
    clear_ordering_checkpoint();
}

void Print::release_live_printing_plan()
{
    m_printing_plan.reset();
}

void Print::set_keep_ordering_checkpoint(const bool keep)
{
    if (m_keep_ordering_checkpoint == keep)
        return;
    m_keep_ordering_checkpoint = keep;
    if (!keep)
        clear_ordering_checkpoint();
}

bool Print::save_ordering_checkpoint()
{
    if (!m_keep_ordering_checkpoint || m_printing_plan == nullptr) {
        clear_ordering_checkpoint();
        return false;
    }

    m_ordering_checkpoint = m_printing_plan->clone();
    return true;
}

bool Print::has_ordering_checkpoint() const
{
    return m_keep_ordering_checkpoint && m_ordering_checkpoint != nullptr;
}

bool Print::restore_ordering_checkpoint()
{
    if (!has_ordering_checkpoint())
        return false;

    // Clone rather than consume the copy so another invalidation can restore
    // the same checkpoint after this working copy has been edited.
    m_printing_plan = m_ordering_checkpoint->clone();
    return true;
}

void Print::clear_ordering_checkpoint()
{
    // The copy's layer and region-island pointers borrow from this Print.
    // Release it before its source objects can be replaced.
    m_ordering_checkpoint.reset();
}

// Take ownership of path and reader ID strings for a completed readable export;
// no file is opened or retained here, and invalidation clears both values.
void Print::set_gcode_reader_source_path(std::string path, std::string reader_plugin_id)
{
    m_gcode_reader_source_path = std::move(path);
    m_gcode_reader_plugin_id = std::move(reader_plugin_id);
}

void Print::save_metadata_checkpoint(const Steps::Checkpoint checkpoint)
{
    assert(checkpoint == Steps::Checkpoint::None || checkpoint == Steps::Checkpoint::Slicing);
    std::unique_ptr<CheckpointMetadataSnapshot> snapshot = std::make_unique<CheckpointMetadataSnapshot>();
    snapshot->records = m_records.capture();
    for (const PrintObject &object : this->objects())
        snapshot->object_properties.emplace(&object, static_cast<const PluginPropertyContainer &>(object));
    // Publish only a complete copy; a failed capture must not replace the retry baseline.
    (checkpoint == Steps::Checkpoint::None ? m_none_metadata_checkpoint : m_slicing_metadata_checkpoint) = std::move(snapshot);
}

bool Print::has_metadata_checkpoint(const Steps::Checkpoint checkpoint) const
{
    assert(checkpoint == Steps::Checkpoint::None || checkpoint == Steps::Checkpoint::Slicing);
    return (checkpoint == Steps::Checkpoint::None ? m_none_metadata_checkpoint : m_slicing_metadata_checkpoint) != nullptr;
}

void Print::restore_metadata_checkpoint(const Steps::Checkpoint checkpoint)
{
    assert(checkpoint == Steps::Checkpoint::None || checkpoint == Steps::Checkpoint::Slicing);
    const CheckpointMetadataSnapshot *snapshot =
        (checkpoint == Steps::Checkpoint::None ? m_none_metadata_checkpoint : m_slicing_metadata_checkpoint).get();
    // Without a checkpoint baseline, leave current metadata untouched.
    if (snapshot == nullptr)
        return;
    m_records.restore(snapshot->records);
    for (PrintObject &object : this->objects()) {
        const std::map<const PrintObject *, PluginPropertyContainer>::const_iterator it = snapshot->object_properties.find(&object);
        // Objects absent from this baseline have no properties to preserve.
        if (it == snapshot->object_properties.end())
            object.clear_properties();
        else
            object.copy_properties_from(it->second);
    }
}

void Print::forget_metadata_checkpoint_object(const PrintObject *object)
{
    // Erase before deletion so a new object at the same address cannot inherit an old baseline.
    if (m_none_metadata_checkpoint != nullptr)
        m_none_metadata_checkpoint->object_properties.erase(object);
    if (m_slicing_metadata_checkpoint != nullptr)
        m_slicing_metadata_checkpoint->object_properties.erase(object);
}

// Clear the owned path and reader ID without removing the producer's file;
// future reads require a fresh publication and no ownership is transferred.
void Print::reset_gcode_reader_source_path()
{
    m_gcode_reader_source_path.clear();
    m_gcode_reader_plugin_id.clear();
}

// Own complete immutable snapshots until clear(). The mutex serializes only
// registry lookup/insertion; apply/clear and changes to defaults require idle workers.
const PrintRegion &Print::register_print_region(const PrintRegionConfig &config)
{
    const size_t hash = config.hash();
    std::lock_guard<std::mutex> lock(m_region_registry_mutex);
    std::vector<std::unique_ptr<const PrintRegion>> &bucket = m_region_registry[hash];
    for (const std::unique_ptr<const PrintRegion> &region : bucket)
        if (region->config() == config)
            return *region;
    bucket.push_back(std::make_unique<const PrintRegion>(config, hash));
    return *bucket.back();
}

// Validation and copying finish before interning, so a rejected partial config
// cannot leave a partially populated snapshot in the registry.
const PrintRegion *Print::register_print_region_config(const ConfigBase &config)
{
    PrintRegionConfig complete = m_default_region_config;
    for (const std::string &key : config.keys()) {
        const ConfigOption *source = config.optptr(key);
        ConfigOption *target = complete.optptr(key, true);
        if (source == nullptr || target == nullptr || source->type() != target->type())
            return nullptr;
        target->set(*source);
    }
    return &register_print_region(complete);
}

// Registered historical configurations are deliberately absent here. Traverse
// active associations in model order, with the hidden auxiliary object last.
void Print::rebuild_active_print_regions()
{
    m_print_regions.clear();
    std::set<const PrintRegion*> seen;
    const auto append_object = [this, &seen](const PrintObject &object) {
        for (const PrintRegion &region : object.all_regions())
            if (seen.insert(&region).second)
                m_print_regions.push_back(&region);
    };
    for (const PrintObjectUPtr &object : m_objects)
        if (object->shared_regions() != nullptr)
            append_object(*object);
    if (m_auxiliary_object != nullptr)
        append_object(*m_auxiliary_object);
}

PrintObject &Print::mutable_auxiliary_object()
{
    if (m_auxiliary_object)
        return *m_auxiliary_object;

    /*
    Print-level helper geometry needs an Object owner because Layer, LayerRegion
    and LayerIsland are all object-scoped. The auxiliary object is deliberately
    hidden from m_objects, so normal model slicing and GUI object counts do not
    see it. Plugins reach it through an explicit data-tree API.
    */
    std::shared_ptr<PrintObjectRegions> shared_regions = std::make_shared<PrintObjectRegions>();
    shared_regions->all_regions.push_back(&register_print_region(m_default_region_config));
    for (const PrintRegion *region : m_print_regions)
        if (region != shared_regions->all_regions.front())
            shared_regions->all_regions.push_back(region);

    /*
    The hidden object covers the print bed in XY for diagnostics and future
    placement logic. The auxiliary layers themselves are still built from their
    explicit 2D subjects, not from this bounding box.
    */
    const Points bed_shape = get_bed_shape(m_config);
    const BoundingBox bed_bbox = bed_shape.empty() ? BoundingBox{} : get_extents(bed_shape);
    const Vec3crd size(
        bed_bbox.size().x(),
        bed_bbox.size().y(),
        scale_i(m_config.max_print_height.value));

    m_auxiliary_object.reset(new PrintObject(this, size, std::move(shared_regions)));
    rebuild_active_print_regions();
    return *m_auxiliary_object;
}

void Print::reset_auxiliary_object()
{
    m_auxiliary_object.reset();
    rebuild_active_print_regions();
}

void Print::set_task(const TaskParams &)
{
    std::scoped_lock<std::mutex> lock(this->state_mutex());
    this->call_cancel_callback();
}

void Print::finalize() {}

void Print::clear() {
    std::unique_lock<std::mutex> lock(this->state_mutex());
    // The GUI stop callback needs this lock; plugin cleanup must run after the lock is released.
    this->call_cancel_callback();
    lock.unlock();
    // A full reset must remain possible even when a plugin cannot release its private cache.
    if (!this->clear_plugin_state(STEP_LAYER_HEIGHT))
        BOOST_LOG_TRIVIAL(error) << "Discarding failed plugin cleanup participants while resetting Print.";
    lock.lock();
    // The worker is already stopped; reset its bookkeeping before destroying the retained owners.
    // Plans borrow islands and layers; release them before their owners.
    this->reset_printing_plan();
    this->reset_gcode_reader_source_path();
    this->reset_step_execution_plan_all_unguarded();
    m_pipeline_state.clear();
    m_running_step.store(STEP_NONE, std::memory_order_release);
    m_none_metadata_checkpoint.reset();
    m_slicing_metadata_checkpoint.reset();
    m_objects.clear();
    this->reset_auxiliary_object();
    m_print_regions.clear();
    m_model.clear_objects();
    m_records.clear();
    m_plugin_cleanup_participants.clear();
    m_plugin_cleanup_pending.store(false, std::memory_order_release);
    m_next_plugin_setup_order = 0;
    std::lock_guard<std::mutex> registry_lock(m_region_registry_mutex);
    m_region_registry.clear();
}

bool Print::request_replay_from(const Steps::Checkpoint checkpoint)
{
    if (checkpoint == Steps::Checkpoint::GCode)
        return false;
    // The stop callback may clear the running step while it releases this mutex.
    const slicing_step_t running = running_step();
    const bool running_affected = running != STEP_NONE &&
        !Steps::executes_before(running, Steps::checkpoint_resume_step(checkpoint));
    // The worker does not lock the step set or snapshot pointer; wait for it before touching either.
    this->call_cancel_callback();
    const bool affected = this->request_steps_after(checkpoint);
    if (checkpoint == Steps::Checkpoint::None)
        for (PrintObjectUPtr &object : m_objects)
            // apply() publishes new objects before creating their lazy slicing parameters.
            if (object->m_slicing_params != nullptr)
                object->m_slicing_params->valid = false;
    return affected || running_affected;
}

bool Print::request_replay_from_worker(const Steps::Checkpoint checkpoint)
{
    if (checkpoint == Steps::Checkpoint::GCode)
        return false;
    // Protect state readers without invoking the UI callback that would wait for this same worker.
    std::scoped_lock<std::mutex> lock(this->state_mutex());
    const bool affected = this->request_steps_after(checkpoint);
    // The worker owns this replay; an unchanged UI apply must not interrupt its preparation.
    m_plugin_cleanup_pending.store(false, std::memory_order_release);
    return affected;
}

Steps::Checkpoint Print::current_checkpoint() const
{
    for (const slicing_step_t step : Steps::execution_order()) {
        if (!should_execute_step(step))
            continue;
        const Steps::Checkpoint checkpoint = Steps::checkpoint_before_step(step);
        // Both plan checkpoints need their plan: the retained Ordering copy, or
        // the live plan that STEP_GCODE reads. Without it, rebuild the plan
        // from the frozen Print data tree.
        if (checkpoint == Steps::Checkpoint::Ordering && !has_ordering_checkpoint())
            return Steps::Checkpoint::Print;
        if (checkpoint == Steps::Checkpoint::Plan && m_printing_plan == nullptr)
            return Steps::Checkpoint::Print;
        return checkpoint;
    }
    return Steps::Checkpoint::GCode;
}

void Print::prepare_checkpoint_resume()
{
    const Steps::Checkpoint checkpoint = current_checkpoint();
    if (checkpoint == Steps::Checkpoint::GCode)
        return;
    // An interrupted attempt may have finished part of the suffix; all of it must become pending again.
    this->request_replay_from_worker(checkpoint);
    // apply() already notified plugins and destroyed invalidated outputs. Doing it again here
    // is a no-op in that case, but it also covers a canceled run that produced part of the
    // suffix and invalidations made outside apply().
    // Plugins must drop their borrowed handles before the host destroys or replaces those owners.
    if (!this->clear_plugin_state(Steps::checkpoint_resume_step(checkpoint)))
        throw PluginCleanupError("Plugin cleanup failed before checkpoint resume.");
    Steps::StepPipeline::clear_checkpoint_outputs(*this, checkpoint);
    if (checkpoint == Steps::Checkpoint::None || checkpoint == Steps::Checkpoint::Slicing)
        this->restore_metadata_checkpoint(checkpoint);
    if (checkpoint == Steps::Checkpoint::Ordering) {
        const bool restored = this->restore_ordering_checkpoint();
        assert(restored);
        (void)restored;
    }
    m_plugin_cleanup_pending.store(false, std::memory_order_release);
}

void Print::track_plugin_setup(Plugin &plugin, const slicing_step_t step)
{
    // A setup may serve many object/layer runs, but cleanup is once per plugin, step and Print.
    for (const PluginCleanupParticipant &participant : m_plugin_cleanup_participants)
        if (participant.plugin == &plugin && participant.step == step)
            return;

    m_plugin_cleanup_participants.push_back({&plugin, step, m_next_plugin_setup_order++});
}

bool Print::clear_plugin_state(const slicing_step_t first_invalid_step)
{
    const bool succeeded = m_plugin_cleanup_participants.empty() ||
        Orchestrator::instance().clear_plugin_participants(*this, first_invalid_step);
    // Keep failures pending even if apply() already consumed the changed inputs before it failed.
    m_plugin_cleanup_pending.store(!succeeded, std::memory_order_release);
    return succeeded;
}

// Drop the entry of a plugin whose clear callback succeeded, so the remaining
// entries are exactly the ones that still need a retry.
void Print::erase_plugin_cleanup_participant(Plugin *plugin, const slicing_step_t step)
{
    m_plugin_cleanup_participants.erase(
        std::remove_if(m_plugin_cleanup_participants.begin(),
                       m_plugin_cleanup_participants.end(),
                       [plugin, step](const PluginCleanupParticipant &participant) {
                           return participant.plugin == plugin && participant.step == step;
                       }),
        m_plugin_cleanup_participants.end());
}

bool Print::running_step_may_edit(const slicing_step_t last_editor_step) const
{
    const slicing_step_t running = running_step();
    return running == STEP_NONE || !Steps::executes_before(last_editor_step, running);
}

bool Print::request_steps_after(const Steps::Checkpoint checkpoint)
{
    assert(checkpoint != Steps::Checkpoint::GCode);
    // Release only the stored Ordering copy: the live plan still owns data that
    // plugins may hold handles to, until their cleanup callbacks have run.
    if (checkpoint <= Steps::Checkpoint::Print)
        this->clear_ordering_checkpoint();
    if (checkpoint == Steps::Checkpoint::None)
        m_slicing_metadata_checkpoint.reset();
    return this->mark_step_and_dependents_for_execution_unguarded(Steps::checkpoint_resume_step(checkpoint));
}

void Print::mark_step_for_execution(slicing_step_t step)
{
    std::scoped_lock<std::mutex> lock(state_mutex());
    mark_step_for_execution_unguarded(step);
}

bool Print::mark_step_for_execution_unguarded(slicing_step_t step)
{
    if (step == STEP_NONE)
        return false;
    PipelineStepState &state = m_pipeline_state[step];
    const bool affected = state.done || running_step() == step;
    const bool warnings_changed = std::any_of(state.warnings.begin(), state.warnings.end(),
        [](const PrintStateBase::Warning &warning) { return warning.current; });
    if (state.done || warnings_changed) {
        state.done = false;
        for (PrintStateBase::Warning &warning : state.warnings)
            warning.current = false;
        state.timestamp = PrintStateBase::next_timestamp();
    }
    m_steps_to_execute.insert(step);
    // The step may reuse owners that plugin caches still hold, so the next
    // apply() or resume must offer a clear callback before running it.
    m_plugin_cleanup_pending.store(true, std::memory_order_release);
    return affected;
}

void Print::mark_step_and_dependents_for_execution(slicing_step_t step)
{
    std::scoped_lock<std::mutex> lock(state_mutex());
    mark_step_and_dependents_for_execution_unguarded(step);
}

bool Print::mark_step_and_dependents_for_execution_unguarded(slicing_step_t step)
{
    if (step == STEP_NONE)
        return false;
    if (step == STEP_ANY) {
        bool affected = running_step() != STEP_NONE;
        for (const std::pair<const slicing_step_t, PipelineStepState> &entry : m_pipeline_state)
            affected |= entry.second.done;
        this->reset_step_execution_plan_all_unguarded();
        return affected;
    }

    bool affected = this->mark_step_for_execution_unguarded(step);
    for (slicing_step_t dependent : Steps::dependent_steps_closure(step))
        affected |= this->mark_step_for_execution_unguarded(dependent);
    return affected;
}

bool Print::should_execute_step(slicing_step_t step) const
{
    return m_steps_to_execute.find(step) != m_steps_to_execute.end();
}

void Print::mark_step_executed(slicing_step_t step)
{
    bool removed_warnings = false;
    {
        std::scoped_lock<std::mutex> lock(state_mutex());
        this->throw_if_canceled();
        PipelineStepState &state = m_pipeline_state[step];
        const size_t count = state.warnings.size();
        state.warnings.erase(std::remove_if(state.warnings.begin(), state.warnings.end(),
            [](const PrintStateBase::Warning &warning) { return !warning.current; }), state.warnings.end());
        removed_warnings = count != state.warnings.size();
        if (!state.done || removed_warnings)
            state.timestamp = PrintStateBase::next_timestamp();
        state.done = true;
        m_steps_to_execute.erase(step);
    }
    // Notification callbacks may read the state again, so the mutex is released first.
    if (removed_warnings)
        status_update_warnings(int(step), PrintStateBase::WarningLevel::NON_CRITICAL, std::string());
}

void Print::reset_step_execution_plan_all()
{
    std::scoped_lock<std::mutex> lock(state_mutex());
    reset_step_execution_plan_all_unguarded();
}

void Print::reset_step_execution_plan_all_unguarded()
{
    m_pipeline_progress.reset();
    clear_ordering_checkpoint();
    m_slicing_metadata_checkpoint.reset();
    m_steps_to_execute.clear();
    for (slicing_step_t step : Steps::execution_order())
        mark_step_for_execution_unguarded(step);
    // A full replay can reuse every owner, so plugin caches must be cleared first.
    m_plugin_cleanup_pending.store(true, std::memory_order_release);
}

void Print::set_running_step(slicing_step_t step)
{
    // Freeze checks can borrow the atomic step while control paths already own this mutex.
    std::scoped_lock<std::mutex> lock(state_mutex());
    const slicing_step_t previous = running_step();
    if (step != STEP_NONE) {
        PipelineStepState &state = m_pipeline_state[step];
        state.done = false;
        for (PrintStateBase::Warning &warning : state.warnings)
            warning.current = false;
        state.timestamp = PrintStateBase::next_timestamp();
        m_freeze_diagnostic_claimed.store(false, std::memory_order_relaxed);
    } else if (previous != STEP_NONE) {
        m_pipeline_state.at(previous).timestamp = PrintStateBase::next_timestamp();
    }
    m_running_step.store(step, std::memory_order_release);
}

PrintStateBase::StateWithWarnings Print::step_state_with_warnings(slicing_step_t step) const
{
    std::scoped_lock<std::mutex> lock(state_mutex());
    PrintStateBase::StateWithWarnings result;
    const std::map<slicing_step_t, PipelineStepState>::const_iterator it = m_pipeline_state.find(step);
    if (it != m_pipeline_state.end()) {
        result.state = it->second.done ? PrintStateBase::State::Done : PrintStateBase::State::Fresh;
        result.timestamp = it->second.timestamp;
        result.warnings = it->second.warnings;
    }
    if (step != STEP_NONE && running_step() == step)
        result.state = PrintStateBase::State::Started;
    return result;
}

bool Print::slices_ready() const
{
    return is_step_done(STEP_FINALIZE_SLICING);
}

void Print::add_step_warning(PrintStateBase::WarningLevel level, const std::string &message, int message_id)
{
    slicing_step_t step;
    bool changed = false;
    {
        std::scoped_lock<std::mutex> lock(state_mutex());
        step = running_step();
        assert(step != STEP_NONE);
        PipelineStepState &state = m_pipeline_state.at(step);
        std::vector<PrintStateBase::Warning>::iterator it = std::find_if(state.warnings.begin(), state.warnings.end(),
            [&](const PrintStateBase::Warning &warning) {
                return message_id == 0 ? warning.message_id == 0 && warning.message == message : warning.message_id == message_id;
            });
        if (it == state.warnings.end()) {
            state.warnings.push_back({level, true, message, message_id});
            changed = true;
        } else if (!it->current || it->level != level || it->message != message) {
            *it = {level, true, message, message_id};
            changed = true;
        }
        if (changed)
            state.timestamp = PrintStateBase::next_timestamp();
    }
    // Warning delivery can synchronously invoke a state reader.
    if (changed)
        status_update_warnings(int(step), level, message);
}

const PrintObject *Print::get_print_object_by_model_object_id(ObjectID object_id) const {
    auto it = std::find_if(m_objects.begin(), m_objects.end(), [object_id](const PrintObjectUPtr &obj) {
        return obj->model_object()->id() == object_id;
    });
    return (it == m_objects.end()) ? nullptr : it->get();
}

const PrintObject *Print::get_object(ObjectID object_id) const {
    auto it = std::find_if(m_objects.begin(), m_objects.end(),
                           [object_id](const PrintObjectUPtr &obj) { return obj->id() == object_id; });
    return (it == m_objects.end()) ? nullptr : it->get();
}

// All FFF config scopes use the same declared first-consumer checkpoint.
bool Print::invalidate_state_by_config_options(const ConfigOptionResolver & /* new_config */,
                                               const std::vector<t_config_option_key> &opt_keys) {
    if (opt_keys.empty())
        return false;

    bool invalidated = false;
    for (const t_config_option_key &opt_key : opt_keys) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(opt_key);
        if (def == nullptr) {
            invalidated |= this->request_replay_from(Steps::Checkpoint::None);
            continue;
        }
        if (def->invalidates_step == STEP_NONE)
            continue;
        invalidated |= this->request_replay_from(Steps::checkpoint_before_step(def->invalidates_step));
    }
    if (invalidated)
        m_timestamp_last_change = std::time(0);
    return invalidated;
}

bool Print::is_step_done(slicing_step_t step) const
{
    return step_state_with_warnings(step).is_done();
}

PrintStateBase::StateWithTimeStamp Print::step_state_with_timestamp(slicing_step_t step) const
{
    return step_state_with_warnings(step);
}

void Print::active_step_add_warning(PrintStateBase::WarningLevel warning_level, const std::string &message, int message_id)
{
    if (running_step() != STEP_NONE)
        add_step_warning(warning_level, message, message_id);
    else
        Orchestrator::instance().add_plugin_message(
            warning_level == PrintStateBase::WarningLevel::CRITICAL ?
                Orchestrator::PluginMessageLevel::Error : Orchestrator::PluginMessageLevel::Warning,
            nullptr, STEP_NONE, message.c_str());
}

// Collect zero-based object/support tools at scaled Z (negative means all layers) and the global tower contour tool; return an owned set, retain source ownership, and propagate invalid-index/allocation failures.
std::set<extruder_index_t> Print::object_extruders(const PrintObjectPtrs &objects, coord_t z /*= -1*/) const
{
    std::set<extruder_index_t> extruders;
    for (const PrintObject *object : objects) {
        std::vector<std::reference_wrapper<const PrintRegion>> ok_regions;
        if (z < 0) {
            ok_regions = object->all_regions();
        } else {
            std::set<const PrintRegion*> region_set;
            for (const Layer &layer : object->layers()) {
                if ((layer.scaled_bottom_z()) <= z && z <= layer.scaled_print_z() ) {
                    for (const LayerSliceIsland &layer_island_ptr : layer.islands()) {
                        for (const LayerRegionIsland &region_island_ptr : layer_island_ptr.regions_islands()) {
                            if (region_island_ptr.has_extrusions()) {
                                for (const LayerRegion *lr : region_island_ptr.regions()) {
                                    region_set.insert(&lr->region());
                                }
                            }
                        }
                    }
                }
            }
            for (const PrintRegion *lr : region_set) {
                ok_regions.emplace_back(*lr);
            }
            assert(ok_regions.size() == region_set.size());
        }
        for (const PrintRegion &region : ok_regions) {
            region.collect_object_printing_extruders(*object->print(), extruders);
        }
    }
    return extruders;
}
std::set<extruder_index_t> Print::object_extruders(coord_t z /*= -1*/) const
{
    PrintObjectPtrs objects;
    objects.reserve(m_objects.size());
    for (const PrintObjectUPtr &object : m_objects)
        objects.emplace_back(object.get());
    return object_extruders(objects, z);
}

// returns 0-based indices of used extruders
std::set<extruder_index_t> Print::support_material_extruders(coord_t z /*= -1*/) const
{
    std::set<extruder_index_t> extruders;
    bool support_uses_current_extruder = false;
    const size_t num_extruders = m_config.nozzle_diameter.size();

    for (const PrintObjectUPtr &object : m_objects) {
        if (object->has_support_material()) {
            bool has_support = true;
            bool has_support_interface = object->config().support_material_interface_layers > 0;
            if (z >= 0) {
                has_support = false;
                for (const Layer &suppl : object->auxiliary_layers()) {
                    if (suppl.get_property<LayerSupportProperty>() != nullptr &&
                        suppl.scaled_bottom_z() <= z &&
                        z <= suppl.scaled_print_z() &&
                        suppl.has_extrusions()) {
                        has_support = true;
                    }
                }
            }
            if (has_support) {
                assert(object->config().support_material_extruder >= 0);
                if (object->config().support_material_extruder == 0)
                    support_uses_current_extruder = true;
                else {
                    const size_t index = size_t(object->config().support_material_extruder.value - 1);
                    if (index > size_t(std::numeric_limits<extruder_index_t>::max()))
                        throw std::invalid_argument("A support extruder index exceeds extruder_index_t.");
                    extruders.insert(static_cast<extruder_index_t>(index >= num_extruders ? 0 : index));
                }
            }
            if (has_support && has_support_interface) {
                assert(object->config().support_material_interface_extruder >= 0);
                if (object->config().support_material_interface_extruder == 0)
                    support_uses_current_extruder = true;
                else {
                    const size_t index = size_t(object->config().support_material_interface_extruder.value - 1);
                    if (index > size_t(std::numeric_limits<extruder_index_t>::max()))
                        throw std::invalid_argument("A support interface extruder index exceeds extruder_index_t.");
                    extruders.insert(static_cast<extruder_index_t>(index >= num_extruders ? 0 : index));
                }
            }
        }
    }

    if (support_uses_current_extruder)
        // Add all object extruders to the support extruders as it is not know which one will be used to print supports.
        append(extruders, this->object_extruders());
    
    return extruders;
}

// returns 0-based indices of used extruders
std::set<extruder_index_t> Print::extruders(coord_t z /*= -1*/) const
{
    std::set<extruder_index_t> extruders = this->object_extruders(z);
    append(extruders, this->support_material_extruders(z));

    if (z < 0) {
        // The wipe tower extruder can also be set. When the wipe tower is enabled and it will be generated,
        // append its extruder into the list too.
        if (has_global_wipe_tower() && m_default_object_config.wipe_tower_extruder != 0 && extruders.size() > 1) {
            assert(m_default_object_config.wipe_tower_extruder > 0 &&
                   m_default_object_config.wipe_tower_extruder < int(m_config.nozzle_diameter.size()));
            const int32_t zero_based = m_default_object_config.wipe_tower_extruder.value - 1;
            if (zero_based > std::numeric_limits<extruder_index_t>::max())
                throw std::invalid_argument("The wipe tower extruder index exceeds extruder_index_t.");
            extruders.insert(static_cast<extruder_index_t>(zero_based)); // the config value is 1-based
        }
    }

    return extruders;
}

int32_t Print::first_layer_bed_temperature() const {
    if (m_config.print_first_layer_bed_temperature.is_enabled())
        return m_config.print_first_layer_bed_temperature.value;

    int32_t temperature = 0;
    for (extruder_index_t extruder_id : this->extruders(0))
        temperature = std::max(temperature, m_config.first_layer_bed_temperature.get_at(extruder_id));
    return temperature;
}

uint16_t Print::num_object_instances() const
{
    uint16_t instances = 0;
    for (const PrintObjectUPtr &print_object : m_objects)
        instances += (uint16_t)print_object->instances().size();
    return instances;
}

double Print::max_allowed_layer_height() const
{
    double nozzle_diameter_max = 0.;
    for (extruder_index_t extruder_id : this->extruders())
        nozzle_diameter_max = std::max(nozzle_diameter_max, m_config.nozzle_diameter.get_at(extruder_id));
    return nozzle_diameter_max;
}

std::vector<ObjectID> Print::print_object_ids() const 
{ 
    std::vector<ObjectID> out; 
    // Reserve one more for the caller to append the ID of the Print itself.
    out.reserve(m_objects.size() + 1);
    for (const PrintObjectUPtr &print_object : m_objects)
        out.emplace_back(print_object->id());
    return out;
}

bool Print::has_infinite_skirt() const
{
    return (m_config.draft_shield.value == dsEnabled && m_config.skirts > 0)/* || (m_config.ooze_prevention && this->extruders().size() > 1)*/;
}

bool Print::has_skirt() const
{
    return (m_config.skirt_height > 0 && m_config.skirts > 0) || this->has_infinite_skirt() || m_config.draft_shield.value != dsDisabled;
    // case dsLimited should only be taken into account when skirt_height and skirts are positive,
    // so it is covered by the first condition.
}

bool Print::has_brim() const
{
    const PrintObject *auxiliary_object = this->auxiliary_object();
    if (auxiliary_object != nullptr && has_auxiliary_layer(*auxiliary_object, layer_is_brim_adhesion))
        return true;
    return std::any_of(m_objects.begin(), m_objects.end(), [](const PrintObjectUPtr &object) { return object->has_brim(); });
}

const ExtrusionEntityCollection &Print::brim() const
{
    m_legacy_brim_cache.clear();
    const PrintObject *auxiliary_object = this->auxiliary_object();
    if (auxiliary_object != nullptr)
        collect_auxiliary_layer_extrusions(*auxiliary_object, layer_is_brim_adhesion, m_legacy_brim_cache);
    return m_legacy_brim_cache;
}

const ExtrusionEntityCollection &Print::skirt() const
{
    m_legacy_skirt_cache.clear();
    const PrintObject *auxiliary_object = this->auxiliary_object();
    if (auxiliary_object != nullptr)
        collect_auxiliary_layer_extrusions(*auxiliary_object, layer_is_normal_skirt_adhesion, m_legacy_skirt_cache);
    return m_legacy_skirt_cache;
}

const std::optional<ExtrusionEntityCollection> &Print::skirt_first_layer() const
{
    m_legacy_skirt_first_layer_cache.reset();
    const PrintObject *auxiliary_object = this->auxiliary_object();
    if (auxiliary_object != nullptr && has_auxiliary_layer(*auxiliary_object, layer_is_first_layer_skirt_adhesion)) {
        m_legacy_skirt_first_layer_cache.emplace();
        collect_auxiliary_layer_extrusions(
            *auxiliary_object, layer_is_first_layer_skirt_adhesion, *m_legacy_skirt_first_layer_cache);
    }
    return m_legacy_skirt_first_layer_cache;
}

// Borrow the Print and compare clearance-expanded instance hulls plus their tower envelopes; optionally copy colliding polygons, preserve ownership, and propagate geometry/allocation failures.
bool Print::sequential_print_horizontal_clearance_valid(const Print &print, Polygons* polygons)
{
    if (print.config().extruder_clearance_radius == 0) {
        return true;
    }
    Polygons convex_hulls_other;
    const std::vector<WipeTowerFootprint> towers = wipe_tower_footprints(print);
    if (polygons != nullptr) {
        polygons->clear();
    }
    std::vector<size_t> intersecting_idxs;

	std::map<ObjectID, Polygon> map_model_object_to_convex_hull;
    const double dist_grow = min_object_distance(static_cast<const ConfigBase*>(&print.full_print_config()), 0);
	for (const PrintObject &print_object : print.objects()) {
        const double object_grow = (print.config().complete_objects && !print_object.config().brim_per_object) ? dist_grow : std::max(dist_grow, print_object.config().brim_width.value);
	    assert(! print_object.model_object()->instances.empty());
	    assert(! print_object.instances().empty());
	    ObjectID model_object_id = print_object.model_object()->id();
	    auto it_convex_hull = map_model_object_to_convex_hull.find(model_object_id);
        // Get convex hull of all printable volumes assigned to this print object.
        ModelInstance *model_instance0 = print_object.model_object()->instances.front();
	      if (it_convex_hull == map_model_object_to_convex_hull.end()) {
	          // Calculate the convex hull of a printable object. 
	          // Grow convex hull with the clearance margin.
	          // FIXME: Arrangement has different parameters for offsetting (jtMiter, limit 2)
	          // which causes that the warning will be showed after arrangement with the
	          // appropriate object distance. Even if I set this to jtMiter the warning still shows up.
            Geometry::Transformation trafo = model_instance0->get_transformation();
            trafo.set_offset(Vec3d{ 0.0, 0.0, model_instance0->get_offset().z() });
            Polygon ch2d = print_object.model_object()->convex_hull_2d(trafo.get_matrix());
            Polygons offs_ch2d = offset(ch2d,
                // Shrink the extruder_clearance_radius a tiny bit, so that if the object arrangement algorithm placed the objects
                // exactly by satisfying the extruder_clearance_radius, this test will not trigger collision.
                scale_d(0.5 * object_grow - BuildVolume::BedEpsilon), jtRound, scale_d(0.1));
            // for invalid geometries the vector returned by offset() may be empty
            if (!offs_ch2d.empty())
                it_convex_hull = map_model_object_to_convex_hull.emplace_hint(it_convex_hull, model_object_id, offs_ch2d.front());
        }
        if (it_convex_hull != map_model_object_to_convex_hull.end()) {
            // Make a copy, so it may be rotated for instances.
            //FIXME seems like the rotation isn't taken into account
            Polygon convex_hull0 = it_convex_hull->second;
            //this can create bugs in macos, for reasons.
            const double z_diff = Geometry::rotation_diff_z(model_instance0->get_matrix(), print_object.instances().front().model_instance->get_matrix());
            if (std::abs(z_diff) > EPSILON)
                convex_hull0.rotate(z_diff);
            // Now we check that no instance of convex_hull intersects any of the previously checked object instances.
            for (const PrintInstance& instance : print_object.instances()) {
                Polygon convex_hull = convex_hull0;
                // instance.shift is a position of a centered object, while model object may not be centered.
                // Convert the shift from the PrintObject's coordinates into ModelObject's coordinates by removing the centering offset.
                convex_hull.translate(instance.shift - print_object.center_offset());
                for (const WipeTowerFootprint &tower : towers)
                    if (tower.print_object == &print_object && tower.instance == instance.model_instance) {
                        Points points = convex_hull.points;
                        const Polygons expanded = offset(tower.polygon, scale_d(0.5 * object_grow - BuildVolume::BedEpsilon), jtRound, scale_d(0.1));
                        for (const Polygon &polygon : expanded)
                            append(points, polygon.points);
                        convex_hull = Geometry::convex_hull(points);
                    }
                // if output needed, collect indices (inside convex_hulls_other) of intersecting hulls
                for (size_t i = 0; i < convex_hulls_other.size(); ++i) {
                    if (!intersection(convex_hulls_other[i], convex_hull).empty()) {
                        if (polygons == nullptr)
                            return false;
                        else {
                            intersecting_idxs.emplace_back(i);
                            intersecting_idxs.emplace_back(convex_hulls_other.size());
                        }
                    }
                }
                convex_hulls_other.emplace_back(std::move(convex_hull));
            }
        }
    }

    if (!intersecting_idxs.empty()) {
        // use collected indices (inside convex_hulls_other) to update output
        std::sort(intersecting_idxs.begin(), intersecting_idxs.end());
        intersecting_idxs.erase(std::unique(intersecting_idxs.begin(), intersecting_idxs.end()), intersecting_idxs.end());
        for (size_t i : intersecting_idxs) {
            polygons->emplace_back(std::move(convex_hulls_other[i]));
        }
        return false;
    }
    return true;
}

static inline bool sequential_print_vertical_clearance_valid(const Print &print)
{
	std::vector<const PrintInstance*> print_instances_ordered = print.sort_object_instances_by_model_order();
	// Ignore the last instance printed.
	print_instances_ordered.pop_back();
	// Find the other highest instance.
	auto it = std::max_element(print_instances_ordered.begin(), print_instances_ordered.end(), [](auto l, auto r) {
		return l->print_object->height() < r->print_object->height();
	});
    return it == print_instances_ordered.end() || (*it)->print_object->height() <= scale_i(print.config().extruder_clearance_height.value);
}

coord_t Print::get_object_first_layer_height(const PrintObject& object) const {
    //get object first layer height
    coord_t object_first_layer_height = scale_to_layer_coord(object.config().first_layer_height.value);
    if (object.config().first_layer_height.percent) {
        std::set<extruder_index_t> object_extruders;
        for (const PrintRegion& region : object.all_regions()) {
            PrintRegion::collect_object_printing_extruders(config(), object.config(), region.config(), object_extruders);
        }
        object_first_layer_height = 1000000000;
        for (extruder_index_t extruder_id : object_extruders) {
            const double nozzle_diameter = config().nozzle_diameter.get_at(extruder_id);
            const coord_t first_layer_height = scale_to_layer_coord(object.config().first_layer_height.get_effective_value(nozzle_diameter));
            object_first_layer_height = std::min(object_first_layer_height, first_layer_height);
        }
    }
    assert(object_first_layer_height < 1000000000);
    return object_first_layer_height;
}

coord_t Print::get_min_first_layer_height() const
{
    if (m_objects.empty())
        throw Slic3r::InvalidArgument("first_layer_height() can't be called without PrintObjects");

    coord_t min_layer_height = 10000000000;
    for(const PrintObjectUPtr &obj : m_objects)
        min_layer_height = std::min(min_layer_height, get_object_first_layer_height(*obj));

    if(min_layer_height == 10000000000)
        throw Slic3r::InvalidArgument("first_layer_height() can't be computed");

    return min_layer_height;
}

// Matches "G92 E0" with various forms of writing the zero and with an optional comment.
boost::regex regex_g92e0 { "^[ \\t]*[gG]92[ \\t]*[eE](0(\\.0*)?|\\.0+)[ \\t]*(;.*)?$" };

std::pair<PrintBase::PrintValidationError, std::string> Print::validate(std::vector<std::string>* warnings) const
{
    std::set<extruder_index_t> extruders = this->extruders();

    if (warnings) {
        for (auto it_a = extruders.begin(); it_a != extruders.end() ;++it_a)
            for (auto it_b = std::next(it_a); it_b != extruders.end() ;++it_b)
                if (std::abs(m_config.bed_temperature.get_at(*it_a) - m_config.bed_temperature.get_at(*it_b)) > 15
                 || std::abs(m_config.first_layer_bed_temperature.get_at(*it_a) - m_config.first_layer_bed_temperature.get_at(*it_b)) > 15) {
                    warnings->emplace_back("_BED_TEMPS_DIFFER");
                    goto DONE;
                }
        DONE:;
    }

    if (m_objects.empty())
        return { PrintBase::PrintValidationError::pveWrongPosition, _u8L("All objects are outside of the print volume.") };

    if (extruders.empty())
        return { PrintBase::PrintValidationError::pveNoPrint, _u8L("The supplied settings will cause an empty print.") };

    // Empty-print errors take precedence. This checks only the print-level
    // config, not object/region overrides, using the runtime group selector.
    const DynamicPrintConfig &full_config = full_print_config();
    const std::optional<std::string> unsupported_setting = Steps::unsupported_active_setting(
        Orchestrator::instance(), PrintConfigDef::instance(), full_config,
        [&full_config](const std::string &group_id) -> const Plugin * {
            Orchestrator &orchestrator = Orchestrator::instance();
            for (Plugin *plugin : orchestrator.registered_plugins())
                if (plugin != nullptr && orchestrator.is_plugin_active(plugin) &&
                    plugin->get_exclusive_group() == group_id)
                    return Steps::selected_active_plugin_from_group(
                        orchestrator, plugin->get_step(), group_id, &full_config);
            return nullptr;
        });
    if (unsupported_setting)
        return { PrintBase::PrintValidationError::pveWrongSettings, *unsupported_setting };

    if (m_config.complete_objects /*|| m_config.parallel_objects_step > 0*/) {
    	if (! sequential_print_horizontal_clearance_valid(*this, const_cast<Polygons*>(&m_sequential_print_clearance_contours)))
            return { PrintBase::PrintValidationError::pveWrongPosition, _u8L("Some objects are too close; your extruder will collide with them.") };
        if (m_config.complete_objects && ! sequential_print_vertical_clearance_valid(*this))
            return { PrintBase::PrintValidationError::pveWrongPosition,_u8L("Some objects are too tall and cannot be printed without extruder collisions.") };
    }
    else
        const_cast<Polygons*>(&m_sequential_print_clearance_contours)->clear();

    // The perimeter policy belongs to regions, while curled-overhang routing
    // is still a print-wide algorithm. They conflict as soon as any region
    // requests perimeter avoidance.
    const bool any_region_avoids_crossing = std::any_of(
        m_print_regions.begin(), m_print_regions.end(),
        [](const PrintRegion *region) {
            return region != nullptr && region->config().avoid_crossing_perimeters.value;
        });
    if (any_region_avoids_crossing && m_config.avoid_crossing_curled_overhangs) {
        return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Avoid crossing perimeters option and avoid crossing curled overhangs option cannot be both enabled together.") };
    }    

    if (m_config.spiral_vase) {
        size_t total_copies_count = 0;
        for (const PrintObjectUPtr &object : m_objects)
            total_copies_count += object->instances().size();
        // #4043
        if (total_copies_count > 1 && ! m_config.complete_objects.value)
            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Only a single object may be printed at a time in Spiral Vase mode. "
                     "Either remove all but the last object, or enable sequential mode by \"complete_objects\".") };
        assert(m_objects.size() == 1 || config().complete_objects.value);
        if (m_objects.front()->all_regions().size() > 1)
            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Spiral Vase option can only be used when printing single material objects, and without any modifiers that may break the vase.") };
    }

    //if (m_config.machine_limits_usage == MachineLimitsUsage::EmitToGCode && m_config.gcode_flavor == gcfKlipper)
    //    return _u8L("Machine limits cannot be emitted to G-Code when Klipper firmware flavor is used. "
    //             "Change the value of machine_limits_usage.");

    // Cache of layer height profiles for checking:
    // 1) Whether all layers are synchronized if printing with wipe tower and / or unsynchronized supports.
    // 2) Whether build volume Z is not violated.
    std::vector<std::vector<coordf_t>> layer_height_profiles;
    auto layer_height_profile = [this, &layer_height_profiles](const size_t print_object_idx) -> const std::vector<coordf_t>& {
        const PrintObject       &print_object = *m_objects[print_object_idx];
        if (layer_height_profiles.empty())
            layer_height_profiles.assign(m_objects.size(), std::vector<coordf_t>());
        std::vector<coordf_t>   &profile      = layer_height_profiles[print_object_idx];
        if (profile.empty())
            PrintObject::update_layer_height_profile(*print_object.model_object(), print_object.slicing_parameters(), profile);
        return profile;
    };

    // Checks that the print does not exceed the max print height
    for (size_t print_object_idx = 0; print_object_idx < m_objects.size(); ++ print_object_idx) {
        const PrintObject &print_object = *m_objects[print_object_idx];
        //FIXME It is quite expensive to generate object layers just to get the print height!
        if (auto layers = generate_object_layers(print_object.slicing_parameters(), layer_height_profile(print_object_idx));
            ! layers.empty() && layers.back() > this->config().max_print_height + EPSILON) {
            return { PrintBase::PrintValidationError::pveWrongPosition, 
                // Test whether the last slicing plane is below or above the print volume.
                0.5 * (layers[layers.size() - 2] + layers.back()) > this->config().max_print_height + EPSILON ?
                format(_u8L("The object %1% exceeds the maximum build volume height."), print_object.model_object()->name) :
                format(_u8L("While the object %1% itself fits the build volume, its last layer exceeds the maximum build volume height."), print_object.model_object()->name) +
                " " + _u8L("You might want to reduce the size of your model or change current print settings and retry.") };
        }
    }

    // Some of the objects has variable layer height applied by painting or by a table.
    bool has_custom_layering = std::find_if(m_objects.begin(), m_objects.end(), 
        [](const PrintObjectUPtr &object) { return object->model_object()->has_custom_layering(); }) 
        != m_objects.end();

    if (this->has_wipe_tower() && ! m_objects.empty()) {
        // Make sure all extruders use same diameter filament and have the same nozzle diameter
        // EPSILON comparison is used for nozzles and 10 % tolerance is used for filaments
        double first_nozzle_diam = m_config.nozzle_diameter.get_at(*extruders.begin());
        double first_filament_diam = m_config.filament_diameter.get_at(*extruders.begin());
        //for (const extruder_index_t extruder_idx : extruders) {
        //    double nozzle_diam = m_config.nozzle_diameter.get_at(extruder_idx);
        //    double filament_diam = m_config.filament_diameter.get_at(extruder_idx);
        //    if (nozzle_diam - EPSILON > first_nozzle_diam || nozzle_diam + EPSILON < first_nozzle_diam
        //     || std::abs((filament_diam-first_filament_diam)/first_filament_diam) > 0.1)
        //        return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The wipe tower is only supported if all extruders have the same nozzle diameter "
        //                 "and use filaments of the same diameter.") };
        //}

        //if (m_config.gcode_flavor != gcfRepRap 
        //    && m_config.gcode_flavor != gcfSprinter
        //    && m_config.gcode_flavor != gcfRepetier 
        //    && m_config.gcode_flavor != gcfMarlinLegacy
        //    && m_config.gcode_flavor != gcfMarlinFirmware
        //    && m_config.gcode_flavor != gcfKlipper )
        //    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is currently only supported for the Marlin, Klipper, RepRap/Sprinter, Repetier and NematX G-code flavors.") };
        //if (! m_config.use_relative_e_distances)
            //return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is currently only supported with the relative extruder addressing (use_relative_e_distances=1).") };
        //if (m_config.ooze_prevention && m_config.single_extruder_multi_material)
        //    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Ooze prevention is currently not supported with the wipe tower enabled.") };
        //if (m_config.use_volumetric_e)
        //    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower currently does not support volumetric E (use_volumetric_e=0).") };
        //if (m_config.complete_objects && extruders.size() > 1)
        //    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is currently not supported for multimaterial sequential prints.") };

        //if (m_objects.size() > 1) {
        //    const SlicingParameters     &slicing_params0       = m_objects.front()->slicing_parameters();
        //    size_t                       tallest_object_idx    = 0;
        //    for (size_t i = 1; i < m_objects.size(); ++ i) {
        //        const PrintObject       *object         = m_objects[i];
        //        const SlicingParameters &slicing_params = object->slicing_parameters();
        //        if (std::abs(slicing_params.first_print_layer_height - slicing_params0.first_print_layer_height) > EPSILON ||
        //            std::abs(slicing_params.layer_height             - slicing_params0.layer_height            ) > EPSILON)
        //            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is only supported for multiple objects if they have equal layer heights") };
        //        if (slicing_params.raft_layers() != slicing_params0.raft_layers())
        //            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is only supported for multiple objects if they are printed over an equal number of raft layers") };
        //        if (object->config().support_material_contact_distance_type != m_objects.front()->config().support_material_contact_distance_type
        //            || object->config().support_material_contact_distance.value != m_objects.front()->config().support_material_contact_distance.value
        //            || object->config().support_material_bottom_contact_distance.value != m_objects.front()->config().support_material_bottom_contact_distance.value
        //            || slicing_params0.gap_object_support != slicing_params.gap_object_support
        //            || slicing_params0.gap_support_object != slicing_params.gap_support_object)
        //            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is only supported for multiple objects if they are printed with the same support_material_contact_distance") };
        //        if (! equal_layering(slicing_params, slicing_params0))
        //            return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe Tower is only supported for multiple objects if they are sliced equally.") };
        //        if (has_custom_layering) {
        //            auto &lh         = layer_height_profile(i);
        //            auto &lh_tallest = layer_height_profile(tallest_object_idx);
        //            if (*(lh.end()-2) > *(lh_tallest.end()-2))
        //                tallest_object_idx = i;
        //        }
        //    }

        //    if (has_custom_layering) {
        //        for (size_t idx_object = 0; idx_object < m_objects.size(); ++ idx_object) {
        //            if (idx_object == tallest_object_idx)
        //                continue;
        //            // Check that the layer height profiles are equal. This will happen when one object is
        //            // a copy of another, or when a layer height modifier is used the same way on both objects.
        //            // The latter case might create a floating point inaccuracy mismatch, so compare
        //            // element-wise using an epsilon check.
        //            size_t i = 0;
        //            const coordf_t eps = 0.5 * EPSILON; // layers closer than EPSILON will be merged later. Let's make
        //            // this check a bit more sensitive to make sure we never consider two different layers as one.
        //            while (i < layer_height_profiles[idx_object].size()
        //                && i < layer_height_profiles[tallest_object_idx].size()) {
        //                if (i%2 == 0 && layer_height_profiles[tallest_object_idx][i] > layer_height_profiles[idx_object][layer_height_profiles[idx_object].size() - 2 ])
        //                    break;
        //                if (std::abs(layer_height_profiles[idx_object][i] - layer_height_profiles[tallest_object_idx][i]) > eps)
        //                    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("The Wipe tower is only supported if all objects have the same variable layer height") };
        //                ++i;
        //            }
        //        }
        //    }
        //}
    }
    
	{
		// Find the smallest used nozzle diameter and the number of unique nozzle diameters.
		double min_nozzle_diameter = std::numeric_limits<double>::max();
		double max_nozzle_diameter = 0;
		for (extruder_index_t extruder_id : extruders) {
			double dmr = m_config.nozzle_diameter.get_at(extruder_id);
			min_nozzle_diameter = std::min(min_nozzle_diameter, dmr);
			max_nozzle_diameter = std::max(max_nozzle_diameter, dmr);
		}

#if 0
        // We currently allow one to assign extruders with a higher index than the number
        // of physical extruders the machine is equipped with, as the Printer::apply() clamps them.
        unsigned int total_extruders_count = m_config.nozzle_diameter.size();
        for (const auto& extruder_idx : extruders)
            if ( extruder_idx >= total_extruders_count )
                return _u8L("One or more object were assigned an extruder that the printer does not have.");
#endif

        const coord_t print_first_layer_height = get_min_first_layer_height();
        for (const PrintObjectUPtr &object : m_objects) {
            if (object->has_support_material()) {
                if ((object->config().support_material_extruder == 0 || object->config().support_material_interface_extruder == 0) && max_nozzle_diameter - min_nozzle_diameter > EPSILON) {
                    // The object has some form of support and either support_material_extruder or support_material_interface_extruder
                    // will be printed with the current tool without a forced tool change. Play safe, assert that all object nozzles
                    // are of the same diameter.
                    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Printing with multiple extruders of differing nozzle diameters. "
                           "If support is to be printed with the current extruder (support_material_extruder == 0 or support_material_interface_extruder == 0), "
                           "all nozzles have to be of the same diameter.") };
                }
                // The tree and organic styles run the same plugin family and clamp the tip diameter to the branch diameter,
                // so both styles need branches that the support extrusion width can print. The tip diameter is checked against
                // the extrusion width for the tree style only. Organic branch sections thinner than the extrusion width print
                // as a stationary deposit when compact and along their medial axis when elongated (parts under the medial axis
                // width thresholds still print nothing). Tree sections are sparse-filled Base surfaces, which drop areas
                // narrower than half a spacing, so the tree tip must be at least the extrusion width.
                if (object->config().support_material_style.value == smsTree || object->config().support_material_style.value == smsOrganic) {
                    float extrusion_width = std::min(
                        support_material_flow(object.get()).width(),
                        support_material_interface_flow(object.get()).width());
                    if (object->config().support_material_style.value == smsTree &&
                        object->config().support_tree_tip_diameter < extrusion_width - EPSILON)
                        return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Organic support tree tip diameter must not be smaller than support material extrusion width.") };
                    if (object->config().support_tree_branch_diameter < 2. * extrusion_width - EPSILON)
                        return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Organic support branch diameter must not be smaller than 2x support material extrusion width.") };
                    if (object->config().support_tree_branch_diameter < object->config().support_tree_tip_diameter)
                        return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Organic support branch diameter must not be smaller than support tree tip diameter.") };
                }
            }

            // Do we have custom support data that would not be used?
            // Notify the user in that case.
            if (! object->has_support() && warnings) {
                for (const ModelVolume* mv : object->model_object()->volumes) {
                    bool has_enforcers = mv->is_support_enforcer() ||
                        (mv->is_model_part() && mv->supported_facets.has_facets(*mv, EnforcerBlockerType::ENFORCER));
                    if (has_enforcers) {
                        warnings->emplace_back("_SUPPORTS_OFF");
                        break;
                    }
                }
            }
            
            // validate layer_height for each region
            for (const PrintRegion& region : object->all_regions()) {
                std::set<extruder_index_t> object_extruders;
                PrintRegion::collect_object_printing_extruders(config(), object->config(), region.config(), object_extruders);
                const coord_t object_first_layer_height = get_object_first_layer_height(*object);
                const coord_t layer_height = scale_to_layer_coord(object->config().layer_height.value);
                for (extruder_index_t extruder_id : object_extruders) {
                    double nozzle_diameter = config().nozzle_diameter.get_at(extruder_id);
                    const coord_t min_layer_height = scale_to_layer_coord(config().min_layer_height.get_effective_value(nozzle_diameter, extruder_id));
                    coord_t max_layer_height = scale_to_layer_coord(config().max_layer_height.get_effective_value(nozzle_diameter, extruder_id));
                    if (max_layer_height <= 0 || !config().max_layer_height.is_enabled()) {
                        max_layer_height = scale_to_layer_coord(nozzle_diameter * 0.75);
                    }
                    if (min_layer_height > max_layer_height) {
                        return {PrintBase::PrintValidationError::pveWrongSettings,
                                _u8L("Min layer height can't be greater than Max layer height")};
                    }
                    //if (max_layer_height > nozzle_diameter) return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Max layer height can't be greater than nozzle diameter") };
                    double skirt_width = Flow::new_from_config_width(frPerimeter,
                        *Flow::extrusion_width_option("skirt", m_default_region_config),
                        *Flow::extrusion_spacing_option("skirt", m_default_region_config),
                        (float)m_config.nozzle_diameter.get_at(extruder_id), 
                        unscaled(print_first_layer_height),
                        1,0 //don't care, all i want if width from width
                    ).width();
                    //check first layer layer_ranges
                    
                    if (object->shared_regions()->layer_ranges.front().layer_height_range_.first < object_first_layer_height) {
                        if (object_first_layer_height < min_layer_height)
                            return { PrintBase::PrintValidationError::pveWrongSettings,
                                format(_u8L("First layer height can't be lower than %s"), "min layer height") };
                        for (auto tuple : std::vector<std::pair<double, const char*>>{
                                {nozzle_diameter, "nozzle diameter"},
                                {unscaled(max_layer_height), "max layer height"},
                                {skirt_width, "skirt extrusion width"},
                                {object->config().support_material ?
                                    region.width(FlowRole::frSupportMaterial, true, *object) :
                                    unscaled(object_first_layer_height), "support material extrusion width"},
                                {region.width(FlowRole::frPerimeter, true, *object), "perimeter extrusion width"},
                                {region.width(FlowRole::frExternalPerimeter, true, *object), "perimeter extrusion width"},
                                {region.width(FlowRole::frInfill, true, *object), "infill extrusion width"},
                                {region.width(FlowRole::frSolidInfill, true, *object), "solid infill extrusion width"},
                                {region.width(FlowRole::frTopSolidInfill, true, *object), "top solid infill extrusion width"},
                            })
                            if (object_first_layer_height > scale_to_layer_coord(tuple.first))
                                return { PrintBase::PrintValidationError::pveWrongSettings,
                                    format(_u8L("First layer height can't be greater than %s"), tuple.second) };

                    }
                    //check not-first layer
                    if (object->shared_regions()->layer_ranges.front().layer_height_range_.second > layer_height) {
                        if (layer_height < min_layer_height)
                            return { PrintBase::PrintValidationError::pveWrongSettings,
                                format(_u8L("Layer height can't be lower than %s"), "min layer height") };
                        for (auto tuple : std::vector<std::pair<double, const char*>>{
                                {nozzle_diameter, "nozzle diameter"},
                                {unscaled(max_layer_height), "max layer height"},
                                {skirt_width, "skirt extrusion width"},
                                {object->config().support_material ?
                                    region.width(FlowRole::frSupportMaterial, false, *object) :
                                    unscaled(layer_height), "support material extrusion width"},
                                {region.width(FlowRole::frPerimeter, false, *object), "perimeter extrusion width"},
                                {region.width(FlowRole::frExternalPerimeter, false, *object), "perimeter extrusion width"},
                                {region.width(FlowRole::frInfill, false, *object), "infill extrusion width"},
                                {region.width(FlowRole::frSolidInfill, false, *object), "solid infill extrusion width"},
                                {region.width(FlowRole::frTopSolidInfill, false, *object), "top solid infill extrusion width"},
                            })
                            if (layer_height > scale_to_layer_coord(tuple.first)) {
                                return {PrintBase::PrintValidationError::pveWrongSettings,
                                        format(_u8L("Layer height can't be greater than %s"), tuple.second)};
                            }
                    }
                }
            }

        }
    }
    {
        bool before_layer_gcode_resets_extruder = boost::regex_search(m_config.before_layer_gcode.value, regex_g92e0);
        bool layer_gcode_resets_extruder        = boost::regex_search(m_config.layer_gcode.value, regex_g92e0);
        if (m_config.use_relative_e_distances) {
            // See GH issues #6336 #5073$
            //merill: should add it in gcode.cpp then!
            //if (! before_layer_gcode_resets_extruder && ! layer_gcode_resets_extruder)
            //    return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("Relative extruder addressing requires resetting the extruder position at each layer to prevent loss of floating point accuracy. Add \"G92 E0\" to layer_gcode.") };
        } else {
            if (before_layer_gcode_resets_extruder)
                return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("\"G92 E0\" was found in before_layer_gcode, which is incompatible with absolute extruder addressing.") };
            else if (layer_gcode_resets_extruder)
                return { PrintBase::PrintValidationError::pveWrongSettings, _u8L("\"G92 E0\" was found in layer_gcode, which is incompatible with absolute extruder addressing.") };
        }
    }

    return { PrintValidationError::pveNone, std::string() };
}

#if 0
// the bounding box of objects placed in copies position
// (without taking skirt/brim/support material into account)
BoundingBox Print::bounding_box() const
{
    BoundingBox bb;
    for (const PrintObject *object : m_objects)
        for (const PrintInstance &instance : object->instances()) {
            BoundingBox bb2(object->bounding_box());
            bb.merge(bb2.min + instance.shift);
            bb.merge(bb2.max + instance.shift);
        }
    return bb;
}

// the total bounding box of extrusions, including skirt/brim/support material
// this methods needs to be called even when no steps were processed, so it should
// only use configuration values
BoundingBox Print::total_bounding_box() const
{
    // get objects bounding box
    BoundingBox bb = this->bounding_box();
    
    // we need to offset the objects bounding box by at least half the perimeters extrusion width
    Flow perimeter_flow = m_objects.front()->layer(0).region(0).flow(frPerimeter);
    double extra = perimeter_flow.width/2;
    
    // consider support material
    if (this->has_support_material()) {
        extra = std::max(extra, SUPPORT_MATERIAL_MARGIN);
    }
    
    // consider brim and skirt
    if (m_config.brim_width.value > 0) {
        Flow brim_flow = this->brim_flow();
        extra = std::max(extra, m_config.brim_width.value + brim_flow.width/2);
    }
    if (this->has_skirt()) {
        int skirts = m_config.skirts.value + m_config.skirt_brim.value;
        if (skirts == 0 && this->has_infinite_skirt()) skirts = 1;
        double max_skirt_width = 0;
        double max_skirt_spacing = 0;
        for (extruder_index_t extruder_id : this->object_extruders()) {
            Flow   flow = this->skirt_flow(extruder_id);
            max_skirt_width = std::max(max_skirt_width, flow.width());
            max_skirt_spacing = std::max(max_skirt_spacing, flow.spacing());
        }
        if (m_config.skirt_distance_from_brim)
            extra += m_config.brim_width.value
                + m_config.skirt_distance.value
                + skirts * max_skirt_spacing
                + max_skirt_width / 2;
        else
            extra += std::max(
                m_config.brim_width.value,
                m_config.skirt_distance.value
                    + skirts * max_skirt_spacing
                    + max_skirt_width / 2);
    }
    
    if (extra > 0)
        bb.offset(scale_(extra));
    
    return bb;
}
#endif

Flow Print::brim_flow(extruder_index_t extruder_id, const PrintObjectConfig& brim_config) const
{
    if (extruder_id == EXTRUDER_ID_UNKNOWN || extruder_id < EXTRUDER_ID_UNKNOWN)
        throw Slic3r::InvalidArgument("brim_flow requires a concrete extruder index");
    //use default region, but current object config.
    PrintRegionConfig tempConf = m_default_region_config;
    tempConf.parent = &brim_config;
    return Flow::new_from_config_width(
        frPerimeter,
        *Flow::extrusion_width_option("brim", tempConf),
        *Flow::extrusion_spacing_option("brim", tempConf),
        (float)m_config.nozzle_diameter.get_at(extruder_id),
        (float)unscaled(get_min_first_layer_height()),
        (extruder_id < m_config.nozzle_diameter.size()) ? brim_config.get_computed_value("filament_max_overlap", extruder_id) : 1
    );
}

Flow Print::skirt_flow(extruder_index_t extruder_id, bool first_layer/*=false*/) const
{
    if (extruder_id == EXTRUDER_ID_UNKNOWN || extruder_id < EXTRUDER_ID_UNKNOWN)
        throw Slic3r::InvalidArgument("skirt_flow requires a concrete extruder index");
    if (m_objects.empty())
        throw Slic3r::InvalidArgument("skirt_first_layer_height() can't be called without PrintObjects");

    //get extruder used to compute first layer height
    double max_nozzle_diam = 0.f;
    for (const PrintObjectUPtr &pobject : m_objects) {
        PrintObject& object = *pobject;
        std::set<extruder_index_t> object_extruders;
        for (const PrintRegion& region : pobject->all_regions()) {
            PrintRegion::collect_object_printing_extruders(config(), object.config(), region.config(), object_extruders);
        }
        //get object first layer extruder diam
        for (extruder_index_t extruder_id : object_extruders) {
            double nozzle_diameter = config().nozzle_diameter.get_at(extruder_id);
            max_nozzle_diam = std::max(max_nozzle_diam, nozzle_diameter);
        }
    }

    
    //send m_default_object_config becasue it's the lowest config needed (extrusion_option need config from object & print)
    return Flow::new_from_config_width(
        frPerimeter,
        *Flow::extrusion_width_option("skirt", m_default_region_config),
        *Flow::extrusion_spacing_option("skirt", m_default_region_config),
        (float)max_nozzle_diam,
        (float)unscaled(get_min_first_layer_height()),
        1 // hard to say what extruder we have here(many) m_default_region_config.get_computed_value("filament_max_overlap", extruder -1),
    );
    
}

bool Print::has_support_material() const
{
    for (const PrintObjectUPtr &object : m_objects)
        if (object->has_support_material()) 
            return true;
    return false;
}

/*  This method assigns extruders to the volumes having a material
    but not having extruders set in the volume config. */
void Print::auto_assign_extruders(ModelObject* model_object) const
{
    // only assign extruders if object has more than one volume
    if (model_object->volumes.size() < 2)
        return;
    
//    size_t extruders = m_config.nozzle_diameter.size();
    for (size_t volume_id = 0; volume_id < model_object->volumes.size(); ++ volume_id) {
        ModelVolume *volume = model_object->volumes[volume_id];
        //FIXME Vojtech: This assigns an extruder ID even to a modifier volume, if it has a material assigned.
        if ((volume->is_model_part() || volume->is_modifier()) && ! volume->material_id().empty() && ! volume->config.has("extruder"))
            volume->config.set("extruder", int(volume_id + 1));
    }
}

#ifdef _DEBUG
class CheckOrientation : public ExtrusionVisitorRecursiveConst
{
public:
    bool ccw;
    CheckOrientation(bool is_ccw) : ExtrusionVisitorRecursiveConst() {ccw = (is_ccw);}
    void default_use(const ExtrusionEntity &entity) override {
        if (!entity.is_leaf()) {
            ExtrusionVisitorRecursiveConst::default_use(entity);
            if (entity.is_loop()) {
                Polygon polygon(entity.as_polyline().to_polyline().points);
                assert(polygon.is_counter_clockwise() == ccw);
            }
        }
    }
};
#endif

// Slice this owned print through the orchestrator and cache tool ordering for preview/export, updating status and timestamps; invalid empty shared ordering or slicing/allocation/cancellation failures propagate.
void Print::process() {
    m_timestamp_last_change = std::time(0);
    name_tbb_thread_pool_threads_set_locale();
    BOOST_LOG_TRIVIAL(info) << "Starting the step pipeline slicing process." << log_memory_info();
    secondary_status_counter_reset();

    // Print::process() is the GUI and CLI entry point for preparing all slicing
    // data. G-code export is now a separate Orchestrator pipeline phase because
    // it needs the final output path and should only run when the user exports.
    Orchestrator::instance().slice(*this);

    // Preview ticks and the legacy exporter consume this ordering cache.
    if (this->should_execute_step(STEP_WIPETOWER)) {
        m_tool_orderings.clear();
        bool is_separate_objects = this->config().complete_objects.value || config().parallel_objects_step.value > 0;
        if (config().parallel_objects_step.value > 0 && config().parallel_islands.value && m_default_object_config.wipe_tower && m_objects.size() == 1) {
            is_separate_objects = false;
        }
        if (is_separate_objects) {
            //throw new std::exception();
            // FIXME: parallel_objects_step: end extruder on each pass isn't computed correctly.
            // TODO: add extruder-switch minimizing option.
            // Order object instances for sequential print.
            std::vector<const PrintInstance *> instances_ordering;
            if (config().complete_objects_sort.value == cosObject)
                instances_ordering = this->sort_object_instances_by_model_order();
            else if (config().complete_objects_sort.value == cosZ)
                instances_ordering = this->sort_object_instances_by_max_z();
            else if (config().complete_objects_sort.value == cosY)
                instances_ordering = this->sort_object_instances_by_max_y();
            else if (config().complete_objects_sort.value == cosNearest)
                instances_ordering = chain_print_object_instances(*this);
            // Find the 1st printing object, find its tool ordering and the initial extruder ID.
            extruder_index_t previous_final_extruder = EXTRUDER_ID_UNKNOWN;
            for (auto it = instances_ordering.begin(); it != instances_ordering.end(); ++it) {
                m_tool_orderings.emplace_back(*(*it)->print_object, previous_final_extruder);
                // last_extruder == -1 => nothign to print, so skip
                if (m_tool_orderings.back().last_extruder() != EXTRUDER_ID_UNKNOWN) {
                    previous_final_extruder = m_tool_orderings.back().last_extruder();
                }
            }
        } else {
            // Initialize the tool ordering, so it could be used by the G-code preview slider for planning tool
            // changes and filament switches.
            m_tool_orderings.emplace_back(*this, EXTRUDER_ID_UNKNOWN, /*prime_mmu*/ true /*false*/);
            if (m_tool_orderings.empty() || m_tool_orderings.back().last_extruder() == EXTRUDER_ID_UNKNOWN)
                throw Slic3r::SlicingError(
                    "The print is empty. The model is not printable with current print settings.");

            // add colorchange/pause/custom
            m_tool_orderings.back().assign_custom_gcodes(*this);

        }
    }
    
    secondary_status_counter_reset();

    m_timestamp_last_change = std::time(0);
    BOOST_LOG_TRIVIAL(info) << "Step pipeline slicing process finished." << log_memory_info();
    this->status().set(this->pipeline_progress().percent(), L("Slicing done"),
                       SlicingStatus::FlagBits::SLICING_ENDED);
}
bool has_brim_patch(const PrintObject &obj, ModelVolumeType brim_type)
{
    bool found = false;
    for (const ModelVolume *v : obj.model_object()->volumes) {
        assert(v);
        if (v->type() == brim_type) {
            found = true;
            break;
        }
    }
    return found;
}
bool has_brim_patch(const std::vector<PrintObject*> &objs_group, ModelVolumeType brim_type)
{
    bool found = false;
    for (const PrintObject *obj : objs_group) {
        if (has_brim_patch(*obj, brim_type)) {
            found = true;
            break;
        }
    }
    return found;
}

struct ExtrusionDirectionSetter : public ExtrusionVisitorRecursive {
    using ExtrusionVisitorRecursive::use;
    bool m_set_cw;
    ExtrusionDirectionSetter() : m_set_cw(true) {}
    ExtrusionDirectionSetter(bool set_cw) : m_set_cw(set_cw) {}
    virtual void default_use(ExtrusionEntity& entity) override {
        if (entity.is_loop()) {
            Polygon polygon(entity.as_polyline().to_polyline().points);
            if (polygon.is_counter_clockwise() == m_set_cw ) {
                entity.reverse();
            }
            return;
        }
        ExtrusionVisitorRecursive::default_use(entity);
    }
};

void append_extrusion_children_to_collection(ExtrusionEntityCollection &dst, ExtrusionEntity &src)
{
    if (src.is_nop())
        return;

    if (ExtrusionEntityCollection *collection = dynamic_cast<ExtrusionEntityCollection *>(&src)) {
        /*
        The plugin root is a temporary transport object. Clone its children
        into the persistent Print tree, then clear the transport root. This is
        a little more conservative than moving unique_ptrs across the API
        boundary and keeps all long-lived adhesion extrusion owned by Print.
        If the collection has properties, it is a semantic subtree and must
        stay whole so descendants inherit them.
        */
        if (collection->has_properties()) {
            dst.append(src);
            src.clear_content();
            src.clear_properties();
            return;
        }
        for (const ExtrusionEntity *child : collection->entities())
            dst.append(*child);
        collection->clear();
        return;
    }

    if (src.is_leaf()) {
        dst.append(src);
        src.clear_content();
        src.clear_properties();
        return;
    }

    ExtrusionEntity::Children &children = src.children();
    for (const ExtrusionEntityUPtr &child : children)
        dst.append(*child);
    src.clear_content();
    src.clear_properties();
}

void ApiInternal::PrintAccess::clear_brim(Print &print)
{
    PrintObject *auxiliary_object = const_cast<PrintObject *>(print.auxiliary_object());
    if (auxiliary_object != nullptr)
        remove_auxiliary_layers(*auxiliary_object, layer_is_brim_adhesion);
    for (PrintObjectUPtr &object : print.m_objects)
        ApiInternal::PrintObjectAccess::clear_brim_auxiliary_layers(*object);
}

void ApiInternal::PrintAccess::clear_skirt(Print &print)
{
    PrintObject *auxiliary_object = const_cast<PrintObject *>(print.auxiliary_object());
    if (auxiliary_object != nullptr)
        remove_auxiliary_layers(*auxiliary_object, layer_is_any_skirt_adhesion);
    for (PrintObjectUPtr &object : print.m_objects)
        ApiInternal::PrintObjectAccess::clear_skirt_auxiliary_layers(*object);
}

void ApiInternal::PrintAccess::normalize_skirt_brim_direction(Print &print)
{
    /*
    Skirt/brim output is stored in auxiliary layers. Normalize every tagged
    layer after plugins finish so each generator does not have to duplicate the
    perimeter-direction rule.
    */
    if (print.m_default_region_config.perimeter_direction.value == pdCW_CCW ||
        print.m_default_region_config.perimeter_direction.value == pdCW_CW) {
        ExtrusionDirectionSetter visitor(true);
        PrintObject *auxiliary_object = const_cast<PrintObject *>(print.auxiliary_object());
        if (auxiliary_object != nullptr)
            visit_matching_auxiliary_layer_extrusions(*auxiliary_object, layer_is_any_adhesion, visitor);
        for (PrintObjectUPtr &object : print.m_objects)
            visit_matching_auxiliary_layer_extrusions(*object, layer_is_any_adhesion, visitor);
    }
}

void ApiInternal::PrintAccess::rebuild_first_layer_convex_hull_after_skirt_brim(Print &print)
{
    /*
    The hull is a print-level safety envelope used by bed-leveling, placeholders
    and wipe/purge placement. Build it from the stable final geometry: first
    layer islands plus physical auxiliary-layer slices published by brim/skirt
    plugins. Those auxiliary slices are extrusion coverage, so the hull includes
    path width instead of only centerline points.
    */
    print.m_first_layer_convex_hull.points.clear();
    for (Polygon &polygon : print.first_layer_islands())
        append(print.m_first_layer_convex_hull.points, std::move(polygon.points));

    const PrintObject *auxiliary_object = print.auxiliary_object();
    if (auxiliary_object != nullptr)
        for (const Layer &layer : auxiliary_object->auxiliary_layers())
            if (layer_is_any_adhesion(layer))
                append_auxiliary_layer_points(layer, print.m_first_layer_convex_hull.points);
    for (PrintObjectUPtr &object : print.m_objects)
        append_matching_auxiliary_layer_points(*object, layer_is_any_adhesion, print.m_first_layer_convex_hull.points);

    print.finalize_first_layer_convex_hull();
}

Polygons Print::first_layer_islands() const
{
    Polygons islands;
    for (const PrintObjectUPtr &object : m_objects) {
        Polygons object_islands;
        for (const ExPolygon &expoly : object->m_layers.front()->lslices())
            object_islands.push_back(expoly.contour);
        for (const Layer &support_layer : object->auxiliary_layers()) {
            if (support_layer.get_property<LayerSupportProperty>() == nullptr)
                continue;
            // was polygons_covered_by_spacing, but is it really important?
            for (const LayerSliceIsland &island : support_layer.islands()) {
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (region_island.has_extrusion(LayerRegionIsland::SUPPORT)) {
                               region_island.extrusion(LayerRegionIsland::SUPPORT)
                                   .polygons_covered_by_width(object_islands, float(SCALED_EPSILON));
                    }
                    if (region_island.has_extrusion(LayerRegionIsland::SUPPORT_INTERFACE)) {
                               region_island.extrusion(LayerRegionIsland::SUPPORT_INTERFACE)
                                   .polygons_covered_by_width(object_islands, float(SCALED_EPSILON));
                    }
                }
            }
            break;
        }
        islands.reserve(islands.size() + object_islands.size() * object->instances().size());
        for (const PrintInstance &instance : object->instances())
            for (Polygon &poly : object_islands) {
                islands.push_back(poly);
                islands.back().translate(instance.shift);
            }
    }
    return islands;
}

// Copy scaled generic tower corners from borrowed settings/instances; no generated wipe data is read, ownership stays with the Print, and allocation failures propagate.
Points Print::first_layer_wipe_tower_corners() const
{
    Points points;
    for (const WipeTowerFootprint &tower : wipe_tower_footprints(*this))
        append(points, tower.polygon.points);
    return points;
}

// Append skirt coverage or first-layer islands to the owned cached hull, then replace it with their scaled convex hull; omit config-only towers and propagate allocation failures.
void Print::finalize_first_layer_convex_hull()
{
    append(m_first_layer_convex_hull.points, m_skirt_convex_hull);
    if (m_first_layer_convex_hull.empty()) {
        // Neither skirt nor brim was extruded. Collect points of printed objects from 1st layer.
        for (Polygon &poly : this->first_layer_islands())
            append(m_first_layer_convex_hull.points, std::move(poly.points));
    }
    m_first_layer_convex_hull = Geometry::convex_hull(m_first_layer_convex_hull.points);
}

// Copy the cached scaled hull and current config-only tower corners into an owned export envelope; leave slicing state and borrowed source ownership unchanged, and propagate allocation failures.
Polygon Print::first_layer_convex_hull_with_wipe_towers() const
{
    Points points = m_first_layer_convex_hull.points;
    append(points, first_layer_wipe_tower_corners());
    Polygon hull = Geometry::convex_hull(points);
    return hull;
}

// Inspect borrowed configuration and object instances without changing state; return whether any eligible tower exists, with no ownership transfer.
bool Print::has_wipe_tower() const
{
    if (has_global_wipe_tower())
        return true;
    if (config().nozzle_diameter.size() <= 1 || config().spiral_vase.value)
        return false;
    for (const PrintObject &object : objects())
        if (!object.instances().empty() && ((config().complete_objects.value && object.config().wipe_tower.value) || object.config().wipe_tower_per_object.value))
            return true;
    return false;
}

// Inspect borrowed settings outside complete-object/vase modes; return global eligibility independent of parallel scheduling without changing geometry, ownership or pipeline state.
bool Print::has_global_wipe_tower() const {
    if (config().nozzle_diameter.size() <= 1 || !m_default_object_config.wipe_tower || config().complete_objects ||
        config().spiral_vase.value) {
        return false;
    }
    return true;
}

// Generate a recommended G-code output file name based on the format template, default extension, and template parameters
// (timestamps, object placeholders derived from the model, current placeholder prameters and print statistics.
// Use the final print statistics if available, or just keep the print statistics placeholders if not available yet (before G-code is finalized).
std::string Print::output_filename(const std::string &filename_base) const 
{ 
    // Set the placeholders for the data know first after the G-code export is finished.
    // These values will be just propagated into the output file name.
    DynamicConfig config = this->finished() ? this->print_statistics().config() : this->print_statistics().placeholders();
    config.set_key_value("num_extruders", new ConfigOptionInt((int)m_config.nozzle_diameter.size()));
    config.set_key_value("extruders_count", new ConfigOptionInt((int)m_config.nozzle_diameter.size()));
    config.set_key_value("num_milling", new ConfigOptionInt((int)m_config.milling_diameter.size()));
    config.set_key_value("milling_count", new ConfigOptionInt((int)m_config.milling_diameter.size()));
    config.set_key_value("default_output_extension", new ConfigOptionString(".gcode"));

    // Handle output_filename_format. There is a hack related to binary G-codes: gcode / bgcode substitution.
    std::string output_filename_format = m_config.output_filename_format.value;
    if (m_config.binary_gcode && boost::iends_with(output_filename_format, ".gcode"))
        output_filename_format.insert(output_filename_format.end()-5, 'b');
    if (! m_config.binary_gcode && boost::iends_with(output_filename_format, ".bgcode"))
        output_filename_format.erase(output_filename_format.end()-6);

    return this->PrintBase::output_filename(output_filename_format, ".gcode", filename_base, &config);
}

// Sort the PrintObjects by their increasing Z, likely useful for avoiding colisions on Deltas during sequential prints.
std::vector<const PrintInstance*> Print::sort_object_instances_by_max_z() const
{
    std::vector<const PrintObject*> objects;
    objects.reserve(this->objects().size());
    for (const PrintObject &object : this->objects())
        objects.emplace_back(&object);
    std::sort(objects.begin(), objects.end(), [](const PrintObject* po1, const PrintObject* po2) { return po1->height() < po2->height(); });
    std::vector<const PrintInstance*> instances;
    instances.reserve(objects.size());
    for (const PrintObject* object : objects)
        for (size_t i = 0; i < object->instances().size(); ++i)
            instances.emplace_back(&object->instances()[i]);
    return instances;
}

// Sort the PrintObjects by their increasing Y, likely useful for avoiding colisions on printer with a x-bar during sequential prints.
std::vector<const PrintInstance*> Print::sort_object_instances_by_max_y() const
{
    std::vector<const PrintObject*> objects;
    objects.reserve(this->objects().size());
    for (const PrintObject &object : this->objects())
        objects.emplace_back(&object);
    std::sort(objects.begin(), objects.end(), [](const PrintObject* po1, const PrintObject* po2) { return po1->height() < po2->height(); });
    std::vector<const PrintInstance*> instances;
    instances.reserve(objects.size());
    std::map<const PrintInstance*, coord_t> map_min_y;
    for (const PrintObject* object : objects) {
        for (size_t i = 0; i < object->instances().size(); ++i) {
            instances.emplace_back(&object->instances()[i]);
            // Calculate the convex hull of a printable object. 
            Polygon poly = object->model_object()->convex_hull_2d(
                object->trafo()
                // already in object->trafo()
                //* Geometry::assemble_transform(Vec3d::Zero(),
                //    object->instances()[i].model_instance->get_rotation(), 
                //    object->instances()[i].model_instance->get_scaling_factor(), 
                //    object->instances()[i].model_instance->get_mirror())
            );
            BoundingBox bb(poly.points);
            Vec2crd offset = object->instances()[i].shift - object->center_offset();
            bb.translate(offset.x(), offset.y());
            map_min_y[instances.back()] = bb.min.y();
        }
    }
    std::sort(instances.begin(), instances.end(), [&map_min_y](const PrintInstance* po1, const PrintInstance* po2) { return map_min_y[po1] < map_min_y[po2]; });
    return instances;
}

// Produce a vector of PrintObjects in the order of their respective ModelObjects in this->model().
std::vector<const PrintInstance*> Print::sort_object_instances_by_model_order() const
{
    // Build up map from ModelInstance* to PrintInstance*
    std::vector<std::pair<const ModelInstance*, const PrintInstance*>> model_instance_to_print_instance;
    model_instance_to_print_instance.reserve(this->num_object_instances());
    for (const PrintObject &print_object : this->objects())
        for (const PrintInstance &print_instance : print_object.instances())
            model_instance_to_print_instance.emplace_back(print_instance.model_instance, &print_instance);
    std::sort(model_instance_to_print_instance.begin(), model_instance_to_print_instance.end(), [](auto &l, auto &r) { return l.first < r.first; });

    std::vector<const PrintInstance*> instances;
    instances.reserve(model_instance_to_print_instance.size());
    for (const ModelObject &model_object : this->model().objects())
        for (const ModelInstance *model_instance : model_object.instances) {
            auto it = std::lower_bound(model_instance_to_print_instance.begin(), model_instance_to_print_instance.end(), std::make_pair(model_instance, nullptr), [](auto &l, auto &r) { return l.first < r.first; });
            if (it != model_instance_to_print_instance.end() && it->first == model_instance)
                instances.emplace_back(it->second);
        }
    return instances;
}

const std::string PrintStatistics::FilamentUsedG     = "filament used [g]";
const std::string PrintStatistics::FilamentUsedGMask = "; filament used [g] =";

const std::string PrintStatistics::TotalFilamentUsedG          = "total filament used [g]";
const std::string PrintStatistics::TotalFilamentUsedGMask      = "; total filament used [g] =";
const std::string PrintStatistics::TotalFilamentUsedGValueMask = "; total filament used [g] = %.2lf\n";

const std::string PrintStatistics::FilamentUsedCm3     = "filament used [cm3]";
const std::string PrintStatistics::FilamentUsedCm3Mask = "; filament used [cm3] =";

const std::string PrintStatistics::FilamentUsedMm     = "filament used [mm]";
const std::string PrintStatistics::FilamentUsedMmMask = "; filament used [mm] =";

const std::string PrintStatistics::FilamentCost     = "filament cost";
const std::string PrintStatistics::FilamentCostMask = "; filament cost =";

const std::string PrintStatistics::TotalFilamentCost          = "total filament cost";
const std::string PrintStatistics::TotalFilamentCostMask      = "; total filament cost =";
const std::string PrintStatistics::TotalFilamentCostValueMask = "; total filament cost = %.2lf\n";

const std::string PrintStatistics::TotalFilamentUsedWipeTower     = "total filament used for wipe tower [g]";
const std::string PrintStatistics::TotalFilamentUsedWipeTowerValueMask = "; total filament used for wipe tower [g] = %.2lf\n";



// Copy measured totals, tower shares and times into an owned placeholder config;
// option allocation failures propagate and the source statistics remain unchanged.
DynamicConfig PrintStatistics::config() const
{
    DynamicConfig config;
    if (this->estimated_print_time_str.find(static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Normal)) !=
        this->estimated_print_time_str.end()) {
        std::string normal_print_time = short_time(
            this->estimated_print_time_str.at(static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Normal)));
        config.set_key_value("print_time", new ConfigOptionString(normal_print_time));
        config.set_key_value("normal_print_time", new ConfigOptionString(normal_print_time));
    } else if (this->estimated_print_time_str.find(static_cast<uint8_t>(
                   PrintEstimatedStatistics::ETimeMode::Stealth)) != this->estimated_print_time_str.end()) {
        std::string silent_print_time = short_time(
            this->estimated_print_time_str.at(static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Stealth)));
        config.set_key_value("print_time", new ConfigOptionString(silent_print_time));
    }
    if (this->estimated_print_time_str.find(static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Stealth)) !=
        this->estimated_print_time_str.end()) {
        std::string silent_print_time = short_time(
            this->estimated_print_time_str.at(static_cast<uint8_t>(PrintEstimatedStatistics::ETimeMode::Stealth)));
        config.set_key_value("silent_print_time", new ConfigOptionString(silent_print_time));
    }
    config.set_key_value("used_filament",             new ConfigOptionFloat(this->total_used_filament / 1000.));
    config.set_key_value("extruded_volume",           new ConfigOptionFloat(this->total_extruded_volume));
    config.set_key_value("total_cost",                new ConfigOptionFloat(this->total_cost));
    config.set_key_value("total_toolchanges",         new ConfigOptionInt(this->total_toolchanges));
    config.set_key_value("total_weight",              new ConfigOptionFloat(this->total_weight));
    config.set_key_value("total_wipe_tower_cost",     new ConfigOptionFloat(this->total_wipe_tower_cost));
    config.set_key_value("total_wipe_tower_filament", new ConfigOptionFloat(this->total_wipe_tower_filament));
    config.set_key_value("initial_tool",              new ConfigOptionInt(int(this->initial_extruder_id)));
    config.set_key_value("initial_extruder",          new ConfigOptionInt(int(this->initial_extruder_id)));
    config.set_key_value("initial_filament_type",     new ConfigOptionString(this->initial_filament_type));
    config.set_key_value("printing_filament_types",   new ConfigOptionString(this->printing_filament_types));
    config.set_key_value("num_printing_extruders",    new ConfigOptionInt(int(this->printing_extruders.size())));
//    config.set_key_value("printing_extruders",        new ConfigOptionInts(std::vector<int>(this->printing_extruders.begin(), this->printing_extruders.end())));
    
    return config;
}

DynamicConfig PrintStatistics::placeholders()
{
    DynamicConfig config;
    for (const char *key : { 
        "print_time", "normal_print_time", "silent_print_time", 
        "used_filament", "extruded_volume", "total_cost", "total_weight", 
        "total_toolchanges", "total_wipe_tower_cost", "total_wipe_tower_filament",
        "initial_tool", "initial_extruder", "initial_filament_type", "printing_filament_types", "num_printing_extruders" })
        config.set_key_value(key, new ConfigOptionString(std::string("{") + key + "}"));
    return config;
}

std::string PrintStatistics::finalize_output_path(const std::string &path_in) const
{
    std::string final_path;
    try {
        boost::filesystem::path path(path_in);
        DynamicConfig cfg = this->config();
        PlaceholderParser pp;
        std::string new_stem = pp.process(path.stem().string(), 0, &cfg);
        final_path = (path.parent_path() / (new_stem + path.extension().string())).string();
    } catch (const std::exception &ex) {
        BOOST_LOG_TRIVIAL(error) << "Failed to apply the print statistics to the export file name: " << ex.what();
        final_path = path_in;
    }
    return final_path;
}


} // namespace Slic3r
