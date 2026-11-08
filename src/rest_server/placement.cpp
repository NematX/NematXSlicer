/*
 * Headless arrangement and orientation for a private scene candidate.
 *
 * Arrangement builds a normal libslic3r scene with a private applied Print,
 * then applies only the engine's placement transforms. Orientation likewise
 * works on per-instance meshes and publishes no state until the caller commits
 * the changed candidate.
 */
#include "placement.hpp"

#include "scene_edit.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Arrange/Items/ArrangeItem.hpp"
#include "libslic3r/Arrange/SceneBuilder.hpp"
#include "libslic3r/Arrange/Tasks/ArrangeTask.hpp"
#include "libslic3r/Arrange/Tasks/ArrangeTaskImpl.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Orient.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

// A dynamic selection is copied into FixedSelection before SceneBuilder retains it.
class InstanceSelection final : public arr2::SelectionMask {
public:
    explicit InstanceSelection(std::vector<std::vector<bool>> selected)
        : selected_(std::move(selected))
    {}

    std::vector<bool> selected_objects() const override
    {
        std::vector<bool> result;
        result.reserve(selected_.size());
        for (const std::vector<bool> &instances : selected_)
            result.push_back(std::any_of(instances.begin(), instances.end(), [](bool value) { return value; }));
        return result;
    }

    std::vector<bool> selected_instances(int object_index) const override
    {
        if (object_index < 0 || static_cast<std::size_t>(object_index) >= selected_.size())
            return {};
        return selected_[static_cast<std::size_t>(object_index)];
    }

    bool is_wipe_tower() const override { return false; }

private:
    std::vector<std::vector<bool>> selected_;
};

// Determine whether sequential printing adds a physical nozzle-clearance constraint.
bool sequential_printing(const DynamicPrintConfig &config)
{
    const ConfigOptionBool *complete_objects = config.option<ConfigOptionBool>("complete_objects");
    const ConfigOption *parallel_step = config.option("parallel_objects_step");
    return (complete_objects != nullptr && complete_objects->value) ||
        (parallel_step != nullptr && parallel_step->get_float() > 0.);
}

// Compare every transform coefficient exactly, as placement no-op detection requires.
bool same_transform(const Transform3d &left, const Transform3d &right)
{
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            if (left.matrix()(row, column) != right.matrix()(row, column))
                return false;
    return true;
}

// Find an instance's owner-relative index, which is required by per-instance geometry queries.
std::size_t instance_index(ModelObject &owner, ModelInstance &instance)
{
    const std::vector<ModelInstance *>::iterator found =
        std::find(owner.instances.begin(), owner.instances.end(), &instance);
    if (found == owner.instances.end())
        throw std::logic_error("An oriented instance is missing from its owning object.");
    return static_cast<std::size_t>(std::distance(owner.instances.begin(), found));
}

// Return instances in scene order, applying the operation's empty-target semantics.
std::vector<ModelInstance *> target_instances(
    Model &scene,
    const InstanceTargets &targets,
    bool printable_when_all)
{
    const bool all_instances = targets.empty();
    const std::set<std::uint64_t> requested(targets.begin(), targets.end());
    std::vector<ModelInstance *> result;
    for (ModelObject *object : scene.object_ptrs())
        for (ModelInstance *instance : object->instances) {
            const std::uint64_t id = static_cast<std::uint64_t>(instance->id().id);
            if ((all_instances && (!printable_when_all || instance->is_printable())) ||
                (!all_instances && requested.find(id) != requested.end()))
                result.push_back(instance);
        }
    return result;
}

// Copy all corresponding area-optimization defaults without relying on object layout.
orientation::OrientParams area_orientation_params(const orientation::OrientParamsArea &area)
{
    orientation::OrientParams result;
    result.TAR_A = area.TAR_A;
    result.TAR_B = area.TAR_B;
    result.RELATIVE_F = area.RELATIVE_F;
    result.CONTOUR_F = area.CONTOUR_F;
    result.BOTTOM_F = area.BOTTOM_F;
    result.BOTTOM_HULL_F = area.BOTTOM_HULL_F;
    result.TAR_C = area.TAR_C;
    result.TAR_D = area.TAR_D;
    result.TAR_E = area.TAR_E;
    result.FIRST_LAY_H = area.FIRST_LAY_H;
    result.VECTOR_TOL = area.VECTOR_TOL;
    result.NEGL_FACE_SIZE = area.NEGL_FACE_SIZE;
    result.ASCENT = area.ASCENT;
    result.PLAFOND_ADV = area.PLAFOND_ADV;
    result.CONTOUR_AMOUNT = area.CONTOUR_AMOUNT;
    result.OV_H = area.OV_H;
    result.height_offset = area.height_offset;
    result.height_log = area.height_log;
    result.height_log_k = area.height_log_k;
    result.LAF_MAX = area.LAF_MAX;
    result.LAF_MIN = area.LAF_MIN;
    result.TAR_LAF = area.TAR_LAF;
    result.TAR_PROJ_AREA = area.TAR_PROJ_AREA;
    result.BOTTOM_MIN = area.BOTTOM_MIN;
    result.BOTTOM_MAX = area.BOTTOM_MAX;
    result.height_to_bottom_hull_ratio_MIN = area.height_to_bottom_hull_ratio_MIN;
    result.BOTTOM_HULL_MAX = area.BOTTOM_HULL_MAX;
    result.APPERANCE_FACE_SUPP = area.APPERANCE_FACE_SUPP;
    result.overhang_angle = area.overhang_angle;
    result.use_low_angle_face = area.use_low_angle_face;
    result.min_volume = area.min_volume;
    result.fun_dir = area.fun_dir;
    result.parallel = area.parallel;
    result.progressind = area.progressind;
    result.stopcondition = area.stopcondition;
    return result;
}

// Reject parameters that cannot be represented by the arrangement engine's float settings.
void validate_arrange_request(
    const ArrangeRequest &request,
    double distance,
    double minimum_distance)
{
    if (!std::isfinite(distance) || distance < minimum_distance ||
        distance > maximum_arrange_distance_mm())
        throw SceneEditError(422, "invalid_arrange", "distance must meet the minimum clearance and fit the engine coordinate range.");
    if (!std::isfinite(request.distance_from_bed) || request.distance_from_bed < 0. ||
        request.distance_from_bed > maximum_arrange_distance_mm())
        throw SceneEditError(422, "invalid_arrange", "distance_from_bed must be non-negative and fit the engine coordinate range.");
}

// Require a supported finite angle before constructing any orientation meshes.
void validate_orient_request(const OrientRequest &request)
{
    if (!std::isfinite(request.overhang_angle) || request.overhang_angle < 45. || request.overhang_angle > 90.)
        throw SceneEditError(422, "invalid_orient", "overhang_angle must be between 45 and 90 degrees.");
}

} // namespace

double minimum_arrange_distance(const DynamicPrintConfig &config)
{
    if (!sequential_printing(config))
        return 0.;
    return min_object_distance(&config, 0.);
}

double default_arrange_distance(const DynamicPrintConfig &config)
{
    const ConfigOptionFloat *duplicate_distance = config.option<ConfigOptionFloat>("duplicate_distance");
    if (duplicate_distance == nullptr || duplicate_distance->value == 0.)
        return 6.;
    return duplicate_distance->value + min_object_distance(&config, 0.);
}

double maximum_arrange_distance_mm() noexcept
{
    // Keep margin for the bed inset, half-distance arithmetic, and polygon offsets after scaling.
    const double scaled_limit = static_cast<double>(std::numeric_limits<coord_t>::max()) / 16.;
    const double millimeter_limit = scaled_limit / UNSCALING_FACTOR;
    const float engine_limit = static_cast<float>(millimeter_limit);
    if (static_cast<double>(engine_limit) > millimeter_limit)
        return static_cast<double>(std::nextafter(engine_limit, 0.f));
    return static_cast<double>(engine_limit);
}

ArrangeOutcome arrange_instances(
    Model &scene,
    const DynamicPrintConfig &config,
    const InstanceTargets &targets,
    const ArrangeRequest &request)
{
    ArrangeOutcome outcome;
    if (scene.objects().empty())
        return outcome;

    const double minimum_distance = minimum_arrange_distance(config);
    const double distance = request.distance.value_or(default_arrange_distance(config));
    validate_arrange_request(request, distance, minimum_distance);
    const std::vector<ModelInstance *> selected = target_instances(scene, targets, false);
    std::set<std::uint64_t> selected_ids;
    for (const ModelInstance *instance : selected) {
        const std::uint64_t id = static_cast<std::uint64_t>(instance->id().id);
        selected_ids.insert(id);
        outcome.instances.push_back({id, std::nullopt});
    }

    std::vector<std::pair<ModelInstance *, Transform3d>> before;
    before.reserve(scene.objects().size());
    for (ModelObject *object : scene.object_ptrs())
        for (ModelInstance *instance : object->instances)
            before.emplace_back(instance, instance->get_matrix());

    Model print_scene(scene);
    Print print;
    Orchestrator::RunContext run_context;
    Orchestrator &orchestrator = Orchestrator::instance();
    Orchestrator::RunScope run_scope = orchestrator.enter_run(run_context, &print);
    const ModelObjectPtrs print_objects = print_scene.object_ptrs();
    for (ModelObject *object : print_objects)
        print.auto_assign_extruders(object);
    print.apply(print_scene, DynamicPrintConfig(config));
    arr2::ArrangeSettings settings;
    settings.set_distance_from_objects(static_cast<float>(distance));
    settings.set_distance_from_bed(static_cast<float>(request.distance_from_bed));
    settings.set_rotation_enabled(request.rotations);
    settings.set_xl_alignment(request.alignment);
    settings.set_geometry_handling(request.geometry);
    settings.set_arrange_strategy(request.strategy);

    std::unique_ptr<arr2::FixedSelection> fixed_selection;
    if (targets.empty()) {
        fixed_selection = std::make_unique<arr2::FixedSelection>(scene);
    } else {
        std::vector<std::vector<bool>> selection_mask;
        selection_mask.reserve(scene.objects().size());
        for (const ModelObject &object : scene.objects()) {
            std::vector<bool> object_mask;
            object_mask.reserve(object.instances.size());
            for (const ModelInstance *instance : object.instances)
                object_mask.push_back(selected_ids.find(static_cast<std::uint64_t>(instance->id().id)) != selected_ids.end());
            selection_mask.push_back(std::move(object_mask));
        }
        InstanceSelection dynamic_selection(std::move(selection_mask));
        fixed_selection = std::make_unique<arr2::FixedSelection>(dynamic_selection);
    }
    // The Print supplies brim and skirt extents, which the engine subtracts from the bed.
    // Object spacing stays the requested distance, as in the GUI.
    arr2::Scene arrange_scene{arr2::SceneBuilder{}
        .set_bed(config)
        .set_arrange_settings(settings)
        .set_model(scene)
        .set_fff_print(&print)
        .set_selection(std::move(fixed_selection))};
    std::unique_ptr<arr2::ArrangeTask<arr2::ArrangeItem>> task =
        arr2::ArrangeTask<arr2::ArrangeItem>::create(arrange_scene);
    std::unique_ptr<arr2::ArrangeTaskResult> result = task->process_native(arr2::DummyCtl{});
    if (!result->apply_on(arrange_scene.model()))
        throw std::logic_error("The arrange result could not be applied to its source scene.");

    for (const arr2::TrafoOnlyArrangeItem &item : result->items) {
        const std::optional<ObjectID> id = arr2::retrieve_id(item);
        if (!id)
            continue;
        const std::uint64_t instance_id = static_cast<std::uint64_t>(id->id);
        if (selected_ids.find(instance_id) == selected_ids.end())
            continue;
        const std::vector<ArrangedInstance>::iterator entry = std::find_if(
            outcome.instances.begin(), outcome.instances.end(),
            [instance_id](const ArrangedInstance &instance) { return instance.instance_id == instance_id; });
        if (entry != outcome.instances.end() && arr2::is_arranged(item))
            entry->bed = arr2::get_bed_index(item);
    }

    for (const std::pair<ModelInstance *, Transform3d> &entry : before)
        if (!same_transform(entry.first->get_matrix(), entry.second)) {
            outcome.changed = true;
            break;
        }
    return outcome;
}

OrientOutcome orient_instances(
    Model &scene,
    const InstanceTargets &targets,
    const OrientRequest &request)
{
    validate_orient_request(request);
    OrientOutcome outcome;
    const std::vector<ModelInstance *> selected = target_instances(scene, targets, true);
    if (selected.empty())
        return outcome;

    std::vector<std::pair<ModelInstance *, Transform3d>> before;
    before.reserve(selected.size());
    orientation::OrientMeshs meshes;
    meshes.reserve(selected.size());
    for (ModelInstance *instance : selected) {
        ModelObject *object = instance->get_object();
        before.emplace_back(instance, instance->get_matrix());
        orientation::OrientMesh mesh;
        // Orient in world axes while isolating the target geometry from sibling instances.
        mesh.mesh = object->raw_mesh();
        instance->transform_mesh(&mesh.mesh);
        mesh.overhang_angle = request.overhang_angle;
        mesh.name = object->name;
        mesh.setter = [instance](const orientation::OrientMesh &oriented) {
            ModelObject *owner = instance->get_object();
            instance->rotate(oriented.rotation_matrix);
            owner->invalidate_bounding_box();
            // Lay only this instance on the bed; the object's lowest point may belong to a sibling.
            const std::size_t index = instance_index(*owner, *instance);
            const double min_z = owner->get_instance_min_z(index);
            if (!std::isfinite(min_z) || min_z == std::numeric_limits<double>::max())
                throw SceneEditError(422, "invalid_orient", "The instance has no valid model geometry for bed placement.");
            instance->set_offset(Z, instance->get_offset(Z) - min_z);
            owner->invalidate_bounding_box();
        };
        meshes.push_back(std::move(mesh));
        outcome.instances.push_back({static_cast<std::uint64_t>(instance->id().id), instance->get_matrix()});
    }

    orientation::OrientParams params;
    if (request.min_area) {
        const orientation::OrientParamsArea area_params;
        params = area_orientation_params(area_params);
        params.min_volume = false;
    } else {
        params.min_volume = true;
    }
    // Orient's implementation reports progress unconditionally for each mesh.
    params.progressind = [](unsigned, std::string) {};
    orientation::orient(meshes, {}, params);
    for (const orientation::OrientMesh &mesh : meshes)
        mesh.apply();

    for (std::size_t index = 0; index < selected.size(); ++index) {
        outcome.instances[index].matrix = selected[index]->get_matrix();
        if (!same_transform(selected[index]->get_matrix(), before[index].second))
            outcome.changed = true;
    }
    return outcome;
}

} // namespace Slic3r::Rest
