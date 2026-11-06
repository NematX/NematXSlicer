// Scene editing mutates a private Model copy and describes entity-ID changes for atomic publication.
#include "scene_edit.hpp"
#include "file_names.hpp"

#include "libslic3r/TriangleMesh.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace Slic3r::Rest {

namespace {

bool is_part_type_writable(ModelVolumeType type)
{
    return type >= ModelVolumeType::MODEL_PART && type <= ModelVolumeType::BRIM_NEGATIVE &&
        type != ModelVolumeType::SEAM_POSITION_INSIDE_CENTER;
}

size_t model_part_count(const ModelObject &object)
{
    return size_t(std::count_if(object.volumes.begin(), object.volumes.end(),
        [](const ModelVolume *volume) { return volume->is_model_part(); }));
}

void invalidate_cut_group(ModelObject &object)
{
    const CutObjectBase cut = object.cut_id;
    for (ModelObject *candidate : object.get_model()->object_ptrs())
        if (candidate->cut_id.is_equal(cut))
            candidate->invalidate_cut();
}

void validate_name(const std::string &name)
{
    try {
        validate_session_name(name);
    } catch (const std::invalid_argument &error) {
        throw SceneEditError(400, "invalid_request", error.what());
    }
    if (name.find_first_of("<>:/\\|?*\"") != std::string::npos)
        throw SceneEditError(422, "invalid_name", "The name contains a character forbidden in file names.");
}

} // namespace

void invalidate_cut_group(ModelObject &object)
{
    const CutObjectBase cut = object.cut_id;
    for (ModelObject *candidate : object.get_model()->object_ptrs())
        if (candidate->cut_id.is_equal(cut))
            candidate->invalidate_cut();
}

void validate_name(const std::string &name)
{
    try {
        validate_session_name(name);
    } catch (const std::invalid_argument &error) {
        throw SceneEditError(400, "invalid_request", error.what());
    }
    if (name.find_first_of("<>:/\\|?*\"") != std::string::npos)
        throw SceneEditError(422, "invalid_name", "The name contains a character forbidden in file names.");
}

bool shape_allowed(ModelVolumeType type, const std::string &shape)
{
    if (type == ModelVolumeType::BRIM_PATCH || type == ModelVolumeType::BRIM_NEGATIVE)
        return shape == "square" || shape == "circle";
    if (type == ModelVolumeType::SEAM_POSITION_CENTER)
        return shape == "small_sphere" || shape == "sphere";
    if (type == ModelVolumeType::SEAM_POSITION_CENTER_Z)
        return shape == "small_cylinder";
    if (type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME ||
        type == ModelVolumeType::PARAMETER_MODIFIER || type == ModelVolumeType::SUPPORT_BLOCKER ||
        type == ModelVolumeType::SUPPORT_ENFORCER || type == ModelVolumeType::SEAM_POSITION_INSIDE)
        return shape == "box" || shape == "cylinder" || shape == "sphere" || shape == "slab";
    return false;
}

} // namespace

SceneEditError::SceneEditError(int status, std::string code, const std::string &message)
    : std::runtime_error(message), status_(status), code_(std::move(code))
{
}

std::optional<ModelVolumeType> parse_part_type(std::string_view name)
{
    static const std::pair<std::string_view, ModelVolumeType> types[] = {
        {"part", ModelVolumeType::MODEL_PART}, {"negative_volume", ModelVolumeType::NEGATIVE_VOLUME},
        {"modifier", ModelVolumeType::PARAMETER_MODIFIER}, {"support_blocker", ModelVolumeType::SUPPORT_BLOCKER},
        {"support_enforcer", ModelVolumeType::SUPPORT_ENFORCER}, {"seam_position_center", ModelVolumeType::SEAM_POSITION_CENTER},
        {"seam_position_center_z", ModelVolumeType::SEAM_POSITION_CENTER_Z},
        {"seam_position_inside_center", ModelVolumeType::SEAM_POSITION_INSIDE_CENTER},
        {"seam_position_inside", ModelVolumeType::SEAM_POSITION_INSIDE}, {"brim_patch", ModelVolumeType::BRIM_PATCH},
        {"brim_negative", ModelVolumeType::BRIM_NEGATIVE}
    };
    for (const std::pair<std::string_view, ModelVolumeType> &entry : types)
        if (entry.first == name)
            return entry.second;
    return std::nullopt;
}

void append_object_entities(const ModelObject &object, std::vector<EntityBinding> &out)
{
    out.push_back({EntityKind::object, object.id().id});
    for (const ModelVolume *volume : object.volumes)
        out.push_back({EntityKind::volume, volume->id().id});
    for (const ModelInstance *instance : object.instances)
        out.push_back({EntityKind::instance, instance->id().id});
}

bool is_loadable_part_type(ModelVolumeType type)
{
    return type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME ||
        type == ModelVolumeType::PARAMETER_MODIFIER || type == ModelVolumeType::SUPPORT_BLOCKER ||
        type == ModelVolumeType::SUPPORT_ENFORCER || type == ModelVolumeType::BRIM_PATCH ||
        type == ModelVolumeType::BRIM_NEGATIVE;
}

SceneChange set_part_type(ModelObject &object, ModelVolume &volume, ModelVolumeType type)
{
    if (!is_part_type_writable(type))
        throw SceneEditError(422, "invalid_part_type", "This part type cannot be assigned.");
    if (type == volume.type())
        return SceneChange{false};
    if (volume.is_model_part() && model_part_count(object) == 1)
        throw SceneEditError(422, "invalid_part_type", "The last solid part cannot change type.");
    if (object.is_cut() && (type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME))
        throw SceneEditError(422, "cut_object", "A cut object cannot receive this part type.");
    if (volume.emboss_shape.has_value() && (type == ModelVolumeType::SUPPORT_BLOCKER || type == ModelVolumeType::SUPPORT_ENFORCER))
        throw SceneEditError(422, "invalid_part_type", "An embossed part cannot become a support modifier.");
    volume.set_type(type);
    object.sort_volumes(true);
    object.invalidate_bounding_box();
    return {};
}

SceneChange rename_object(ModelObject &object, const std::string &name)
{
    validate_name(name);
    if (object.name == name)
        return SceneChange{false};
    object.name = name;
    if (object.is_text())
        object.volumes.front()->name = name;
    return {};
}

SceneChange rename_volume(ModelVolume &volume, const std::string &name)
{
    validate_name(name);
    if (volume.is_text())
        throw SceneEditError(422, "invalid_name", "Embossed parts cannot be renamed.");
    if (volume.name == name)
        return SceneChange{false, false};
    volume.name = name;
    SceneChange change;
    change.affects_slicing = false;
    return change;
}

SceneChange delete_object(Model &model, ModelObject &object, bool invalidate_cut)
{
    if (object.is_cut() && !invalidate_cut)
        throw SceneEditError(422, "cut_object", "Deleting a cut object requires cut invalidation.");
    SceneChange change;
    append_object_entities(object, change.unbindings);
    if (object.is_cut())
        invalidate_cut_group(object);
    model.delete_object(&object);
    return change;
}

SceneChange delete_volume(ModelObject &object, ModelVolume &volume, bool invalidate_cut)
{
    if (volume.is_model_part() && model_part_count(object) == 1)
        throw SceneEditError(422, "invalid_delete", "The last solid part cannot be deleted.");
    if (object.is_cut() && (volume.is_model_part() || volume.type() == ModelVolumeType::NEGATIVE_VOLUME)) {
        if (!invalidate_cut)
            throw SceneEditError(422, "cut_object", "Deleting this cut part requires cut invalidation.");
        invalidate_cut_group(object);
    }
    SceneChange change;
    change.unbindings.push_back({EntityKind::volume, volume.id().id});
    const size_t index = size_t(std::find(object.volumes.begin(), object.volumes.end(), &volume) - object.volumes.begin());
    const std::uint64_t retained_id = object.volumes.size() == 2 ? object.volumes[1 - index]->id().id : 0;
    object.delete_volume(index);
    if (object.volumes.size() == 1) {
        change.rebindings.push_back({EntityKind::volume, retained_id, object.volumes[0]->id().id});
        if (!object.volumes[0]->config.empty()) {
            object.config.apply(object.volumes[0]->config);
            object.volumes[0]->config.reset();
        }
    }
    object.invalidate_bounding_box();
    return change;
}

SceneChange delete_instance(ModelObject &object, size_t index)
{
    if (object.instances.size() == 1)
        throw SceneEditError(422, "invalid_delete", "The last instance cannot be deleted.");
    if (object.is_cut())
        throw SceneEditError(422, "cut_object", "Instances of cut objects cannot be deleted.");
    SceneChange change;
    change.unbindings.push_back({EntityKind::instance, object.instances.at(index)->id().id});
    object.delete_instance(index);
    object.invalidate_bounding_box();
    return change;
}

} // namespace Slic3r::Rest
