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
    return type >= ModelVolumeType::MODEL_PART && type <= ModelVolumeType::SUPPORT_INTERFACE_COLUMN &&
        type != ModelVolumeType::SEAM_POSITION_INSIDE_CENTER;
}

size_t model_part_count(const ModelObject &object)
{
    return size_t(std::count_if(object.volumes.begin(), object.volumes.end(),
        [](const ModelVolume *volume) { return volume->is_model_part(); }));
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
    if (type == ModelVolumeType::SUPPORT_COLUMN || type == ModelVolumeType::NO_SUPPORT_ZONE ||
        type == ModelVolumeType::SUPPORT_INTERFACE_COLUMN)
        return shape == "box" || shape == "cylinder";
    if (type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME ||
        type == ModelVolumeType::PARAMETER_MODIFIER || type == ModelVolumeType::SUPPORT_BLOCKER ||
        type == ModelVolumeType::SUPPORT_ENFORCER || type == ModelVolumeType::SEAM_POSITION_INSIDE)
        return shape == "box" || shape == "cylinder" || shape == "sphere" || shape == "slab";
    return false;
}

} // namespace

// Invalidate the source object's cut identity and every object carrying the same identity.
void invalidate_cut_group(ModelObject &object)
{
    const CutObjectBase cut = object.cut_id;
    for (ModelObject *candidate : object.get_model()->object_ptrs())
        if (candidate->cut_id.is_equal(cut))
            candidate->invalidate_cut();
}

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
        {"brim_negative", ModelVolumeType::BRIM_NEGATIVE}, {"support_column", ModelVolumeType::SUPPORT_COLUMN},
        {"no_support_zone", ModelVolumeType::NO_SUPPORT_ZONE}, {"support_interface_column", ModelVolumeType::SUPPORT_INTERFACE_COLUMN}
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
    for (const std::pair<const t_layer_height_range, ModelConfig> &range : object.layer_config_ranges)
        out.push_back({EntityKind::layer_range, object.id().id, range.first});
}

bool is_loadable_part_type(ModelVolumeType type)
{
    return type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME ||
        type == ModelVolumeType::PARAMETER_MODIFIER || type == ModelVolumeType::SUPPORT_BLOCKER ||
        type == ModelVolumeType::SUPPORT_ENFORCER || type == ModelVolumeType::BRIM_PATCH ||
        type == ModelVolumeType::BRIM_NEGATIVE || type == ModelVolumeType::SUPPORT_COLUMN ||
        type == ModelVolumeType::NO_SUPPORT_ZONE || type == ModelVolumeType::SUPPORT_INTERFACE_COLUMN;
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

SceneChange add_part(ModelObject &target, Model imported, ModelVolumeType type, const std::string &client_file_name)
{
    if (!is_loadable_part_type(type))
        throw SceneEditError(422, "invalid_part_type", "This part type cannot be loaded.");
    if (target.is_cut() && (type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME))
        throw SceneEditError(422, "cut_object", "A solid or negative part cannot be added to a cut object.");
    if (!target.origin_translation.isZero())
        for (ModelObject *object : imported.object_ptrs()) {
            object->center_around_origin();
            for (ModelVolume *volume : object->volumes)
                volume->translate(target.origin_translation - object->origin_translation);
        }
    TriangleMesh mesh = imported.mesh();
    if (mesh.empty())
        throw SceneEditError(422, "invalid_model", "The uploaded model has no mesh.");
    ModelVolume *volume = target.add_volume(std::move(mesh), type);
    volume->name = client_file_name;
    volume->config.set_key_value("extruder", new ConfigOptionInt(0));
    volume->source.input_file = client_file_name;
    int target_index = 0;
    for (ModelObject *object : target.get_model()->object_ptrs()) {
        if (object == &target)
            break;
        ++target_index;
    }
    volume->source.object_idx = target_index;
    volume->source.volume_idx = int(target.volumes.size()) - 1;
    if (imported.objects().size() == 1 && imported.get_object(0).volumes.size() == 1)
        volume->source.mesh_offset = imported.get_object(0).volumes.front()->source.mesh_offset;
    volume->set_offset(volume->source.mesh_offset - target.volumes.front()->source.mesh_offset);
    target.sort_volumes(true);
    target.invalidate_bounding_box();
    SceneChange change;
    change.bindings.push_back({EntityKind::volume, volume->id().id});
    return change;
}

SceneChange add_shape(ModelObject &target, size_t instance_index, const ShapeRequest &request)
{
    if (!shape_allowed(request.type, request.shape))
        throw SceneEditError(422, "invalid_shape", "This shape is not available for the requested part type.");
    if (target.is_cut() && (request.type == ModelVolumeType::MODEL_PART || request.type == ModelVolumeType::NEGATIVE_VOLUME))
        throw SceneEditError(422, "cut_object", "A solid or negative shape cannot be added to a cut object.");
    if (request.size && (!request.size->allFinite() || (request.size->array() <= 0).any()))
        throw SceneEditError(422, "invalid_shape", "Shape size must contain three positive finite values.");
    if (request.position && !request.position->allFinite())
        throw SceneEditError(422, "invalid_shape", "Shape position must contain three finite values.");
    const BoundingBoxf3 bb = target.instance_bounding_box(instance_index);
    const Geometry::Transformation &instance = target.instances.at(instance_index)->get_transformation();
    indexed_triangle_set its;
    if (request.shape == "box") its = its_make_cube(100, 100, 100);
    else if (request.shape == "cylinder") its = its_make_cylinder(25, 100);
    else if (request.shape == "sphere") its = its_make_sphere(50, PI / 18);
    else if (request.shape == "slab") its = its_make_cube(1.5 * bb.size().x(), 1.5 * bb.size().y(), 0.5 * bb.size().z());
    else if (request.shape == "square") its = its_make_cube(100, 100, 0.1);
    else if (request.shape == "circle") its = its_make_cylinder(5, 0.1);
    else if (request.shape == "small_sphere") its = its_make_sphere(10, PI / 18);
    else its = its_make_cylinder(10, 100);
    TriangleMesh mesh(std::move(its));
    if (request.size) {
        const Vec3f actual = mesh.bounding_box().size().cast<float>();
        // Mesh vertices and scaling use floats. Validate the factors after that
        // conversion: finite positive doubles may overflow or round down to zero.
        const Vec3f factors = (request.size->cast<float>().array() / actual.array()).matrix();
        if (!factors.allFinite() || (factors.array() <= 0).any())
            throw SceneEditError(422, "invalid_shape", "Shape size cannot be represented by positive finite mesh scale factors.");
        mesh.scale(factors);
    }
    // Check the generated mesh before adding a volume: scaling can overflow its
    // bounds, and an instance with no height produces a degenerate default slab.
    const BoundingBoxf3 mesh_bb = mesh.bounding_box();
    if (!mesh_bb.min.allFinite() || !mesh_bb.max.allFinite() || !mesh_bb.size().allFinite() ||
        (mesh_bb.size().array() <= 0).any())
        throw SceneEditError(422, "invalid_shape", "Shape bounds must be finite and have three positive dimensions.");
    ModelVolume *volume = target.add_volume(std::move(mesh), request.type);
    volume->set_transformation(instance.get_matrix_no_offset().inverse());
    Vec3d offset;
    if (request.shape == "slab") {
        const Vec3d center = bb.center().cast<double>() - instance.get_offset();
        offset = Vec3d(center.x(), center.y(), 0.5 * mesh_bb.size().z() + bb.min.z() - instance.get_offset().z());
    } else if (request.position) {
        offset = *request.position - instance.get_offset();
    } else {
        offset = Vec3d(bb.max.x(), bb.min.y(), bb.min.z()) + 0.5 * mesh_bb.size().cast<double>() - instance.get_offset();
    }
    volume->set_offset(instance.get_matrix_no_offset().inverse() * offset);
    std::string base = "Generic";
    if (volume->is_seam_position()) base = "Seam";
    else if (volume->is_brim()) base = "Brim";
    else if (volume->is_support_enforcer()) base = "Support";
    else if (volume->is_support_blocker()) base = "Blocker";
    else if (volume->is_support_column()) base = "Column";
    else if (volume->is_no_support_zone()) base = "NoSupport";
    else if (volume->is_support_interface_column()) base = "InterfaceColumn";
    std::string display = request.shape;
    if (display == "small_sphere") display = "sphere";
    if (display == "small_cylinder") display = "cylinder";
    display[0] = char(std::toupper(static_cast<unsigned char>(display[0])));
    volume->name = base + "-" + display;
    volume->config.set_key_value("extruder", new ConfigOptionInt(0));
    volume->source.is_from_builtin_objects = true;
    target.sort_volumes(true);
    target.invalidate_bounding_box();
    SceneChange change;
    change.bindings.push_back({EntityKind::volume, volume->id().id});
    return change;
}

SceneChange split_object(Model &model, ModelObject &object)
{
    if (object.is_cut())
        throw SceneEditError(422, "cut_object", "Cut objects cannot be split.");
    if (object.volumes.size() == 1 && !object.volumes[0]->is_splittable())
        throw SceneEditError(422, "not_splittable", "The object has only one solid shell.");
    for (ModelVolume *volume : object.volumes)
        volume->reset_extra_facets();
    ModelObjectPtrs created;
    object.split(&created);
    if (created.size() <= 1) {
        for (ModelObject *candidate : created)
            model.delete_object(candidate);
        throw SceneEditError(422, "not_splittable", "The object has only one solid shell.");
    }
    SceneChange change;
    for (ModelObject *candidate : created) {
        candidate->sort_volumes(true);
        candidate->ensure_on_bed();
        append_object_entities(*candidate, change.bindings);
    }
    append_object_entities(object, change.unbindings);
    model.delete_object(&object);
    return change;
}

SceneChange split_volume(ModelObject &object, ModelVolume &volume, unsigned int extruders)
{
    if (!volume.is_splittable())
        throw SceneEditError(422, "not_splittable", "The part has only one shell.");
    for (ModelVolume *candidate : object.volumes)
        candidate->reset_extra_facets();
    SceneChange change;
    change.unbindings.push_back({EntityKind::volume, volume.id().id});
    const size_t index = size_t(std::find(object.volumes.begin(), object.volumes.end(), &volume) - object.volumes.begin());
    const size_t count = volume.split(extruders);
    std::vector<ModelVolume *> pieces(object.volumes.begin() + index, object.volumes.begin() + index + count);
    object.input_file.clear();
    object.sort_volumes(true);
    for (ModelVolume *piece : pieces)
        change.bindings.push_back({EntityKind::volume, piece->id().id});
    object.invalidate_bounding_box();
    return change;
}

SceneChange merge_objects(Model &model, const std::vector<ModelObject *> &sources, const std::string &name)
{
    validate_name(name);
    if (sources.size() < 2)
        throw SceneEditError(422, "invalid_merge", "At least two objects are required.");
    std::set<ModelObject *> distinct;
    for (ModelObject *source : sources) {
        if (!distinct.insert(source).second || source->instances.size() != 1)
            throw SceneEditError(422, "invalid_merge", "Merge objects must be distinct and have one instance.");
        if (source->is_cut())
            throw SceneEditError(422, "cut_object", "Cut objects cannot be merged.");
    }
    ModelObject *merged = model.merge_objects_into_multipart(sources, name, true);
    SceneChange change;
    append_object_entities(*merged, change.bindings);
    for (ModelObject *source : sources) {
        change.unbindings.push_back({EntityKind::object, source->id().id});
        for (ModelInstance *instance : source->instances)
            change.unbindings.push_back({EntityKind::instance, instance->id().id});
        for (const std::pair<const t_layer_height_range, ModelConfig> &range : source->layer_config_ranges)
            change.unbindings.push_back({EntityKind::layer_range, source->id().id, range.first});
    }
    for (ModelObject *source : sources)
        model.delete_object(source);
    return change;
}

} // namespace Slic3r::Rest
