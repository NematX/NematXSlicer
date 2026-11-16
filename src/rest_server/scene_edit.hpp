// Scene-edit operations apply GUI-compatible model rules without depending on HTTP.
#pragma once

#include "session.hpp"
#include "libslic3r/Model.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace Slic3r::Rest {

// A refused edit carries both the HTTP status and stable JSON error code.
class SceneEditError : public std::runtime_error {
public:
    SceneEditError(int status, std::string code, const std::string &message);
    int status() const noexcept { return status_; }
    const std::string &code() const noexcept { return code_; }
private:
    int status_;
    std::string code_;
};

// Session mapping changes are committed atomically with the edited scene.
struct SceneChange {
    bool changed = true;
    bool affects_slicing = true;
    std::vector<EntityBinding> bindings;
    std::vector<EntityBinding> unbindings;
    std::vector<EntityRebinding> rebindings;
};

struct ShapeRequest {
    ModelVolumeType type;
    std::string shape;
    std::optional<Vec3d> size;
    std::optional<Vec3d> position;
};

std::optional<ModelVolumeType> parse_part_type(std::string_view name);
inline std::string part_type_name(ModelVolumeType type)
{
    static const char *names[] = {"part", "negative_volume", "modifier", "support_blocker", "support_enforcer",
        "seam_position_center", "seam_position_center_z", "seam_position_inside_center", "seam_position_inside",
        "brim_patch", "brim_negative", "support_column", "no_support_zone"};
    const int index = int(type);
    return index >= 0 && index < int(sizeof(names) / sizeof(names[0])) ? names[index] : "invalid";
}
inline nlohmann::json painted_keys(const ModelVolume &volume)
{
    nlohmann::json result = nlohmann::json::array();
    if (!volume.supported_facets.empty())
        result.push_back("support");
    if (!volume.mm_segmentation_facets.empty())
        result.push_back("mmu");
    for (const std::string &key : volume.facets_annotation_keys())
        result.push_back(key);
    return result;
}
void append_object_entities(const ModelObject &object, std::vector<EntityBinding> &out);
bool is_loadable_part_type(ModelVolumeType type);
// Invalidate the cut identity and cut metadata of every object in the same group.
void invalidate_cut_group(ModelObject &object);

SceneChange set_part_type(ModelObject &object, ModelVolume &volume, ModelVolumeType type);
SceneChange rename_object(ModelObject &object, const std::string &name);
SceneChange rename_volume(ModelVolume &volume, const std::string &name);
SceneChange delete_object(Model &model, ModelObject &object, bool invalidate_cut);
SceneChange delete_volume(ModelObject &object, ModelVolume &volume, bool invalidate_cut);
SceneChange delete_instance(ModelObject &object, size_t index);
SceneChange add_part(ModelObject &target, Model imported, ModelVolumeType type, const std::string &client_file_name);
SceneChange add_shape(ModelObject &target, size_t instance_index, const ShapeRequest &request);
SceneChange split_object(Model &model, ModelObject &object);
SceneChange split_volume(ModelObject &object, ModelVolume &volume, unsigned int extruders);
SceneChange merge_objects(Model &model, const std::vector<ModelObject *> &sources, const std::string &name);

} // namespace Slic3r::Rest
