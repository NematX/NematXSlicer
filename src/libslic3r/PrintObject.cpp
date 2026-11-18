///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Hejl @hejllukas, Pavel Mikuš @Godrak, Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966, Oleksandra Iushchenko @YuSanka, David Kocík @kocikdav, Roman Beránek @zavorka
///|/ Copyright (c) 2021 Justin Schuh @jschuh
///|/ Copyright (c) 2021 Ilya @xorza
///|/ Copyright (c) 2016 Joseph Lenox @lordofhyphens
///|/ Copyright (c) Slic3r 2014 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2015 Maksim Derbasov @ntfshard
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PrintObject.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include "NativeAdhesionLayerHelpers.hpp"
#include "Api/internal/PrintObjectAccess.hpp"
#include "BoundingBox.hpp"
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "Geometry.hpp"
#include "Layer.hpp"
#include "Print.hpp"
#include "Config/PrintConfig.hpp"
#include "PrintObjectRegion.hpp"
#include "PrintRegion.hpp"
#include "Slicing.hpp"
#include "Steps/StepPipeline.hpp"
#include "Thread.hpp"
#include "TriangleMeshSlicer.hpp"
#include "TriangleSelectorWrapper.hpp"
#include "Utils.hpp"

/*
Maintain the sliced representation of one model object for Print.

PrintObject owns its object layers and auxiliary layers. Regions connect those
layers to effective settings; instances describe the physical copies that share
this sliced geometry. Preparing or clearing layers therefore changes data used
by later surface, extrusion and ordering stages, not the source model mesh.

The object supplies slicing parameters, layer creation and cleanup, and access
to the generated islands. Its owning Print coordinates recomputation. References
to layers and their descendants must not survive a rebuild of those collections.
Region zero is the object's fallback for helper geometry outside all volume
masks. Its configuration combines Print defaults with object overrides, using
the same normalization as volume regions but no volume or layer-range override.
Hull bounds use the transformed printable model above the bed, centered like the
slices. Ordering adds each copy's shift to compare its actual edges on the plate.

Representative direct calls (branches are not a timeline):

    PrintObject::update_slicing_parameters()
        |-- SlicingParameters::create_from_config()
        |-- default_region_config()
        `-- this->object_extruders()

    object_default_region_config()
        |-- apply_to_print_region_config(): apply only object-level overrides
        `-- finalize_region_config(): share extruder, density and fuzzy-skin rules

    PrintObject::convex_hull_bounding_box()
        |-- ModelObject::convex_hull_2d(): transform printable volumes above the bed
        `-- BoundingBox::translate(): subtract the slicing center offset
*/

using namespace std::literals;

#ifdef PRINT_OBJECT_TIMING
    #define PRINT_OBJECT_TIME_LIMIT_DEFAULT 50
    #include <boost/current_function.hpp>
    #include "Timer.hpp"
    #define PRINT_OBJECT_TIME_LIMIT_MILLIS(limit) Timing::TimeLimitAlarm time_limit_alarm(uint64_t(limit) * 1000000l, BOOST_CURRENT_FUNCTION)
#else
    #define PRINT_OBJECT_TIME_LIMIT_MILLIS(limit) do {} while(false)
#endif

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
#define SLIC3R_DEBUG
#endif

#ifdef SLIC3R_DEBUG
#ifndef _DEBUG
#undef NDEBUG
#define DEBUG
#define _DEBUG
#include "SVG.hpp"
#undef assert
#include <cassert>
#endif
#endif

namespace Slic3r {

// Normalize extruder indices, near-zero infill and unusable fuzzy skin in the
// owned configuration; this is shared by object defaults and volume regions.
static void finalize_region_config(PrintRegionConfig &config, size_t num_extruders);
// Return owned Print defaults with object-level regional overrides, excluding
// layer ranges and volumes. Sources remain unchanged; allocation errors propagate.
PrintRegionConfig object_default_region_config(const PrintRegionConfig &print_default_region_config,
                                               const ModelObject &object, size_t num_extruders);

void ApiInternal::PrintObjectAccess::set_layer_profile(PrintObject &object, std::vector<coord_t> &&layer_profile)
{
    object.m_layer_profile = std::move(layer_profile);
}

void ApiInternal::PrintObjectAccess::replace_layers_by_moving_contents(PrintObject &object, LayerUPtrs &&new_layers)
{
    object.clear_layers();
    object.m_layers = std::move(new_layers);
}

PrintInstances &ApiInternal::PrintObjectAccess::mutable_instances(PrintObject &object)
{
    return object.m_instances;
}

void ApiInternal::PrintObjectAccess::clear_brim_auxiliary_layers(PrintObject &object)
{
    remove_auxiliary_layers(object, layer_is_brim_adhesion);
}

void ApiInternal::PrintObjectAccess::clear_skirt_auxiliary_layers(PrintObject &object)
{
    remove_auxiliary_layers(object, layer_is_any_skirt_adhesion);
}

const ExtrusionEntityCollection &PrintObject::brim() const
{
    m_legacy_brim_cache.clear();
    std::vector<ExtrusionEntityCollection> by_instance(instances().size());
    bool marked = false;
    for (const Layer &layer : auxiliary_layers()) {
        if (!layer_is_brim_adhesion(layer))
            continue;
        if (layer.get_property<LayerInstanceProperty>() == nullptr) {
            append_auxiliary_layer_extrusions(layer, m_legacy_brim_cache);
            continue;
        }
        const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(this));
        const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
        for (const std::pair<uint32_t, c_point> &projection : slic3r_api::layer_instance_projections(object_view, layer_view)) {
            ExtrusionEntityCollection copy;
            append_auxiliary_layer_extrusions(layer, copy);
            translate_adhesion_tree(copy, Point(projection.second.x, projection.second.y));
            by_instance[projection.first].append(std::move(copy));
        }
        marked = true;
    }
    if (marked)
        for (ExtrusionEntityCollection &copy : by_instance)
            m_legacy_brim_cache.append(std::move(copy));
    return m_legacy_brim_cache;
}

ExtrusionEntityCollection PrintObject::brim_in_plateau_coordinates() const
{
    ExtrusionEntityCollection result;
    const slic3r_api::Object object_view(reinterpret_cast<const object_handle *>(this));
    for (const Layer &layer : auxiliary_layers()) {
        if (!layer_is_brim_adhesion(layer))
            continue;
        const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
        for (const std::pair<uint32_t, c_point> &projection : slic3r_api::layer_instance_projections(object_view, layer_view)) {
            ExtrusionEntityCollection copy;
            append_auxiliary_layer_extrusions(layer, copy);
            translate_adhesion_tree(copy, Point(projection.second.x, projection.second.y));
            result.append(std::move(copy));
        }
    }
    return result;
}

const ExtrusionEntityCollection &PrintObject::skirt() const
{
    m_legacy_skirt_cache.clear();
    collect_auxiliary_layer_extrusions(*this, layer_is_normal_skirt_adhesion, m_legacy_skirt_cache);
    return m_legacy_skirt_cache;
}

const std::optional<ExtrusionEntityCollection> &PrintObject::skirt_first_layer() const
{
    m_legacy_skirt_first_layer_cache.reset();
    if (has_auxiliary_layer(*this, layer_is_first_layer_skirt_adhesion)) {
        m_legacy_skirt_first_layer_cache.emplace();
        collect_auxiliary_layer_extrusions(
            *this, layer_is_first_layer_skirt_adhesion, *m_legacy_skirt_first_layer_cache);
    }
    return m_legacy_skirt_first_layer_cache;
}

// Constructor is called from the main thread, therefore all Model / ModelObject / ModelIntance data are valid.
PrintObject::PrintObject(Print* print, ModelObject* model_object, const Transform3d& trafo, PrintInstances&& instances) :
    PrintObjectBase(model_object),
    m_print(print),
    m_trafo(trafo)
{
    // Compute centering offet to be applied to our meshes so that we work with smaller coordinates
    // requiring less bits to represent Clipper coordinates.

	// Snug bounding box of a rotated and scaled object by the 1st instantion, without the instance translation applied.
	// All the instances share the transformation matrix with the exception of translation in XY and rotation by Z,
	// therefore a bounding box from 1st instance of a ModelObject is good enough for calculating the object center,
	// snug height and an approximate bounding box in XY.
    BoundingBoxf3  bbox        = model_object->raw_bounding_box();
    Vec3d 		   bbox_center = bbox.center();
	// We may need to rotate the bbox / bbox_center from the original instance to the current instance.
    double z_diff = Geometry::rotation_diff_z(model_object->instances.front()->get_matrix(), instances.front().model_instance->get_matrix());
    if (std::abs(z_diff) > EPSILON) {
		auto z_rot  = Eigen::AngleAxisd(z_diff, Vec3d::UnitZ());
		bbox 		= bbox.transformed(Transform3d(z_rot));
		bbox_center = (z_rot * bbox_center).eval();
	}

    // Center of the transformed mesh (without translation).
    m_center_offset = Point::new_scale(bbox_center.x(), bbox_center.y());
    // Size of the transformed mesh. This bounding may not be snug in XY plane, but it is snug in Z.
    m_size = (bbox.size() * (1. / SCALING_FACTOR)).cast<coord_t>();
    m_size.z() = coord_t(model_object->max_z() * (1. / SCALING_FACTOR));

    this->set_instances(std::move(instances));

    //create config hierarchy
    m_config.parent = &print->config();
}

PrintObject::PrintObject(Print* print,
                         const Vec3crd &size,
                         std::shared_ptr<PrintObjectRegions> shared_regions) :
    PrintObjectBase(nullptr),
    m_print(print),
    m_size(size),
    m_shared_regions(std::move(shared_regions))
{
    m_config.parent = &print->config();
    m_config.apply(print->default_object_config());

    PrintInstance instance;
    instance.print_object = this;
    instance.model_instance = nullptr;
    instance.shift = Point::new_scale(0., 0.);
    m_instances.push_back(instance);
}

PrintObject::~PrintObject() = default;

void PrintObject::throw_if_canceled()
{
    m_print->throw_if_canceled();
}

Transform3d PrintObject::trafo_centered() const {
    Transform3d t = this->trafo();
    t.pretranslate(Vec3d(-unscaled(m_center_offset.x()), -unscaled(m_center_offset.y()), 0));
    return t;
}

BoundingBox PrintObject::bounding_box() const {
    return BoundingBox(Point(-m_size.x() / 2, -m_size.y() / 2), Point(m_size.x() / 2, m_size.y() / 2));
}

BoundingBox PrintObject::convex_hull_bounding_box() const
{
    if (model_object() == nullptr)
        return {};
    BoundingBox bounds(model_object()->convex_hull_2d(trafo()).points);
    if (bounds.defined)
        bounds.translate(-center_offset().x(), -center_offset().y());
    return bounds;
}

bool PrintObject::is_mm_painted() const { return this->model_object()->is_mm_painted(); }

PrintBase::ApplyStatus PrintObject::set_instances(PrintInstances &&instances) {
    for (PrintInstance &i : instances) {
        // Add the center offset, which will be subtracted from the mesh when slicing.
        i.shift += m_center_offset;
    }
    // Invalidate and set copies.
    PrintBase::ApplyStatus status = PrintBase::APPLY_STATUS_UNCHANGED;
    bool equal_length = instances.size() == m_instances.size();
    bool equal = equal_length && std::equal(instances.begin(), instances.end(), m_instances.begin(),
        [](const PrintInstance& lhs, const PrintInstance& rhs) { return lhs.model_instance == rhs.model_instance && lhs.shift == rhs.shift; });
    if (! equal) {
        status = PrintBase::APPLY_STATUS_CHANGED;
        // Initial construction has no published instance data or plugin borrows to invalidate.
        if (!m_instances.empty()) {
            bool invalidated = m_print->request_replay_from(Steps::checkpoint_before_step(STEP_SKIRT_BRIM));
            if (!equal_length)
                invalidated |= m_print->request_replay_from(Steps::checkpoint_before_step(STEP_WIPETOWER));
            if (invalidated)
                status = PrintBase::APPLY_STATUS_INVALIDATED;
        }
        m_instances = std::move(instances);
        for (PrintInstance &i : m_instances) {
            i.print_object = this;
        }
    }
    return status;
}

std::vector<std::reference_wrapper<const PrintRegion>> PrintObject::all_regions() const
{
    std::vector<std::reference_wrapper<const PrintRegion>> out;
    out.reserve(m_shared_regions->all_regions.size());
    for (const Slic3r::PrintRegion *region : m_shared_regions->all_regions) {
        out.emplace_back(*region);
    }
    return out;
}

size_t PrintObject::num_printing_regions() const throw() {
    assert(m_shared_regions);
    return m_shared_regions->all_regions.size();
}

const PrintRegion &PrintObject::printing_region(size_t idx) const throw() {
    assert(m_shared_regions);
    return *m_shared_regions->all_regions[idx];
}

const PrintRegionConfig &PrintObject::default_region_config(const PrintRegionConfig &from_print) const {
    if (this->m_shared_regions && num_printing_regions() > 0) {
        return printing_region(0).config();
    }
    return from_print;
}

bool PrintObject::has_brim() const {
    bool has_brim_volume = false;
    for (const ModelVolume *volume : this->model_object()->volumes) {
        if (volume->is_brim_patch()) {
            has_brim_volume = true;
        }
    }
    return has_brim_volume || ((this->config().brim_width.value > 0 && this->config().brim_width_interior.value > 0)
        && !this->has_raft());
}

Polygons PrintObject::get_brim_patch(ModelVolumeType brim_type, const PrintInstance *instance /*= nullptr*/) const {
    Polygons polys;
    for (const ModelVolume *v : this->model_object()->volumes) {
        assert(v);
        if (v->type() == brim_type) {
            if (instance == nullptr) {
                for (const PrintInstance &inst : this->instances()) {
                    Polygons vol_outline;
                    auto transl = Transform3d::Identity();
                    assert(inst.model_instance);
                    vol_outline = project_mesh(v->mesh().its,
                                               transl * inst.model_instance->get_matrix() * v->get_matrix(), [] {});
                    append(polys, vol_outline);
                }
            } else {
                Polygons vol_outline;
                auto transl = Transform3d::Identity();
                assert(instance->model_instance);
                vol_outline = project_mesh(v->mesh().its,
                                            transl * instance->model_instance->get_matrix() * v->get_matrix(), [] {});
                append(polys, vol_outline);
            }
        }
    }
    coord_t scaled_brim_resolution = std::max(SCALED_EPSILON * 10, scale_i(this->print()->config().resolution.value));
    return ensure_valid(union_(polys), scaled_brim_resolution);
}

void PrintObject::clear_layers()
{
    m_layers.clear();
}

void PrintObject::clear_auxiliary_layers()
{
    m_auxiliary_layers.clear();
}

Layer& PrintObject::add_auxiliary_layer(size_t id, coord_t height, coord_t print_z)
{
    m_auxiliary_layers.emplace_back(new Layer(id, this, height, print_z, -1., true));
    return *m_auxiliary_layers.back();
}

LayerUPtrs::iterator PrintObject::insert_auxiliary_layer(LayerUPtrs::const_iterator pos,
                                                         size_t id,
                                                         coord_t height,
                                                         coord_t print_z,
                                                         double slice_z) {
        return m_auxiliary_layers.insert(pos, std::unique_ptr<Layer>(new Layer(id, this, height, print_z, slice_z, true)));
}

static void clamp_exturder_to_default(ConfigOptionInt &opt, size_t num_extruders)
{
    if (opt.value > (int)num_extruders)
        // assign the default extruder
        opt.value = 1;
}

PrintObjectConfig PrintObject::object_config_from_model_object(const PrintObjectConfig &default_object_config, const ModelObject &object, size_t num_extruders)
{
    PrintObjectConfig config = default_object_config;
    {
        DynamicPrintConfig src_normalized(object.config.get());
        src_normalized.normalize_fdm();
        config.apply(src_normalized, true);
    }
    // Clamp invalid extruders to the default extruder (with index 1).
    clamp_exturder_to_default(config.support_material_extruder,           num_extruders);
    clamp_exturder_to_default(config.support_material_interface_extruder, num_extruders);
    return config;
}

const std::string                                                    key_extruder { "extruder" };
static constexpr const std::initializer_list<const std::string_view> keys_extruders { "infill_extruder"sv, "solid_infill_extruder"sv, "perimeter_extruder"sv };

static void apply_to_print_region_config(PrintRegionConfig &out, const DynamicPrintConfig &in)
{
    // 1) Copy the "extruder key to infill_extruder and perimeter_extruder.
    auto *opt_extruder = in.opt<ConfigOptionInt>(key_extruder);
    if (opt_extruder)
        if (int extruder = opt_extruder->value; extruder != 0) {
            // Not a default extruder.
            out.infill_extruder      .value = extruder;
            out.solid_infill_extruder.value = extruder;
            out.perimeter_extruder   .value = extruder;
        }
    // 2) Copy the rest of the values.
    for (auto it = in.cbegin(); it != in.cend(); ++ it)
        if (it->first != key_extruder)
            if (ConfigOption* my_opt = out.option(it->first, false); my_opt != nullptr) {
                if (one_of(it->first, keys_extruders)) {
                    assert(dynamic_cast<ConfigOptionInt*>(my_opt));
                    // Ignore "default" extruders.
                    int extruder = static_cast<const ConfigOptionInt*>(it->second.get())->value;
                    if (extruder > 0)
                        static_cast<ConfigOptionInt *>(my_opt)->value = (extruder);
                } else
                    my_opt->set(*it->second);
            }
}

static void finalize_region_config(PrintRegionConfig &config, size_t num_extruders)
{
    clamp_exturder_to_default(config.infill_extruder,       num_extruders);
    clamp_exturder_to_default(config.perimeter_extruder,    num_extruders);
    clamp_exturder_to_default(config.solid_infill_extruder, num_extruders);
    if (config.fill_density.value < 0.00011f)
        // Switch of infill for very low infill rates, also avoid division by zero in infill generator for these very low rates.
        // See GH issue #5910.
        config.fill_density.value = 0;
    else
        config.fill_density.value = std::min(config.fill_density.value, 100.);
    if (config.fuzzy_skin.value != FuzzySkinType::None && (config.fuzzy_skin_point_dist.value < 0.01 || config.fuzzy_skin_thickness.value < 0.001))
        config.fuzzy_skin.value = FuzzySkinType::None;
}

PrintRegionConfig region_config_from_model_volume(const PrintRegionConfig &default_or_parent_region_config, const DynamicPrintConfig *layer_range_config, const ModelVolume &volume, size_t num_extruders)
{
    PrintRegionConfig config = default_or_parent_region_config;
    if (volume.is_model_part()) {
        // default_or_parent_region_config is the object's default region config (see
        // object_default_region_config()). Applying the ModelObject's region values
        // again is harmless and keeps this function correct from the Print defaults.
        apply_to_print_region_config(config, volume.get_object()->config.get());
    } else {
        // default_or_parent_region_config contains parent PrintRegion config, which already contains ModelVolume's config.
    }
    if (layer_range_config != nullptr) {
        // Not applicable to modifiers.
        assert(volume.is_model_part());
    	apply_to_print_region_config(config, *layer_range_config);
    }
    apply_to_print_region_config(config, volume.config.get());
    if (! volume.material_id().empty())
        apply_to_print_region_config(config, volume.material()->config.get());
    finalize_region_config(config, num_extruders);
    return config;
}

PrintRegionConfig object_default_region_config(const PrintRegionConfig &print_default_region_config, const ModelObject &object, size_t num_extruders)
{
    // The Print's defaults overridden by the object's own region settings, without
    // any layer range or volume setting. This is the object's fallback region: the
    // settings of helper geometry outside every part, such as skirt, brim or support.
    PrintRegionConfig config = print_default_region_config;
    apply_to_print_region_config(config, object.config.get());
    finalize_region_config(config, num_extruders);
    return config;
}

void PrintObject::update_slicing_parameters()
{
    if (!m_slicing_params || !m_slicing_params->valid)
        m_slicing_params = SlicingParameters::create_from_config(
            this->print()->config(), m_config, this->print()->default_region_config(), this->model_object()->max_z()/*bounding_box().max.z()*/, this->object_extruders());
}

std::shared_ptr<SlicingParameters> PrintObject::slicing_parameters(const DynamicPrintConfig& full_config, const ModelObject& model_object, float object_max_z)
{
	PrintConfig         print_config;
	PrintObjectConfig   object_config;
	PrintRegionConfig   default_region_config;
	print_config.apply(full_config, true);
	object_config.apply(full_config, true);
	default_region_config.apply(full_config, true);
	size_t              num_extruders = print_config.nozzle_diameter.size();
	object_config = object_config_from_model_object(object_config, model_object, num_extruders);

    std::set<extruder_index_t> object_extruders;
    for (const ModelVolume* model_volume : model_object.volumes)
        if (model_volume->is_model_part()) {
            PrintRegion::collect_object_printing_extruders(
                print_config,
                object_config,
                region_config_from_model_volume(default_region_config, nullptr, *model_volume, num_extruders),
                object_extruders);
            for (const std::pair<const t_layer_height_range, ModelConfig>& range_and_config : model_object.layer_config_ranges)
                if (range_and_config.second.has("perimeter_extruder") ||
                    range_and_config.second.has("infill_extruder") ||
                    range_and_config.second.has("solid_infill_extruder"))
                    PrintRegion::collect_object_printing_extruders(
                        print_config,
                        object_config,
                        region_config_from_model_volume(default_region_config, &range_and_config.second.get(), *model_volume, num_extruders),
                        object_extruders);
        }
    //FIXME add painting extruders

    if (object_max_z <= 0.f)
        object_max_z = (float)model_object.raw_bounding_box().size().z();
    return SlicingParameters::create_from_config(print_config, object_config, default_region_config, object_max_z, object_extruders);
}

// returns 0-based indices of extruders used to print the object (without brim, support and other helper extrusions)
std::set<extruder_index_t> PrintObject::object_extruders() const
{
    std::set<extruder_index_t> extruders;
    for (const PrintRegion &region : this->all_regions())
        region.collect_object_printing_extruders(*this->print(), extruders);
    return extruders;
}

double PrintObject::get_first_layer_height() const
{
    //get object first layer height
    double object_first_layer_height = config().first_layer_height.value;
    if (config().first_layer_height.percent) {
        object_first_layer_height = 1000000000;
        for (extruder_index_t extruder_id : object_extruders()) {
            double nozzle_diameter = print()->config().nozzle_diameter.get_at(extruder_id);
            object_first_layer_height = std::fmin(object_first_layer_height, config().first_layer_height.get_effective_value(nozzle_diameter));
        }
    }
    assert(object_first_layer_height < 1000000000);
    return object_first_layer_height;
}

bool PrintObject::update_layer_height_profile(const ModelObject& model_object, const SlicingParameters& slicing_parameters, std::vector<coordf_t>& layer_height_profile)
{
    bool updated = false;

    if (layer_height_profile.empty()) {
        // use the constructor because the assignement is crashing on ASAN OsX
        layer_height_profile = model_object.layer_height_profile.get();
        // layer_height_profile = model_object.layer_height_profile;
        // The layer height returned is sampled with high density for the UI layer height painting
        // and smoothing tool to work.
        updated = true;
    }

    // Verify the layer_height_profile.
    if (!layer_height_profile.empty() &&
        // Must not be of even length.
        ((layer_height_profile.size() & 1) != 0 ||
            // Last entry must be at the top of the object.
            std::abs(layer_height_profile[layer_height_profile.size() - 2] - slicing_parameters.object_print_z_max + slicing_parameters.object_print_z_min) > 10 * EPSILON)) {
        if ((layer_height_profile.size() & 1) != 0) {
            BOOST_LOG_TRIVIAL(error) << "Error: can't apply the layer hight profile: layer_height_profile array is odd, not even.";
        } else {
            BOOST_LOG_TRIVIAL(error) << "Error: can't apply the layer hight profile: layer_height_profile last layer is at "
                << layer_height_profile[layer_height_profile.size() - 2]
                <<", and it's too far away from object_print_z_max = "<<(slicing_parameters.object_print_z_max + slicing_parameters.object_print_z_min);
        }
        layer_height_profile.clear();
    }
    if (layer_height_profile.empty()) {
        //layer_height_profile = layer_height_profile_adaptive(slicing_parameters, model_object.layer_config_ranges, model_object.volumes);
        layer_height_profile = layer_height_profile_from_ranges(slicing_parameters, model_object.layer_config_ranges);
        // The layer height profile is already compressed.
        updated = true;
    }
    return updated;
}

template<class LayerPtrContainer>
// Project an already selected patch. Filter only a local copy, retain its open
// boundaries, and normalize generated rings with the same helper as mesh slices.
// A painted vertical facet projects to a collinear ring. It is kept: the painting
// consumers widen the projections by a margin before using them.
static void project_triangles_to_slabs(RefView<Layer, LayerPtrContainer> layers, const indexed_triangle_set &source_facets, const Transform3f &tr, bool seam, std::vector<Polygons> &out)
{
    indexed_triangle_set scratch;
    const indexed_triangle_set &custom_facets = mesh_without_degenerate_faces(source_facets, scratch);
    if (custom_facets.indices.empty())
        return;

    const float tr_det_sign = (tr.matrix().determinant() > 0. ? 1.f : -1.f);

    // The projection will be at most a pentagon. Let's minimize heap
    // reallocations by saving in in the following struct.
    // Points are used so that scaling can be done in parallel
    // and they can be moved from to create an ExPolygon later.
    struct LightPolygon {
        LightPolygon() { pts.reserve(5); }
        LightPolygon(const std::array<Vec2f, 3>& tri) {
            pts.reserve(3);
            pts.emplace_back(Point::new_scale(tri.front()));
            pts.emplace_back(Point::new_scale(tri[1]));
            pts.emplace_back(Point::new_scale(tri.back()));
        }

        Points pts;

        void add(const Vec2f& pt) {
            pts.emplace_back(Point::new_scale(pt));
            assert(pts.size() <= 5);
        }

    };

    // Structure to collect projected polygons. One element for each triangle.
    // Saves vector of polygons and layer_id of the first one.
    struct TriangleProjections {
        size_t first_layer_id;
        std::vector<LightPolygon> polygons;
    };

    // Vector to collect resulting projections from each triangle.
    std::vector<TriangleProjections> projections_of_triangles(custom_facets.indices.size());

    // Iterate over all triangles.
    Slic3r::parallel_for(size_t(0), custom_facets.indices.size(),
        [&custom_facets, &tr, tr_det_sign, seam, layers, &projections_of_triangles]
        (const size_t idx) {

        PRINT_OBJECT_TIME_LIMIT_MILLIS(PRINT_OBJECT_TIME_LIMIT_DEFAULT);
        std::array<Vec3f, 3> facet;

        // Transform the triangle into worlds coords.
        for (int i=0; i<3; ++i)
            facet[i] = tr * custom_facets.vertices[custom_facets.indices[idx](i)];

        // Ignore triangles with upward-pointing normal. Don't forget about mirroring.
        float z_comp = (facet[1]-facet[0]).cross(facet[2]-facet[0]).z();
        if (! seam && tr_det_sign * z_comp > 0.)
            return; //continue (next facet idx)

        // The algorithm does not process vertical triangles, but it should for seam.
        // In that case, tilt the triangle a bit so the projection does not degenerate.
        if (seam && z_comp == 0.f)
            facet[0].x() += float(EPSILON);

        // Sort the three vertices according to z-coordinate.
        std::sort(facet.begin(), facet.end(),
                  [](const Vec3f& pt1, const Vec3f&pt2) {
                      return pt1.z() < pt2.z();
                  });

        std::array<Vec2f, 3> trianglef;
        for (int i=0; i<3; ++i)
            trianglef[i] = to_2d(facet[i]);

        // Find lowest slice not below the triangle.
        auto it = std::lower_bound(layers.begin(), layers.end(), facet[0].z()+EPSILON,
                      [](const Layer &l1, float z) {
                           return l1.slice_z < z;
                      });

        // Count how many projections will be generated for this triangle
        // and allocate respective amount in projections_of_triangles.
        size_t first_layer_id = projections_of_triangles[idx].first_layer_id = it - layers.begin();
        size_t last_layer_id  = first_layer_id;
        // The cast in the condition below is important. The comparison must
        // be an exact opposite of the one lower in the code where
        // the polygons are appended. And that one is on floats.
        while (last_layer_id + 1 < layers.size()
            && float(layers[last_layer_id].slice_z) <= facet[2].z())
            ++last_layer_id;

        if (first_layer_id == last_layer_id) {
            // The triangle fits just a single slab, just project it. This also avoids division by zero for horizontal triangles.
            float dz = facet[2].z() - facet[0].z();
            assert(dz >= 0);
            // The face is nearly horizontal and it crosses the slicing plane at first_layer_id - 1.
            // Rather add this face to both the planes.
            bool add_below = dz < float(2. * EPSILON) && first_layer_id > 0 && layers[first_layer_id - 1].slice_z > facet[0].z() - EPSILON;
            projections_of_triangles[idx].polygons.reserve(add_below ? 2 : 1);
            projections_of_triangles[idx].polygons.emplace_back(trianglef);
            if (add_below) {
                -- projections_of_triangles[idx].first_layer_id;
                projections_of_triangles[idx].polygons.emplace_back(trianglef);
            }
            return; //continue (next facet idx)
        }

        projections_of_triangles[idx].polygons.resize(last_layer_id - first_layer_id + 1);

        // Calculate how to move points on triangle sides per unit z increment.
        Vec2f ta(trianglef[1] - trianglef[0]);
        Vec2f tb(trianglef[2] - trianglef[0]);
        ta *= 1.f/(facet[1].z() - facet[0].z());
        tb *= 1.f/(facet[2].z() - facet[0].z());

        // Projection on current slice will be built directly in place.
        LightPolygon* proj = &projections_of_triangles[idx].polygons[0];
        proj->add(trianglef[0]);

        bool passed_first = false;
        bool stop = false;

        // Project a sub-polygon on all slices intersecting the triangle.
        while (it != layers.end()) {
            const float z = float((*it).slice_z);

            // Projections of triangle sides intersections with slices.
            // a moves along one side, b tracks the other.
            Vec2f a;
            Vec2f b;

            // If the middle vertex was already passed, append the vertex
            // and use ta for tracking the remaining side.
            if (z > facet[1].z() && ! passed_first) {
                proj->add(trianglef[1]);
                ta = trianglef[2]-trianglef[1];
                ta *= 1.f/(facet[2].z() - facet[1].z());
                passed_first = true;
            }

            // This slice is above the triangle already.
            if (z > facet[2].z() || it+1 == layers.end()) {
                proj->add(trianglef[2]);
                stop = true;
            }
            else {
                // Move a, b along the side it currently tracks to get
                // projected intersection with current slice.
                a = passed_first ? (trianglef[1]+ta*(z-facet[1].z()))
                                 : (trianglef[0]+ta*(z-facet[0].z()));
                b = trianglef[0]+tb*(z-facet[0].z());
                proj->add(a);
                proj->add(b);
            }

           if (stop)
                break;

            // Advance to the next layer.
            ++it;
            ++proj;
            assert(proj <= &projections_of_triangles[idx].polygons.back() );

            // a, b are first two points of the polygon for the next layer.
            proj->add(b);
            proj->add(a);
        }
    }); // end of parallel_for

    // Make sure that the output vector can be used.
    out.resize(layers.size());

    // Allocate each layer once: reallocating a Polygon vector can invoke its
    // tolerant move constructor and trim a near-closing vertex before cleanup.
    std::vector<size_t> next_polygon(out.size(), 0);
    std::vector<size_t> counts(out.size(), 0);
    for (const TriangleProjections &triangle : projections_of_triangles)
        for (size_t i = 0; i < triangle.polygons.size() && triangle.first_layer_id + i < out.size(); ++i)
            ++counts[triangle.first_layer_id + i];
    for (size_t i = 0; i < out.size(); ++i) {
        next_polygon[i] = out[i].size();
        Polygons combined(out[i].size() + counts[i]);
        for (size_t j = 0; j < out[i].size(); ++j)
            combined[j].points.swap(out[i][j].points);
        out[i].swap(combined);
    }

    // Now append the collected polygons to respective layers.
    for (auto& trg : projections_of_triangles) {
        int layer_id = int(trg.first_layer_id);
        for (LightPolygon &poly : trg.polygons) {
            if (layer_id >= int(out.size()))
                break; // part of triangle could be projected above top layer
            assert(! poly.pts.empty());
            out[layer_id][next_polygon[layer_id]++].points.swap(poly.pts);
            ++layer_id;
        }
    }
    for (Polygons &polygons : out)
        normalize_mesh_polygons(polygons, true);
}

std::vector<Polygons> PrintObject::project_and_append_custom_facets(
        bool seam, EnforcerBlockerType type) const
{
    if (seam)
        return this->project_and_append_custom_facets(std::string("builtin:seam"), type);

    std::vector<Polygons> out;
    for (const ModelVolume* mv : this->model_object()->volumes) {
        if (mv->is_model_part()) {
            indexed_triangle_set custom_facets = mv->supported_facets.get_facets_strict(*mv, type);
            its_remove_degenerate_faces(custom_facets, false);
            if (! custom_facets.indices.empty()) {
                std::vector<Polygons> projected;
                // Support blockers or enforcers. Project downward facing painted areas upwards to their respective slicing plane.
                slice_mesh_slabs(custom_facets, slice_z_from_layers(this->layers()), this->trafo_centered() * mv->get_matrix(), nullptr, &projected, [](){});
                // Merge these projections with the output, layer by layer.
                assert(! projected.empty());
                assert(out.empty() || out.size() == projected.size());
                if (out.empty())
                    out = std::move(projected);
                else
                    for (size_t i = 0; i < out.size(); ++ i)
                        append(out[i], std::move(projected[i]));
            }
        }
    }
    return out;
}

std::vector<Polygons> PrintObject::project_and_append_custom_facets(
        const std::string &painting_key, EnforcerBlockerType type) const
{
    std::vector<Polygons> out;
    for (const ModelVolume *mv : this->model_object()->volumes) {
        if (!mv->is_model_part())
            continue;

        const FacetsAnnotation *annotation = mv->facets_annotation(painting_key);
        if (annotation == nullptr)
            continue;

        // Generic facet annotations use the same geometric meaning as seam
        // painting: selected mesh triangles are projected through all touched
        // layer slabs. Plugins can then interpret the produced polygons inside
        // their own slicing step without needing to know TriangleSelector.
        indexed_triangle_set custom_facets = annotation->get_facets_strict(*mv, type);
        its_remove_degenerate_faces(custom_facets, false);
        if (!custom_facets.indices.empty())
            project_triangles_to_slabs(this->layers(), custom_facets,
                (this->trafo_centered() * mv->get_matrix()).cast<float>(),
                true, out);
    }
    return out;
}

const Layer *PrintObject::get_layer_at_printz(coord_t print_z) const {
    auto it = Slic3r::lower_bound_by_predicate(m_layers.begin(), m_layers.end(),
                                               [print_z](const std::unique_ptr<Layer> &layer) { return layer->scaled_print_z() < print_z; });
    return (it == m_layers.end() || (*it)->scaled_print_z() != print_z) ? nullptr : it->get();
}

Layer *PrintObject::get_layer_at_printz(coord_t print_z) {
    return const_cast<Layer *>(std::as_const(*this).get_layer_at_printz(print_z));
}

// Get a layer approximately at print_z.
const Layer *PrintObject::get_layer_at_printz(double print_z, double epsilon) const {
    double limit = print_z - epsilon;
    auto it = Slic3r::lower_bound_by_predicate(m_layers.begin(), m_layers.end(),
                                               [limit](const std::unique_ptr<Layer> &layer) { return layer->unscaled_print_z() < limit; });
    return (it == m_layers.end() || (*it)->unscaled_print_z() > print_z + epsilon) ? nullptr : it->get();
}

Layer *PrintObject::get_layer_at_printz(double print_z, double epsilon) {
    return const_cast<Layer *>(std::as_const(*this).get_layer_at_printz(print_z, epsilon));
}

const Layer *PrintObject::get_first_layer_below_printz(coord_t print_z) const {
    auto it = Slic3r::lower_bound_by_predicate(m_layers.begin(), m_layers.end(),
                                               [print_z](const std::unique_ptr<Layer> &layer) { return layer->scaled_print_z() < print_z; });
    return (it == m_layers.begin()) ? nullptr : (--it)->get();
}

const Layer *PrintObject::get_first_layer_below_printz(double print_z, double epsilon) const {
    double limit = print_z + epsilon;
    auto it = Slic3r::lower_bound_by_predicate(m_layers.begin(), m_layers.end(),
                                               [limit](const std::unique_ptr<Layer> &layer) { return layer->unscaled_print_z() < limit; });
    return (it == m_layers.begin()) ? nullptr : (--it)->get();
}

} // namespace Slic3r
