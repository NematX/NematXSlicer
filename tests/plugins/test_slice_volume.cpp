/* Slice volume plugin tests, and the baseline dump of the object slicing.

   The dump case ([slicing-dump], hidden: it runs only when asked by name or by tag) slices several real meshes through the plugin
   pipeline (layer height generation, then STEP_SLICING) with a list of settings and writes, for every object, layer and region, every
   point of the raw slices exactly as stored: no sorting, no canonical form. It is the gate of the slicing refactors: a change of the
   slicing code must give a byte-identical file.
   - SLIC3R_SLICING_DUMP=<file>      write the dump to the file.
   - SLIC3R_SLICING_DUMP_REF=<file>  compare the dump with the reference file and fail with the first difference.
   - SLIC3R_SLICING_DUMP_THREADS=<n> number of threads of the slicing (default 1: the native slicer chains its segments in a parallel
                                     loop, so the first point of a ring varies between runs with more than one thread; 0 = no limit).
   Each dump is made twice from scratch and the two must be equal before anything is written or compared.

       $env:SLIC3R_SLICING_DUMP="<root>\build\temp\slicing-baseline\baseline.txt"
       plugins_tests "[slicing-dump]"

   Native/API parity has a separate case per mesh and mirror variant, so process
   runners can distribute them. Every case retains the full parameter matrix
   and uses one thread on both sides to keep ring start points deterministic.

       slicing parity case
       `-- check_helper_slicing()
           |-- native_slices(): reference for all parameter settings
           `-- slice_to_expolygons(): public API checked against that reference

   The [support-volumes] cases check the support column and no-support zone volume types: their ABI values and views,
   their 3MF round trip, and the copy of their kind and transform by Print::apply().
*/
#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef SLIC3R_TEST_PYTHON_PLUGINS
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
// The GDI Polygon() function of windows.h would hide Slic3r::Polygon.
#ifndef NOGDI
#define NOGDI
#endif
#include <windows.h>
#include <psapi.h>
#else
#include <dlfcn.h>
#endif
#endif

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_slicing.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

// The numeric values are part of the plugin ABI: plugins compare them without the C++ enum.
static_assert(int(Slic3r::ModelVolumeType::SUPPORT_COLUMN) == 11 && RAW_VOLUME_TYPE_SUPPORT_COLUMN == 11);
static_assert(int(Slic3r::ModelVolumeType::NO_SUPPORT_ZONE) == 12 && RAW_VOLUME_TYPE_NO_SUPPORT_ZONE == 12);

// Projects store both support-geometry volumes as non-printing helpers, in their place among the volumes.
TEST_CASE("Support columns and no-support zones keep their type, identity and transform in 3MF", "[volume][support-volumes]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    CHECK(ModelVolume::type_from_string("SupportColumn") == ModelVolumeType::SUPPORT_COLUMN);
    CHECK(ModelVolume::type_from_string("NoSupportZone") == ModelVolumeType::NO_SUPPORT_ZONE);
    CHECK(ModelVolume::type_to_string(ModelVolumeType::SUPPORT_COLUMN) == "SupportColumn");
    CHECK(ModelVolume::type_to_string(ModelVolumeType::NO_SUPPORT_ZONE) == "NoSupportZone");

    Model model;
    ModelObject *object = model.add_object("part", "part.stl", make_cube(4, 4, 1.5));
    object->add_instance()->set_offset(Vec3d(10, 12, 0));
    object->add_instance()->set_offset(Vec3d(30, 12, 0));
    ModelVolume *column = object->add_volume(make_cylinder(1, 3), ModelVolumeType::SUPPORT_COLUMN, false);
    column->name = "Pillar";
    column->set_offset(Vec3d(8, 2, 0.5));
    column->set_scaling_factor(Vec3d(1, 2, 1));
    ModelVolume *zone = object->add_volume(make_cube(1, 2, 0.6), ModelVolumeType::NO_SUPPORT_ZONE, false);
    zone->name = "Keep clear";
    zone->set_offset(Vec3d(-3, 1, 0.35));
    zone->set_rotation(Vec3d(0, 0, 0.4));

    for (const ModelVolume *helper : {column, zone}) {
        const slic3r_api::Volume view(reinterpret_cast<const volume_handle *>(helper));
        CHECK(view.is_support_geometry_modifier());
        CHECK_FALSE(view.is_support_modifier());
        CHECK_FALSE(view.is_model_part());
        CHECK_FALSE(view.is_modifier());
        CHECK(helper->is_support_volume());
        CHECK_FALSE(helper->is_support_modifier());
    }
    CHECK(slic3r_api::Volume(reinterpret_cast<const volume_handle *>(column)).is_support_column());
    CHECK(slic3r_api::Volume(reinterpret_cast<const volume_handle *>(zone)).is_no_support_zone());
    // Helpers are not printable material: the printable bounds are the cube only.
    const BoundingBoxf3 printable = object->instance_bounding_box(0);
    CHECK(printable.size().x() == Approx(4));
    CHECK(printable.size().y() == Approx(4));
    CHECK(printable.size().z() == Approx(1.5));

    const std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("support-volumes-" + std::to_string(column->id().id) + ".3mf");
    struct ArchiveOwner {
        std::filesystem::path path;
        // Remove only the archive created by this case.
        ~ArchiveOwner() { std::error_code error; std::filesystem::remove(path, error); }
    } archive{path};
    const DynamicPrintConfig saved_config;
    REQUIRE(store_3mf(path.u8string().c_str(), &model, &saved_config));
    Model loaded;
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Disable};
    REQUIRE(load_3mf(path.u8string().c_str(), config, substitutions, &loaded, false, false));
    REQUIRE(loaded.objects().size() == 1);
    const ModelObject &restored = loaded.objects().front();
    REQUIRE(restored.volumes.size() == 3);
    REQUIRE(restored.instances.size() == 2);
    CHECK(restored.volumes[0]->is_model_part());
    for (size_t idx : {size_t(1), size_t(2)}) {
        const ModelVolume &saved = *object->volumes[idx];
        const ModelVolume &read = *restored.volumes[idx];
        CAPTURE(idx);
        CHECK(read.type() == saved.type());
        CHECK(read.name == saved.name);
        const BoundingBoxf3 expected = saved.mesh().transformed_bounding_box(saved.get_matrix());
        const BoundingBoxf3 actual = read.mesh().transformed_bounding_box(read.get_matrix());
        CHECK(actual.min.isApprox(expected.min, 1e-5));
        CHECK(actual.max.isApprox(expected.max, 1e-5));
    }
    CHECK(restored.instances[0]->get_offset().isApprox(object->instances[0]->get_offset()));
    CHECK(restored.instances[1]->get_offset().isApprox(object->instances[1]->get_offset()));
}

// Conversions between demand and geometry support volumes keep the engine ID, so the applied copy must take the new kind.
TEST_CASE("Support volume kind changes and moves synchronize the applied Print", "[volume][support-volumes][replay]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    ModelObject *object = model.add_object("part", "part.stl", make_cube(4, 4, 1.5));
    object->add_instance();
    Print print;
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    print.apply(model, config);
    const ObjectID part_id = object->volumes.front()->id();
    ModelVolume *helper = object->add_volume(make_cube(1, 1, 1), ModelVolumeType::SUPPORT_COLUMN, false);
    const ObjectID helper_id = helper->id();
    for (const ModelVolumeType type : {ModelVolumeType::SUPPORT_COLUMN, ModelVolumeType::SUPPORT_BLOCKER,
             ModelVolumeType::NO_SUPPORT_ZONE, ModelVolumeType::SUPPORT_ENFORCER, ModelVolumeType::NO_SUPPORT_ZONE,
             ModelVolumeType::SUPPORT_COLUMN}) {
        CAPTURE(int(type));
        helper->set_type(type);
        helper->set_offset(helper->get_offset() + Vec3d(0.25, 0.5, 0));
        print.apply(model, config);
        REQUIRE(print.objects().size() == 1);
        const ModelObject &applied = *print.object(0).model_object();
        REQUIRE(applied.volumes.size() == 2);
        CHECK(applied.volumes.front()->id() == part_id);
        CHECK(applied.volumes.back()->id() == helper_id);
        CHECK(applied.volumes.back()->type() == type);
        CHECK(applied.volumes.back()->get_matrix().isApprox(helper->get_matrix()));
    }
    object->delete_volume(1);
    print.apply(model, config);
    REQUIRE(print.object(0).model_object()->volumes.size() == 1);
    CHECK(print.object(0).model_object()->volumes.front()->is_model_part());
}

namespace {

using namespace Slic3r;

// One object to slice for the baseline dump: the mesh, the settings to override and the transformation to apply.
struct SlicingDumpCase {
    const char *name;
    const char *mesh;
    std::vector<std::pair<std::string, std::string>> overrides;
    // 0: none, 1: mirror of the instance on X, 2: mirror of the volume on X.
    int mirror;
};

// The cases of the baseline, in recording order.
const std::vector<SlicingDumpCase> &slicing_dump_cases();
// The mesh of a case by name: a named test mesh, or an .obj of tests/data.
TriangleMesh slicing_dump_mesh(const std::string &name);
// Slices the case through the plugin pipeline (layer heights, then STEP_SLICING) and appends its raw slices to the text.
void dump_slicing_case(std::ostringstream &out, const SlicingDumpCase &current);
// Text lines of the baseline: the cases are sliced inside a TBB arena and under a global limit of the given number of threads (0: no limit).
std::string make_slicing_dump(int threads);
// Writes every ring of the expolygons as it is stored: contour, then holes, point after point.
void dump_expolygons(std::ostringstream &out, const ExPolygons &expolygons);
// A message naming the first line that differs between the reference and the current text, empty when equal.
std::string first_difference(const std::string &reference, const std::string &current);
// The content of a file; fails the test when the file cannot be opened.
std::string read_text(const std::string &path);

const std::vector<SlicingDumpCase> &slicing_dump_cases()
{
    static const std::vector<SlicingDumpCase> cases{
        {"A", "A", {}, 0},
        {"extruder_idler", "extruder_idler", {}, 0},
        {"sloping_hole", "sloping_hole", {}, 0},
        {"cube_with_concave_hole", "cube_with_concave_hole", {}, 0},
        {"two_hollow_squares", "two_hollow_squares", {}, 0},
        {"overhang", "overhang", {}, 0},
        {"A-mirror-instance", "A", {}, 1},
        {"extruder_idler-mirror-volume", "extruder_idler", {}, 2},
        {"extruder_idler-even-odd", "extruder_idler", {{"slicing_mode", "even_odd"}}, 0},
        {"extruder_idler-close-holes", "extruder_idler", {{"slicing_mode", "close_holes"}}, 0},
        {"sloping_hole-even-odd", "sloping_hole", {{"slicing_mode", "even_odd"}}, 0},
        {"sloping_hole-close-holes", "sloping_hole", {{"slicing_mode", "close_holes"}}, 0},
        {"two_hollow_squares-even-odd", "two_hollow_squares", {{"slicing_mode", "even_odd"}}, 0},
        {"two_hollow_squares-close-holes", "two_hollow_squares", {{"slicing_mode", "close_holes"}}, 0},
        {"cube_with_concave_hole-close-holes", "cube_with_concave_hole", {{"slicing_mode", "close_holes"}}, 0},
        {"extruder_idler-no-closing", "extruder_idler", {{"slice_closing_radius", "0"}}, 0},
        {"extruder_idler-big-closing", "extruder_idler", {{"slice_closing_radius", "0.3"}}, 0},
        {"extruder_idler-no-precision", "extruder_idler", {{"model_precision", "0"}}, 0},
        {"extruder_idler-coarse-precision", "extruder_idler", {{"model_precision", "0.01"}}, 0},
        {"extruder_idler-precision-small", "extruder_idler", {{"model_precision", "0.0005"}}, 0},
        {"extruder_idler-resolution", "extruder_idler", {{"resolution", "0.1"}}, 0},
        {"A-resolution", "A", {{"resolution", "0.05"}}, 0},
        // The extra offset of the slicing is max(0, min(outer, inner, inner + hole)): the three settings must be positive to give an offset.
        {"extruder_idler-xy-positive", "extruder_idler", {{"xy_size_compensation", "0.1"}, {"xy_inner_size_compensation", "0.1"}}, 0},
        {"extruder_idler-xy-negative", "extruder_idler", {{"xy_size_compensation", "-0.1"}}, 0},
        {"extruder_idler-xy-inner-small", "extruder_idler", {{"xy_size_compensation", "0.1"}, {"xy_inner_size_compensation", "0.07"}}, 0},
        {"extruder_idler-hole-negative", "extruder_idler",
         {{"xy_size_compensation", "0.1"}, {"xy_inner_size_compensation", "0.1"}, {"hole_size_compensation", "-0.05"}}, 0},
        {"two_hollow_squares-xy-positive", "two_hollow_squares",
         {{"xy_size_compensation", "0.13"}, {"xy_inner_size_compensation", "0.13"}}, 0},
        {"two_hollow_squares-xy-positive-even-odd", "two_hollow_squares",
         {{"xy_size_compensation", "0.08"}, {"xy_inner_size_compensation", "0.08"}, {"slicing_mode", "even_odd"}}, 0},
        {"sloping_hole-xy-positive-close-holes", "sloping_hole",
         {{"xy_size_compensation", "0.2"}, {"xy_inner_size_compensation", "0.2"}, {"slicing_mode", "close_holes"},
          {"slice_closing_radius", "0"}}, 0},
        {"overhang-layers", "overhang", {{"layer_height", "0.1"}, {"first_layer_height", "0.35"}}, 0},
        {"extruder_idler-all", "extruder_idler",
         {{"slicing_mode", "even_odd"}, {"xy_size_compensation", "0.06"}, {"xy_inner_size_compensation", "0.06"},
          {"slice_closing_radius", "0.1"},
          {"model_precision", "0.002"}, {"resolution", "0.03"}}, 2},
    };
    return cases;
}

TriangleMesh slicing_dump_mesh(const std::string &name)
{
    if (name == "A")
        return Test::mesh(Test::TestMesh::A);
    if (name == "sloping_hole")
        return Test::mesh(Test::TestMesh::sloping_hole);
    if (name == "cube_with_concave_hole")
        return Test::mesh(Test::TestMesh::cube_with_concave_hole);
    if (name == "two_hollow_squares")
        return Test::mesh(Test::TestMesh::two_hollow_squares);
    if (name == "overhang")
        return Test::mesh(Test::TestMesh::overhang);
    TriangleMesh mesh;
    REQUIRE(load_obj((std::string(TEST_DATA_DIR) + "/" + name + ".obj").c_str(), &mesh));
    return mesh;
}

void dump_expolygons(std::ostringstream &out, const ExPolygons &expolygons)
{
    auto ring = [&out](const char *kind, const Polygon &polygon) {
        out << " " << kind << " " << polygon.points.size();
        for (const Point &point : polygon.points)
            out << " " << (long long) point.x() << " " << (long long) point.y();
        out << "\n";
    };
    for (size_t idx = 0; idx < expolygons.size(); ++idx) {
        out << "  expolygon " << idx << " holes " << expolygons[idx].holes.size() << "\n";
        ring("contour", expolygons[idx].contour);
        for (const Polygon &hole : expolygons[idx].holes)
            ring("hole", hole);
    }
}

void dump_slicing_case(std::ostringstream &out, const SlicingDumpCase &current)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    for (const std::pair<std::string, std::string> &override : current.overrides)
        config.set_deserialize_strict({{override.first, override.second}});

    Print print;
    Model model;
    ModelObject *object = model.add_object();
    object->name += "object.stl";
    ModelVolume *volume = object->add_volume(slicing_dump_mesh(current.mesh));
    ModelInstance *instance = object->add_instance();
    if (current.mirror == 1)
        instance->set_mirror(X, -1.);
    else if (current.mirror == 2)
        volume->set_mirror(X, -1.);
    model.center_instances_around_point({100, 100});
    for (ModelObject &model_object : model.objects()) {
        model_object.ensure_on_bed();
        print.auto_assign_extruders(&model_object);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();

    Orchestrator &host = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::prepare(print);
    Steps::StepLayerHeightGeneration::run_step(host, print);
    Steps::StepSlicing::run_step(host, print);

    out << "case " << current.name << " objects " << print.objects().size() << "\n";
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx) {
        const PrintObject &print_object = print.object(object_idx);
        out << " object " << object_idx << " layers " << print_object.layer_count() << "\n";
        size_t layer_idx = 0;
        for (const Layer &layer : print_object.layers()) {
            out << " layer " << layer_idx++ << " print_z " << (long long) layer.scaled_print_z() << " regions "
                << layer.regions().size() << "\n";
            size_t region_idx = 0;
            for (const LayerRegion &region : layer.regions()) {
                out << " region " << region_idx++ << " expolygons " << region.get_raw_slices().size() << "\n";
                dump_expolygons(out, region.get_raw_slices());
            }
        }
    }
}

std::string make_slicing_dump(int threads)
{
    std::ostringstream out;
    for (const SlicingDumpCase &current : slicing_dump_cases()) {
        // The print is built, sliced, dumped and destroyed inside the arena, so that no step of the case depends on the number of cores.
        if (threads > 0) {
            tbb::global_control limit(tbb::global_control::max_allowed_parallelism, size_t(threads));
            tbb::task_arena arena(threads);
            arena.execute([&] { dump_slicing_case(out, current); });
        } else {
            dump_slicing_case(out, current);
        }
    }
    return out.str();
}

std::string first_difference(const std::string &reference, const std::string &current)
{
    if (reference == current)
        return {};
    std::istringstream ref_stream(reference), cur_stream(current);
    std::string ref_line, cur_line, last_case;
    for (size_t line = 1;; ++line) {
        const bool has_ref = bool(std::getline(ref_stream, ref_line));
        const bool has_cur = bool(std::getline(cur_stream, cur_line));
        if (has_ref && ref_line.rfind("case ", 0) == 0)
            last_case = ref_line;
        if (!has_ref && !has_cur)
            return "the texts differ only by their end";
        if (has_ref != has_cur || ref_line != cur_line)
            return "first difference at line " + std::to_string(line) + " (" + last_case + ")\n  reference: " +
                   (has_ref ? ref_line : "<end>") + "\n  current:   " + (has_cur ? cur_line : "<end>");
    }
}

std::string read_text(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

} // namespace

TEST_CASE("Slice volume defines no dimensional settings",
          "[plugins][slice-volume][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("slice_volume");
    REQUIRE(plugin != nullptr);
    // Hole compensation moved to slicing.hole_size_compensation, XY
    // compensation to slicing.xy_size_compensation.
    CHECK(plugin->get_defined_config_keys().empty());

    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        CHECK(rule.owner != "slice_volume");
}

// Slices the baseline objects and records every raw slice; checks that two runs give the same text, then writes and/or compares it.
TEST_CASE("Object slicing baseline dump", "[.][plugins][slicing-dump][slow]")
{
    const char *out_env = std::getenv("SLIC3R_SLICING_DUMP");
    const char *ref_env = std::getenv("SLIC3R_SLICING_DUMP_REF");
    const char *threads_env = std::getenv("SLIC3R_SLICING_DUMP_THREADS");
    const std::string out_path = out_env != nullptr ? out_env : "";
    const std::string ref_path = ref_env != nullptr ? ref_env : "";
    const int threads = threads_env != nullptr ? std::atoi(threads_env) : 1;
    if (out_path.empty() && ref_path.empty()) {
        SUCCEED("SLIC3R_SLICING_DUMP and SLIC3R_SLICING_DUMP_REF are not set: nothing to dump");
        return;
    }
    std::error_code ignored;
    if (!out_path.empty() && !ref_path.empty() && std::filesystem::equivalent(out_path, ref_path, ignored))
        FAIL("SLIC3R_SLICING_DUMP and SLIC3R_SLICING_DUMP_REF name the same file: the reference would be overwritten");

    // The reference first, so a missing one fails before the long run.
    const std::string reference = ref_path.empty() ? std::string() : read_text(ref_path);

    const std::string first = make_slicing_dump(threads);
    const std::string second = make_slicing_dump(threads);
    const std::string repeat = first_difference(first, second);
    if (!repeat.empty())
        std::cerr << "the two runs differ: " << repeat << "\n";
    REQUIRE(repeat.empty());

    std::string compared;
    if (!ref_path.empty()) {
        compared = first_difference(reference, first);
        if (!compared.empty())
            std::cerr << "the dump differs from the reference: " << compared << "\n";
    }
    if (!out_path.empty()) {
        std::ofstream out(out_path, std::ios::binary);
        REQUIRE(out.is_open());
        out << first;
        REQUIRE(out.good());
    }
    CHECK(compared.empty());
}

// ---- slic3r_api::TriangleMesh::slice_to_expolygons: identity with the native slicing ----

namespace {

// One set of slicing parameters of the identity test, in millimeters.
struct HelperParams {
    raw_mesh_slicing_mode mode;
    raw_mesh_slicing_mode mode_below;
    uint32_t below_layer;
    float closing_radius;
    float extra_offset;
    double model_resolution;
    double resolution;
};

// Compare all unchanged parameter settings for one mesh/mirror pair, with one
// thread on both sides and a nonempty-result check local to that pair.
void check_helper_slicing(const char *name, bool mirror);

// The parameters of the identity test: every mode, the closing / extra offset branches (both signs), the mode below a layer,
// the model resolution and the resolution, and distances smaller than one scaled unit (they truncate to 0 or to a unit).
const std::vector<HelperParams> &helper_param_cases()
{
    constexpr raw_mesh_slicing_mode RG = RAW_MESH_SLICING_MODE_REGULAR, EO = RAW_MESH_SLICING_MODE_EVEN_ODD,
                                    PO = RAW_MESH_SLICING_MODE_POSITIVE, PL = RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR;
    static const std::vector<HelperParams> cases{
        {RG, RG, 0, 0.f, 0.f, 0., 0.},
        {RG, RG, 0, 0.0499f, 0.f, 0.0001, 0.0125},
        {EO, EO, 0, 0.0499f, 0.f, 0.0001, 0.0125},
        {PO, PO, 0, 0.0499f, 0.f, 0.0001, 0.0125},
        {PL, PL, 0, 0.0499f, 0.f, 0.0001, 0.0125},
        {PL, PL, 0, 0.f, 0.f, 0., 0.},
        {RG, RG, 0, 0.3f, 0.1f, 0.0001, 0.0125},
        {RG, RG, 0, 0.05f, 0.2f, 0.0001, 0.0125},
        {RG, RG, 0, 0.f, 0.15f, 0.0001, 0.0125},
        {RG, RG, 0, 0.f, -0.05f, 0.0001, 0.0125},
        {RG, RG, 0, 0.1f, -0.04f, 0.0001, 0.},
        {RG, RG, 0, 0.0499f, -0.04f, 0.0001, 0.0125},
        {RG, EO, 4, 0.0499f, 0.f, 0.0001, 0.0125},
        {PO, PL, 5, 0.0499f, 0.f, 0.0001, 0.0125},
        {PL, RG, 6, 0.0499f, 0.f, 0.0001, 0.0125},
        {EO, PL, 3, 0.02f, 0.07f, 0.0001, 0.0125},
        {RG, RG, 0, 0.0499f, 0.f, 0.01, 0.05},
        {RG, RG, 0, 0.f, 0.f, 0.05, 0.},
        {RG, RG, 0, 0.0000019f, 0.0000013f, 0.0000017, 0.0000019},
        {EO, RG, 2, 0.0499007f, 0.0000019f, 0.0001, 0.01250009},
        {RG, RG, 0, 0.0499f, 0.f, 0.0001, 0.2},
        {EO, PL, 4, 0.2f, -0.03f, 0.002, 0.03},
    };
    return cases;
}

c_matrix4d helper_transform(bool mirror)
{
    c_matrix4d transform = matrix4d_identity();
    if (mirror) {
        // A mirror on X (negative determinant: the host flips the triangles) and a translation.
        transform.value[0] = -1.;
        transform.value[3] = 7.3;
        transform.value[7] = -2.1;
    }
    return transform;
}

c_mesh_slicing_params helper_slicing_params(const HelperParams &settings, bool mirror)
{
    c_mesh_slicing_params params = {};
    params.mode = settings.mode;
    params.mode_below = settings.mode_below;
    params.slicing_mode_normal_below_layer = settings.below_layer;
    params.transform = helper_transform(mirror);
    params.closing_radius = settings.closing_radius;
    params.extra_offset = settings.extra_offset;
    params.model_resolution = settings.model_resolution;
    params.resolution = settings.resolution;
    return params;
}

// Slice Z values spread over the height of the mesh once the transform is applied.
std::vector<float> helper_slice_zs(const TriangleMesh &mesh, const c_mesh_slicing_params &params, size_t count)
{
    const Transform3d transform = [&] {
        Transform3d out = Transform3d::Identity();
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                out.matrix()(row, col) = params.transform.value[row * 4 + col];
        return out;
    }();
    const BoundingBoxf3 box = mesh.transformed_bounding_box(transform);
    std::vector<float> zs;
    for (size_t idx = 0; idx < count; ++idx)
        zs.push_back(float(box.min.z() + (box.max.z() - box.min.z()) * (double(idx) + 0.37) / double(count)));
    return zs;
}

// The old host path: native slice_mesh_ex() on the mesh (triangles flipped for a negative determinant), then the host ensure_valid().
std::vector<ExPolygons> native_slices(const TriangleMesh &mesh, const c_mesh_slicing_params &params, const std::vector<float> &zs)
{
    MeshSlicingParamsEx native;
    native.mode = MeshSlicingParams::SlicingMode(int(params.mode));
    native.slicing_mode_normal_below_layer = params.slicing_mode_normal_below_layer;
    native.mode_below = MeshSlicingParams::SlicingMode(int(params.mode_below));
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            native.trafo.matrix()(row, col) = params.transform.value[row * 4 + col];
    native.closing_radius = params.closing_radius;
    native.extra_offset = params.extra_offset;
    native.resolution = params.resolution;
    native.model_resolution = params.model_resolution;
    indexed_triangle_set its = mesh.its;
    if (native.trafo.rotation().determinant() < 0.)
        its_flip_triangles(its);
    std::vector<ExPolygons> sliced = slice_mesh_ex(its, zs, native);
    for (ExPolygons &layer : sliced)
        ensure_valid(layer, scale_i(native.resolution));
    return sliced;
}

// The first difference (sizes, order, every point) between the native slices and the helper slices; empty when identical.
std::string compare_helper_slices(const std::vector<ExPolygons> &native,
                                  const std::vector<slic3r_api::StoredExPolygonCollection> &api)
{
    if (native.size() != api.size())
        return "layer count " + std::to_string(native.size()) + " / " + std::to_string(api.size());
    auto same_ring = [](const Polygon &expected, const slic3r_api::Polygon &actual) {
        const std::vector<c_point> points = actual.points();
        if (points.size() != expected.points.size())
            return false;
        for (size_t idx = 0; idx < points.size(); ++idx)
            if (points[idx].x != expected.points[idx].x() || points[idx].y != expected.points[idx].y())
                return false;
        return true;
    };
    for (size_t layer = 0; layer < native.size(); ++layer) {
        const std::string where = "layer " + std::to_string(layer) + ": ";
        if (native[layer].size() != api[layer].size())
            return where + "expolygon count " + std::to_string(native[layer].size()) + " / " + std::to_string(api[layer].size());
        for (uint32_t idx = 0; idx < api[layer].size(); ++idx) {
            const ExPolygon &expected = native[layer][idx];
            const slic3r_api::ExPolygon actual = api[layer].at(idx);
            if (!same_ring(expected.contour, actual.contour()))
                return where + "contour " + std::to_string(idx);
            if (expected.holes.size() != actual.hole_size())
                return where + "hole count of expolygon " + std::to_string(idx);
            for (uint32_t hole = 0; hole < actual.hole_size(); ++hole)
                if (!same_ring(expected.holes[hole], actual.hole(hole)))
                    return where + "hole " + std::to_string(hole) + " of expolygon " + std::to_string(idx);
        }
    }
    return {};
}

// A cancellation probe: counts the calls of is_cancelled and answers by the call number.
struct CancelProbe {
    std::atomic<int> calls{0};
    int cancel_after = -1; // is_cancelled is true from the call after this number (-1: never)
    int throw_after = -1;  // is_cancelled throws from the call after this number (-1: never)
};

int probe_is_cancelled(plugin_host_context *host)
{
    CancelProbe &probe = *reinterpret_cast<CancelProbe *>(host);
    const int call = ++probe.calls;
    if (probe.throw_after >= 0 && call > probe.throw_after)
        throw std::runtime_error("probe");
    return probe.cancel_after >= 0 && call > probe.cancel_after ? 1 : 0;
}

plugin_run_context probe_context(CancelProbe &probe)
{
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    context.is_cancelled = &probe_is_cancelled;
    return context;
}

// The cube-like mesh and parameters of the cancellation cases.
constexpr size_t cancel_layer_count = 12;

// is_cancelled calls of triangle_mesh_slice_to_polygons() on a small mesh: the check before the work, the one of
// slice_make_lines (face 0), the one between the lines and the loops, the one of make_loops (layer 0), and the one before
// publishing.
constexpr int raw_slice_checks = 5;

void check_helper_slicing(const char *name, bool mirror)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // One thread on both sides: the native slicer chains its segments in a parallel loop, so the first point of a ring varies otherwise.
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
    tbb::task_arena arena(1);
    size_t expolygon_total = 0, comparisons = 0;
    arena.execute([&] {
        const TriangleMesh mesh = slicing_dump_mesh(name);
        const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
        for (const HelperParams &settings : helper_param_cases()) {
            const c_mesh_slicing_params params = helper_slicing_params(settings, mirror);
            const std::vector<float> zs = helper_slice_zs(mesh, params, 14);
            const std::vector<ExPolygons> native = native_slices(mesh, params, zs);
            PluginStorage storage;
            const std::vector<slic3r_api::StoredExPolygonCollection> api =
                view.slice_to_expolygons(reinterpret_cast<storage_handle *>(&storage), params, zs, nullptr);
            const std::string difference = compare_helper_slices(native, api);
            INFO(name << (mirror ? " mirrored" : "") << ", parameters #"
                 << (&settings - helper_param_cases().data()));
            CHECK(difference == "");
            ++comparisons;
            for (const slic3r_api::StoredExPolygonCollection &layer : api)
                expolygon_total += layer.size();
        }
    });
    // The slices must not be empty, or the comparison would prove nothing.
    CHECK(expolygon_total > comparisons * 5);
}
} // namespace

// Each case runs the full parameter matrix and rejects vacuous results.
TEST_CASE("slice_to_expolygons matches native slicing (A)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("A", false);
}
// Keep the detailed idler mesh as its own scheduling unit.
TEST_CASE("slice_to_expolygons matches native slicing (extruder_idler)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("extruder_idler", false);
}
// Preserve the sloped-hole geometry across every slicing mode.
TEST_CASE("slice_to_expolygons matches native slicing (sloping_hole)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("sloping_hole", false);
}
// Preserve concave-hole coverage across every slicing mode.
TEST_CASE("slice_to_expolygons matches native slicing (cube_with_concave_hole)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("cube_with_concave_hole", false);
}
// Preserve multiple hollow components across every slicing mode.
TEST_CASE("slice_to_expolygons matches native slicing (two_hollow_squares)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("two_hollow_squares", false);
}
// Preserve the overhang mesh across every slicing mode.
TEST_CASE("slice_to_expolygons matches native slicing (overhang)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("overhang", false);
}
// Keep negative-determinant transform coverage for the letter mesh.
TEST_CASE("slice_to_expolygons matches native slicing (A mirrored)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("A", true);
}
// Keep negative-determinant transform coverage for the detailed idler mesh.
TEST_CASE("slice_to_expolygons matches native slicing (extruder_idler mirrored)", "[plugins][slice-volume][slicing-helper]")
{
    check_helper_slicing("extruder_idler", true);
}

TEST_CASE("slice_to_expolygons helper handles an empty list of layers", "[plugins][slice-volume][slicing-helper]")
{
    const TriangleMesh mesh = slicing_dump_mesh("A");
    const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
    PluginStorage storage;
    const c_mesh_slicing_params params = helper_slicing_params(helper_param_cases()[1], false);
    CHECK(view.slice_to_expolygons(reinterpret_cast<storage_handle *>(&storage), params, {}, nullptr).empty());
}

TEST_CASE("slice_to_expolygons helper stops on cancellation", "[plugins][slice-volume][slicing-helper][cancel]")
{
    const TriangleMesh mesh = slicing_dump_mesh("extruder_idler");
    const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
    const c_mesh_slicing_params params = helper_slicing_params(helper_param_cases()[1], false);
    const std::vector<float> zs = helper_slice_zs(mesh, params, cancel_layer_count);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    // A cancelled call must leave nothing behind in the storage of the caller.
    const size_t objects_before = storage.size();

    SECTION("a context that never cancels gives the slices") {
        CancelProbe probe;
        const plugin_run_context context = probe_context(probe);
        const std::vector<slic3r_api::StoredExPolygonCollection> out = view.slice_to_expolygons(handle, params, zs, &context);
        CHECK(out.size() == zs.size());
        // The checks of the raw slicing, one per layer, one at the end.
        CHECK(probe.calls.load() == raw_slice_checks + int(zs.size()) + 1);
    }
    SECTION("a context cancelled before the call") {
        CancelProbe probe;
        probe.cancel_after = 0;
        const plugin_run_context context = probe_context(probe);
        CHECK_THROWS_AS(view.slice_to_expolygons(handle, params, zs, &context), slic3r_api::PluginCancelled);
        CHECK(storage.size() == objects_before);
    }
    SECTION("a context cancelled while the layers are finished") {
        CancelProbe probe;
        probe.cancel_after = raw_slice_checks + int(zs.size()) / 2;
        const plugin_run_context context = probe_context(probe);
        CHECK_THROWS_AS(view.slice_to_expolygons(handle, params, zs, &context), slic3r_api::PluginCancelled);
        CHECK(storage.size() == objects_before);
    }
    SECTION("a context cancelled after the last layer") {
        CancelProbe probe;
        probe.cancel_after = raw_slice_checks + int(zs.size());
        const plugin_run_context context = probe_context(probe);
        CHECK_THROWS_AS(view.slice_to_expolygons(handle, params, zs, &context), slic3r_api::PluginCancelled);
        CHECK(storage.size() == objects_before);
    }
    SECTION("an exception of the first layer job is rethrown after the loop, not through the host frames") {
        CancelProbe probe;
        probe.throw_after = raw_slice_checks;
        const plugin_run_context context = probe_context(probe);
        // The exception of the probe itself, not the message of the host for an internal error.
        CHECK_THROWS_WITH(view.slice_to_expolygons(handle, params, zs, &context), "probe");
        CHECK(storage.size() == objects_before);
    }
    SECTION("a call inside a parallel group that is already cancelled") {
        tbb::task_group_context group;
        bool cancelled = false;
        tbb::parallel_for(0, 1, [&](int) {
            group.cancel_group_execution();
            try {
                view.slice_to_expolygons(handle, params, zs, nullptr);
            } catch (const slic3r_api::PluginCancelled &) {
                cancelled = true;
            }
        }, group);
        CHECK(cancelled);
        CHECK(storage.size() == objects_before);
    }
}

// ---- triangle_mesh_slice_to_polygons: status, cancellation and invalid arguments ----

namespace {

// One mesh sliced into 12 collections. fill() slices once so that the later failing calls have outputs to clear.
struct RawSliceFixture {
    TriangleMesh mesh;
    c_mesh_slicing_params params;
    std::vector<float> zs;
    PluginStorage storage;
    std::vector<slic3r_api::StoredPolygonCollection> out;
    std::vector<polygon_collection_handle *> handles;

    RawSliceFixture() :
        mesh(slicing_dump_mesh("extruder_idler")),
        params(helper_slicing_params(helper_param_cases()[1], false)),
        zs(helper_slice_zs(mesh, params, cancel_layer_count))
    {
        for (size_t idx = 0; idx < zs.size(); ++idx) {
            out.emplace_back(reinterpret_cast<storage_handle *>(&storage));
            handles.push_back(out.back().mutable_handle());
        }
    }

    const triangle_mesh_handle *mesh_handle() const { return reinterpret_cast<const triangle_mesh_handle *>(&mesh); }

    int32_t call(const plugin_run_context *context)
    {
        return triangle_mesh_slice_to_polygons(mesh_handle(), &params, zs.data(), handles.data(), uint32_t(zs.size()), context);
    }

    // Make every collection non-empty.
    void fill()
    {
        REQUIRE(call(nullptr) == RAW_MESH_SLICE_STATUS_DONE);
        REQUIRE_FALSE(all_empty());
        for (const slic3r_api::StoredPolygonCollection &layer : out)
            REQUIRE_FALSE(layer.empty());
    }

    bool all_empty() const
    {
        return std::all_of(out.begin(), out.end(), [](const slic3r_api::StoredPolygonCollection &layer) { return layer.empty(); });
    }
};

} // namespace

TEST_CASE("triangle_mesh_slice_to_polygons reports done, cancelled and internal error",
          "[plugins][slice-volume][slicing-helper][cancel]")
{
    RawSliceFixture fixture;
    const size_t objects_before = fixture.storage.size();

    SECTION("a context that never cancels, a NULL context and a NULL callback give the slices") {
        CancelProbe probe;
        const plugin_run_context context = probe_context(probe);
        CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_DONE);
        CHECK(probe.calls.load() == raw_slice_checks);
        CHECK_FALSE(fixture.all_empty());
        CHECK(fixture.call(nullptr) == RAW_MESH_SLICE_STATUS_DONE);
        const plugin_run_context no_callback = {};
        CHECK(fixture.call(&no_callback) == RAW_MESH_SLICE_STATUS_DONE);
        CHECK_FALSE(fixture.all_empty());
    }
    SECTION("a context cancelled before the call is checked before any work") {
        fixture.fill();
        CancelProbe probe;
        probe.cancel_after = 0;
        const plugin_run_context context = probe_context(probe);
        CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_CANCELLED);
        CHECK(probe.calls.load() == 1);
        CHECK(fixture.all_empty());
    }
    SECTION("a context cancelled by the first check of the slicer") {
        fixture.fill();
        CancelProbe probe;
        probe.cancel_after = 1;
        const plugin_run_context context = probe_context(probe);
        CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_CANCELLED);
        // The sentinel stops the work: no check after the one that answered yes.
        CHECK(probe.calls.load() == probe.cancel_after + 1);
        CHECK(fixture.all_empty());
    }
    SECTION("a context cancelled between the lines and the loops, then at the first loop") {
        for (const int cancel_after : {2, 3}) {
            fixture.fill();
            CancelProbe probe;
            probe.cancel_after = cancel_after;
            const plugin_run_context context = probe_context(probe);
            CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_CANCELLED);
            CHECK(probe.calls.load() == cancel_after + 1);
            CHECK(fixture.all_empty());
        }
    }
    SECTION("a cancellation after the last check of the slicer is seen before publishing") {
        fixture.fill();
        CancelProbe probe;
        probe.cancel_after = raw_slice_checks - 1;
        const plugin_run_context context = probe_context(probe);
        CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_CANCELLED);
        // The slicer finished normally: only the check before publishing saw the request.
        CHECK(probe.calls.load() == raw_slice_checks);
        CHECK(fixture.all_empty());
    }
    SECTION("a call inside a parallel group that is already cancelled gives cancelled, also without a context") {
        fixture.fill();
        tbb::task_group_context group;
        std::atomic<int32_t> status{RAW_MESH_SLICE_STATUS_DONE};
        tbb::parallel_for(0, 1, [&](int) {
            group.cancel_group_execution();
            status = fixture.call(nullptr);
        }, group);
        CHECK(status.load() == RAW_MESH_SLICE_STATUS_CANCELLED);
        CHECK(fixture.all_empty());
    }
    SECTION("an exception of the callback becomes an internal error") {
        fixture.fill();
        CancelProbe probe;
        probe.throw_after = 1;
        const plugin_run_context context = probe_context(probe);
        CHECK(fixture.call(&context) == RAW_MESH_SLICE_STATUS_INTERNAL_ERROR);
        CHECK(probe.calls.load() == probe.throw_after + 1);
        CHECK(fixture.all_empty());
    }
    CHECK(fixture.storage.size() == objects_before);
}

// A closed double-sided wall of zero thickness: its slice is a loop of two coincident segments, a loop without area.
// PositiveLargestContour has no area to keep, so the layer is empty (the native slicer used to dereference a null pointer).
TEST_CASE("triangle_mesh_slice_to_polygons gives an empty layer when PositiveLargestContour finds only zero-area loops",
          "[plugins][slice-volume][slicing-helper][triangle-mesh]")
{
    indexed_triangle_set its;
    its.vertices = {Vec3f(0.f, 0.f, 0.f), Vec3f(10.f, 0.f, 0.f), Vec3f(10.f, 0.f, 10.f), Vec3f(0.f, 0.f, 10.f)};
    // The two triangles of the wall, then the same two with the opposite orientation.
    its.indices = {Vec3i32(0, 1, 2), Vec3i32(0, 2, 3), Vec3i32(0, 2, 1), Vec3i32(0, 3, 2)};
    const TriangleMesh mesh(std::move(its));
    const triangle_mesh_handle *mesh_handle = reinterpret_cast<const triangle_mesh_handle *>(&mesh);
    const std::vector<float> zs{5.f};

    PluginStorage storage;
    slic3r_api::StoredPolygonCollection layer(reinterpret_cast<storage_handle *>(&storage));
    polygon_collection_handle *handles[1] = {layer.mutable_handle()};
    c_mesh_slicing_params params = {};
    params.transform = helper_transform(false);
    // Control: the other modes keep the zero-area loops, so the slicer does produce them.
    params.mode = RAW_MESH_SLICING_MODE_POSITIVE;
    params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE;
    REQUIRE(triangle_mesh_slice_to_polygons(mesh_handle, &params, zs.data(), handles, 1, nullptr) == RAW_MESH_SLICE_STATUS_DONE);
    REQUIRE_FALSE(layer.empty());
    // Fixture precondition: every loop of the wall has a zero area.
    for (uint32_t idx = 0; idx < layer.size(); ++idx)
        REQUIRE(layer.at(idx).area() == 0.);

    params.mode = RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR;
    params.mode_below = RAW_MESH_SLICING_MODE_POSITIVE_LARGEST_CONTOUR;

    REQUIRE(triangle_mesh_slice_to_polygons(mesh_handle, &params, zs.data(), handles, 1, nullptr) == RAW_MESH_SLICE_STATUS_DONE);
    CHECK(layer.empty());
}

// keep_largest_contour_only() keeps the contour of largest positive area, and leaves the islands untouched when there
// is none (all contours of zero area or clockwise): the native code used to dereference a null pointer there.
TEST_CASE("keep_largest_contour_only keeps the largest positive contour and tolerates a layer without one",
          "[plugins][slice-volume][slicing-helper]")
{
    // Coordinates are in scaled units: closer points would be merged by the Polygon constructor.
    const Polygon square(Points{Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))});
    const Polygon large(Points{Point(scale_i(20.), 0), Point(scale_i(50.), 0), Point(scale_i(50.), scale_i(30.)),
                               Point(scale_i(20.), scale_i(30.))});
    ExPolygons expolys{ExPolygon(square), ExPolygon(large)};
    keep_largest_contour_only(expolys);
    REQUIRE(expolys.size() == 1);
    CHECK(expolys.front().contour == large);

    // A zero-area (collinear) contour and a clockwise contour: there is no largest positive contour to keep.
    const Polygon collinear(Points{Point(0, 0), Point(scale_i(5.), 0), Point(scale_i(10.), 0)});
    const Polygon clockwise_square(Points{Point(0, 0), Point(0, scale_i(10.)), Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), 0)});
    const ExPolygons original{ExPolygon(collinear), ExPolygon(clockwise_square)};
    REQUIRE(original[0].contour.area() == 0.);
    REQUIRE(original[1].contour.area() < 0.);
    REQUIRE(original[1].contour.points.size() == 4);
    ExPolygons no_positive = original;
    keep_largest_contour_only(no_positive);
    CHECK(no_positive == original);

    ExPolygons zero_area{original[0], original[0]};
    keep_largest_contour_only(zero_area);
    CHECK(zero_area == ExPolygons{original[0], original[0]});
}

TEST_CASE("triangle_mesh_slice_to_polygons rejects invalid arguments and clears the outputs",
          "[plugins][slice-volume][slicing-helper][cancel]")
{
    RawSliceFixture fixture;
    const uint32_t count = uint32_t(fixture.zs.size());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    SECTION("every invalid argument gives -1 and empties the outputs") {
        c_mesh_slicing_params negative_radius = fixture.params;
        negative_radius.closing_radius = -0.1f;
        c_mesh_slicing_params non_finite_transform = fixture.params;
        non_finite_transform.transform.value[3] = std::numeric_limits<double>::quiet_NaN();
        const float quiet_nan = std::numeric_limits<float>::quiet_NaN();
        const double quiet_nan_double = std::numeric_limits<double>::quiet_NaN();
        const double infinity_double = std::numeric_limits<double>::infinity();
        c_mesh_slicing_params nan_radius = fixture.params;
        nan_radius.closing_radius = quiet_nan;
        c_mesh_slicing_params infinite_radius = fixture.params;
        infinite_radius.closing_radius = infinity;
        c_mesh_slicing_params nan_extra_offset = fixture.params;
        nan_extra_offset.extra_offset = quiet_nan;
        c_mesh_slicing_params infinite_extra_offset = fixture.params;
        infinite_extra_offset.extra_offset = -infinity;
        c_mesh_slicing_params nan_resolution = fixture.params;
        nan_resolution.resolution = quiet_nan_double;
        c_mesh_slicing_params infinite_resolution = fixture.params;
        infinite_resolution.resolution = infinity_double;
        c_mesh_slicing_params nan_model_resolution = fixture.params;
        nan_model_resolution.model_resolution = quiet_nan_double;
        c_mesh_slicing_params infinite_model_resolution = fixture.params;
        infinite_model_resolution.model_resolution = infinity_double;
        std::vector<float> not_finite = fixture.zs;
        not_finite[2] = nan;
        std::vector<float> infinite = fixture.zs;
        infinite.back() = infinity;
        std::vector<float> decreasing = fixture.zs;
        std::swap(decreasing[3], decreasing[4]);
        REQUIRE(decreasing[3] > decreasing[4]);

        const std::vector<std::pair<std::string, std::function<int32_t()>>> cases{
            {"mesh NULL", [&] { return triangle_mesh_slice_to_polygons(nullptr, &fixture.params, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"params NULL", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), nullptr, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"z NULL", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, nullptr, fixture.handles.data(), count, nullptr); }},
            {"slices NULL", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, fixture.zs.data(), nullptr, count, nullptr); }},
            {"Z NaN", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, not_finite.data(), fixture.handles.data(), count, nullptr); }},
            {"Z infinite", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, infinite.data(), fixture.handles.data(), count, nullptr); }},
            {"Z decreasing", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, decreasing.data(), fixture.handles.data(), count, nullptr); }},
            {"negative closing radius", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &negative_radius, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"NaN closing radius", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &nan_radius, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"infinite closing radius", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &infinite_radius, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"NaN extra offset", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &nan_extra_offset, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"infinite extra offset", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &infinite_extra_offset, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"NaN resolution", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &nan_resolution, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"infinite resolution", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &infinite_resolution, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"NaN model resolution", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &nan_model_resolution, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"infinite model resolution", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &infinite_model_resolution, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
            {"non-finite transform", [&] { return triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &non_finite_transform, fixture.zs.data(), fixture.handles.data(), count, nullptr); }},
        };
        for (const std::pair<std::string, std::function<int32_t()>> &invalid : cases) {
            INFO(invalid.first);
            fixture.fill();
            CHECK(invalid.second() == RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT);
            // A NULL slices array has no output to clear, the other cases clear them all.
            if (invalid.first != "slices NULL")
                CHECK(fixture.all_empty());
        }
    }
    SECTION("an invalid argument wins over a cancelled context") {
        CancelProbe probe;
        probe.cancel_after = 0;
        const plugin_run_context context = probe_context(probe);
        CHECK(triangle_mesh_slice_to_polygons(nullptr, &fixture.params, fixture.zs.data(), fixture.handles.data(), count, &context) ==
              RAW_MESH_SLICE_STATUS_INVALID_ARGUMENT);
        CHECK(probe.calls.load() == 0);
    }
    SECTION("no layer is valid with NULL arrays, and a NULL entry of the outputs is ignored") {
        CHECK(triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, nullptr, nullptr, 0, nullptr) ==
              RAW_MESH_SLICE_STATUS_DONE);
        CancelProbe probe;
        probe.cancel_after = 0;
        const plugin_run_context context = probe_context(probe);
        CHECK(triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, nullptr, nullptr, 0, &context) ==
              RAW_MESH_SLICE_STATUS_CANCELLED);

        // A cancelled enclosing parallel group is reported even when there is nothing to slice.
        tbb::task_group_context group;
        std::atomic<int32_t> group_status{RAW_MESH_SLICE_STATUS_DONE};
        tbb::parallel_for(0, 1, [&](int) {
            group.cancel_group_execution();
            group_status = triangle_mesh_slice_to_polygons(fixture.mesh_handle(), &fixture.params, nullptr, nullptr, 0, nullptr);
        }, group);
        CHECK(group_status.load() == RAW_MESH_SLICE_STATUS_CANCELLED);

        fixture.handles[1] = nullptr;
        CHECK(fixture.call(nullptr) == RAW_MESH_SLICE_STATUS_DONE);
        CHECK(fixture.out[0].size() > 0);
        CHECK(fixture.out[1].empty());
        CHECK(fixture.out[2].size() > 0);
    }
}

TEST_CASE("The slice_to_polygons wrapper translates the status into exceptions",
          "[plugins][slice-volume][slicing-helper][cancel]")
{
    RawSliceFixture fixture;
    const slic3r_api::TriangleMesh view(fixture.mesh_handle());
    storage_handle *handle = reinterpret_cast<storage_handle *>(&fixture.storage);
    // The collections of the fixture are not used by the wrapper: it allocates its own and frees them on failure.
    const size_t objects_before = fixture.storage.size();

    CHECK(view.slice_to_polygons(handle, fixture.params, fixture.zs, nullptr).size() == fixture.zs.size());
    CHECK(fixture.storage.size() == objects_before);

    CancelProbe cancelled;
    cancelled.cancel_after = 0;
    const plugin_run_context cancelled_context = probe_context(cancelled);
    CHECK_THROWS_AS(view.slice_to_polygons(handle, fixture.params, fixture.zs, &cancelled_context), slic3r_api::PluginCancelled);

    // No layer: the cancellation is still reported, by the context and by the enclosing group.
    CHECK(view.slice_to_polygons(handle, fixture.params, {}, nullptr).empty());
    CHECK_THROWS_AS(view.slice_to_polygons(handle, fixture.params, {}, &cancelled_context), slic3r_api::PluginCancelled);
    CHECK_THROWS_AS(view.slice_to_expolygons(handle, fixture.params, {}, &cancelled_context), slic3r_api::PluginCancelled);
    bool empty_group_cancelled = false;
    tbb::task_group_context empty_group;
    tbb::parallel_for(0, 1, [&](int) {
        empty_group.cancel_group_execution();
        try {
            view.slice_to_polygons(handle, fixture.params, {}, nullptr);
        } catch (const slic3r_api::PluginCancelled &) {
            empty_group_cancelled = true;
        }
    }, empty_group);
    CHECK(empty_group_cancelled);

    CancelProbe failing;
    failing.throw_after = 1;
    const plugin_run_context failing_context = probe_context(failing);
    CHECK_THROWS_AS(view.slice_to_polygons(handle, fixture.params, fixture.zs, &failing_context), std::runtime_error);

    c_mesh_slicing_params negative_radius = fixture.params;
    negative_radius.closing_radius = -0.1f;
    CHECK_THROWS_AS(view.slice_to_polygons(handle, negative_radius, fixture.zs, nullptr), std::invalid_argument);
    CHECK_THROWS_AS(view.slice_to_polygons(handle, fixture.params, {1.f, 0.f}, nullptr), std::invalid_argument);

    tbb::task_group_context group;
    bool group_cancelled = false;
    tbb::parallel_for(0, 1, [&](int) {
        group.cancel_group_execution();
        try {
            view.slice_to_polygons(handle, fixture.params, fixture.zs, nullptr);
        } catch (const slic3r_api::PluginCancelled &) {
            group_cancelled = true;
        }
    }, group);
    CHECK(group_cancelled);

    // Every failure released the collections it allocated.
    CHECK(fixture.storage.size() == objects_before);
}
// ---- slice_to_expolygons helper without the one-thread arena ----

namespace {

// The helper result as native ExPolygons, to measure it with the native area and boolean functions.
ExPolygons helper_layer_as_native(const slic3r_api::StoredExPolygonCollection &layer)
{
    auto ring = [](const slic3r_api::Polygon &source) {
        Polygon out;
        for (const c_point &point : source.points())
            out.points.emplace_back(point.x, point.y);
        return out;
    };
    ExPolygons out;
    for (uint32_t idx = 0; idx < layer.size(); ++idx) {
        const slic3r_api::ExPolygon source = layer.at(idx);
        ExPolygon expolygon;
        expolygon.contour = ring(source.contour());
        for (uint32_t hole = 0; hole < source.hole_size(); ++hole)
            expolygon.holes.push_back(ring(source.hole(hole)));
        out.push_back(std::move(expolygon));
    }
    return out;
}

size_t hole_count(const ExPolygons &expolygons)
{
    size_t count = 0;
    for (const ExPolygon &expolygon : expolygons)
        count += expolygon.holes.size();
    return count;
}

} // namespace

// With several threads the first point of a ring varies, so the helper and the native slicing are compared by region:
// same number of expolygons and holes per layer, same total area, and a symmetric difference that is (almost) empty.
TEST_CASE("slice_to_expolygons helper matches the native slicing with several threads",
          "[plugins][slice-volume][slicing-helper][multithread]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double unit = scale_d(1.);
    // The simplification can keep a point one scaled unit away when its start point changes: a sliver of this area is accepted.
    const double max_sym_diff = 1e-3 * unit * unit;
    double worst = 0.;
    for (const char *mesh_name : {"extruder_idler", "sloping_hole"}) {
        const TriangleMesh mesh = slicing_dump_mesh(mesh_name);
        const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
        for (const size_t settings_idx : {size_t(1), size_t(2), size_t(6), size_t(13), size_t(21)}) {
            const c_mesh_slicing_params params = helper_slicing_params(helper_param_cases()[settings_idx], false);
            const std::vector<float> zs = helper_slice_zs(mesh, params, 14);
            const std::vector<ExPolygons> native = native_slices(mesh, params, zs);
            PluginStorage storage;
            const std::vector<slic3r_api::StoredExPolygonCollection> api =
                view.slice_to_expolygons(reinterpret_cast<storage_handle *>(&storage), params, zs, nullptr);
            REQUIRE(api.size() == native.size());
            for (size_t layer = 0; layer < native.size(); ++layer) {
                INFO(mesh_name << ", parameters #" << settings_idx << ", layer " << layer);
                const ExPolygons helper = helper_layer_as_native(api[layer]);
                CHECK(helper.size() == native[layer].size());
                CHECK(hole_count(helper) == hole_count(native[layer]));
                double helper_area = 0., native_area = 0.;
                for (const ExPolygon &expolygon : helper)
                    helper_area += expolygon.area();
                for (const ExPolygon &expolygon : native[layer])
                    native_area += expolygon.area();
                const double sym_diff = area(diff_ex(helper, native[layer])) + area(diff_ex(native[layer], helper));
                worst = std::max(worst, sym_diff);
                CHECK(sym_diff <= max_sym_diff);
                CHECK(std::abs(helper_area - native_area) <= max_sym_diff);
            }
        }
    }
    std::cerr << "largest symmetric difference with several threads: " << worst / (unit * unit) << " mm2\n";
}

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

// ---- Python helper on the real host ----
//
// The plugin tests export the host API from their executable and embed the Python interpreter of the Python plugin loader
// (see host_library_path() in PythonPluginLoader.cpp). The test finds the interpreter entry points in the process, writes a
// script and runs it there: the script reuses the Slic3rAPI object the loader created, so Python calls the very same host
// functions as the C++ code, on storage and meshes made here.

namespace {

struct PythonEntryPoints {
    int (*run_simple_string)(const char *) = nullptr;
    int (*gil_ensure)() = nullptr;
    void (*gil_release)(int) = nullptr;
};

// Look for the interpreter functions among the modules of the process; all null when no Python runtime is loaded.
PythonEntryPoints find_python_entry_points()
{
    PythonEntryPoints points;
#ifdef _WIN32
    std::vector<HMODULE> modules(1024);
    DWORD needed = 0;
    if (!K32EnumProcessModules(GetCurrentProcess(), modules.data(), DWORD(modules.size() * sizeof(HMODULE)), &needed))
        return points;
    modules.resize(std::min<size_t>(modules.size(), needed / sizeof(HMODULE)));
    for (HMODULE module : modules) {
        FARPROC run = GetProcAddress(module, "PyRun_SimpleString");
        if (run == nullptr)
            continue;
        points.run_simple_string = reinterpret_cast<int (*)(const char *)>(run);
        points.gil_ensure = reinterpret_cast<int (*)()>(GetProcAddress(module, "PyGILState_Ensure"));
        points.gil_release = reinterpret_cast<void (*)(int)>(GetProcAddress(module, "PyGILState_Release"));
        break;
    }
#else
    points.run_simple_string = reinterpret_cast<int (*)(const char *)>(dlsym(RTLD_DEFAULT, "PyRun_SimpleString"));
    points.gil_ensure = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "PyGILState_Ensure"));
    points.gil_release = reinterpret_cast<void (*)(int)>(dlsym(RTLD_DEFAULT, "PyGILState_Release"));
#endif
    if (points.gil_ensure == nullptr || points.gil_release == nullptr)
        points.run_simple_string = nullptr;
    return points;
}

// Deletes the files of one Python run when it goes out of scope.
struct TemporaryFiles {
    std::vector<std::filesystem::path> paths;
    ~TemporaryFiles()
    {
        std::error_code ignored;
        for (const std::filesystem::path &path : paths)
            std::filesystem::remove(path, ignored);
    }
    std::filesystem::path make(const char *suffix)
    {
        static int counter = 0;
        paths.push_back(std::filesystem::temp_directory_path() /
                        ("slic3r_py_real_host_" + std::to_string(counter++) + "_" + suffix));
        return paths.back();
    }
};

// Run the script file in the embedded interpreter; true when it ended without an exception (the traceback is printed otherwise).
bool run_python_script(const PythonEntryPoints &python, const std::filesystem::path &script)
{
    const std::string command = "exec(compile(open(r\'" + script.string() + "\', encoding=\'utf-8\').read(), r\'" +
                                script.string() + "\', \'exec\'), {})";
    const int gil = python.gil_ensure();
    const int status = python.run_simple_string(command.c_str());
    python.gil_release(gil);
    return status == 0;
}

void write_text(const std::filesystem::path &path, const std::string &text)
{
    std::ofstream out(path, std::ios::binary);
    REQUIRE(out.is_open());
    out << text;
    REQUIRE(out.good());
}

// The slices in the text format the Python script writes: layer line, then per expolygon its contour and holes, point by point.
std::string slices_text(const std::vector<slic3r_api::StoredExPolygonCollection> &layers)
{
    std::ostringstream out;
    auto ring = [&out](const char *kind, const slic3r_api::Polygon &polygon) {
        out << kind;
        for (const c_point &point : polygon.points())
            out << " " << (long long) point.x << " " << (long long) point.y;
        out << "\n";
    };
    for (size_t layer = 0; layer < layers.size(); ++layer) {
        out << "layer " << layer << " " << layers[layer].size() << "\n";
        for (uint32_t idx = 0; idx < layers[layer].size(); ++idx) {
            const slic3r_api::ExPolygon expolygon = layers[layer].at(idx);
            out << "ex " << idx << " " << expolygon.hole_size() << "\n";
            ring("c", expolygon.contour());
            for (uint32_t hole = 0; hole < expolygon.hole_size(); ++hole)
                ring("h", expolygon.hole(hole));
        }
    }
    return out.str();
}

// The Python script that slices a mesh with the Python helper and writes the result in the text format of slices_text().
const char *const python_slice_script = R"PY(
import slic3r_api
import slic3r_api_generated as generated
import slic3r_datatree_views as views

api = slic3r_api._registered_apis[0]
mesh = views.TriangleMesh(api, PARAMS["mesh"])
params = generated.CMeshSlicingParams()
params.mode = PARAMS["mode"]
params.mode_below = PARAMS["mode_below"]
params.slicing_mode_normal_below_layer = PARAMS["below_layer"]
for index, value in enumerate(PARAMS["transform"]):
    params.transform.value[index] = value
params.closing_radius = PARAMS["closing_radius"]
params.extra_offset = PARAMS["extra_offset"]
params.model_resolution = PARAMS["model_resolution"]
params.resolution = PARAMS["resolution"]

layers = mesh.slice_to_expolygons(PARAMS["storage"], params.transform, PARAMS["zs"], params)
lines = []
for layer_index, layer in enumerate(layers):
    lines.append("layer %d %d" % (layer_index, layer.size()))
    for index in range(layer.size()):
        expolygon = layer.at(index)
        lines.append("ex %d %d" % (index, expolygon.hole_size()))
        lines.append("c" + "".join(" %d %d" % (point.x, point.y) for point in expolygon.contour().points()))
        for hole in range(expolygon.hole_size()):
            lines.append("h" + "".join(" %d %d" % (point.x, point.y) for point in expolygon.hole(hole).points()))
with open(PARAMS["output"], "w", encoding="utf-8", newline="\n") as stream:
    stream.write("\n".join(lines) + "\n")
for layer in layers:
    layer.free_from_storage()
)PY";

// The Python script that checks the Python collinear point removal against the outputs of the native function in a file.
const char *const python_collinear_script = R"PY(
import slic3r_datatree_views as views

def read_ring(text):
    numbers = [int(item) for item in text.split()]
    return list(zip(numbers[0::2], numbers[1::2]))

checked = 0
mismatches = []
with open(PARAMS["input"], encoding="utf-8") as stream:
    for line in stream:
        offset, ring, expected = line.rstrip("\n").split("|")
        kept = views._remove_collinear_points(read_ring(ring), int(offset))
        checked += 1
        if kept != read_ring(expected):
            mismatches.append((offset, ring, expected, kept))
assert checked > 0
assert not mismatches, "%d of %d rings differ, first: %r" % (len(mismatches), checked, mismatches[:1])
)PY";

// The Python script that checks the status of the raw slicing of the host: success, a cancelled context and an invalid argument.
const char *const python_status_script = R"PY(
import slic3r_api
import slic3r_api_generated as generated
import slic3r_datatree_views as views
from slic3r_run_context import PluginCancelled

api = slic3r_api._registered_apis[0]
mesh = views.TriangleMesh(api, PARAMS["mesh"])
params = generated.CMeshSlicingParams()
for index in range(0, 16, 5):
    params.transform.value[index] = 1.0
zs = PARAMS["zs"]
storage = PARAMS["storage"]
slicers = (mesh.slice_to_polygons, mesh.slice_to_expolygons)

for slicer in slicers:
    layers = slicer(storage, params.transform, zs, params, PARAMS["context"])
    assert len(layers) == len(zs)
    assert any(layer.size() > 0 for layer in layers)
    for layer in layers:
        layer.free_from_storage()

for slicer in slicers:
    try:
        slicer(storage, params.transform, zs, params, PARAMS["cancelled_context"])
    except PluginCancelled:
        pass
    else:
        raise AssertionError("a cancelled context must raise PluginCancelled")

params.closing_radius = -1.0
for slicer in slicers:
    try:
        slicer(storage, params.transform, zs, params, PARAMS["context"])
    except ValueError:
        pass
    else:
        raise AssertionError("an invalid argument must raise ValueError")
)PY";
} // namespace

TEST_CASE("The Python slice_to_expolygons is identical to the C++ helper on the real host",
          "[plugins][slice-volume][slicing-helper][python-real-host]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PythonEntryPoints python = find_python_entry_points();
    // The gate must not pass silently without the embedded interpreter.
    REQUIRE(Test::Plugins::python_plugin_test_runtime_available());
    REQUIRE(python.run_simple_string != nullptr);
    struct Case {
        const char *mesh;
        bool mirror;
        size_t settings;
    };
    // EvenOdd with a closing radius; a mode below a layer with a positive-largest-contour; every distance together with a negative
    // extra offset, a model resolution and a resolution; a negative extra offset alone.
    const std::vector<Case> cases{{"sloping_hole", false, 2}, {"sloping_hole", false, 13}, {"sloping_hole", false, 21},
                                  {"sloping_hole", false, 9}, {"two_hollow_squares", false, 14},
                                  {"extruder_idler", true, 21}, {"extruder_idler", false, 12}};
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
    tbb::task_arena arena(1);
    arena.execute([&] {
        for (const Case &current : cases) {
            const TriangleMesh mesh = slicing_dump_mesh(current.mesh);
            const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
            const HelperParams &settings = helper_param_cases()[current.settings];
            const c_mesh_slicing_params params = helper_slicing_params(settings, current.mirror);
            const std::vector<float> zs = helper_slice_zs(mesh, params, 10);
            PluginStorage storage;
            storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);

            const std::vector<slic3r_api::StoredExPolygonCollection> expected = view.slice_to_expolygons(handle, params, zs, nullptr);

            TemporaryFiles files;
            const std::filesystem::path script = files.make("slice.py");
            const std::filesystem::path output = files.make("slice.txt");
            std::ostringstream text;
            text.precision(17);
            text << "PARAMS = {\"mesh\": " << reinterpret_cast<uintptr_t>(&mesh) << ", \"storage\": " << reinterpret_cast<uintptr_t>(handle)
                 << ", \"mode\": " << int(params.mode) << ", \"mode_below\": " << int(params.mode_below)
                 << ", \"below_layer\": " << params.slicing_mode_normal_below_layer << ", \"transform\": [";
            for (int idx = 0; idx < 16; ++idx)
                text << (idx ? ", " : "") << params.transform.value[idx];
            text << "], \"closing_radius\": " << double(params.closing_radius) << ", \"extra_offset\": " << double(params.extra_offset)
                 << ", \"model_resolution\": " << params.model_resolution << ", \"resolution\": " << params.resolution
                 << ", \"zs\": [";
            for (size_t idx = 0; idx < zs.size(); ++idx)
                text << (idx ? ", " : "") << double(zs[idx]);
            text << "], \"output\": r\'" << output.string() << "\'}\n" << python_slice_script;
            write_text(script, text.str());

            INFO(current.mesh << (current.mirror ? " mirrored" : "") << ", parameters #" << current.settings);
            const size_t objects_before = storage.size();
            REQUIRE(run_python_script(python, script));
            // The Python run frees what it made in the storage.
            CHECK(storage.size() == objects_before);
            const std::string actual = read_text(output.string());
            const std::string wanted = slices_text(expected);
            CHECK(!wanted.empty());
            CHECK(first_difference(wanted, actual) == "");
        }
    });
}

TEST_CASE("The Python slicing maps the status of the real host to its exceptions",
          "[plugins][slice-volume][slicing-helper][python-real-host][cancel]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PythonEntryPoints python = find_python_entry_points();
    // The gate must not pass silently without the embedded interpreter.
    REQUIRE(Test::Plugins::python_plugin_test_runtime_available());
    REQUIRE(python.run_simple_string != nullptr);
    const TriangleMesh mesh = slicing_dump_mesh("extruder_idler");
    const c_mesh_slicing_params params = helper_slicing_params(helper_param_cases()[1], false);
    const std::vector<float> zs = helper_slice_zs(mesh, params, 10);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    CancelProbe never;
    const plugin_run_context context = probe_context(never);
    CancelProbe always;
    always.cancel_after = 0;
    const plugin_run_context cancelled_context = probe_context(always);

    std::ostringstream text;
    text.precision(17);
    text << "PARAMS = {\"mesh\": " << reinterpret_cast<uintptr_t>(&mesh) << ", \"storage\": " << reinterpret_cast<uintptr_t>(handle)
         << ", \"context\": " << reinterpret_cast<uintptr_t>(&context)
         << ", \"cancelled_context\": " << reinterpret_cast<uintptr_t>(&cancelled_context) << ", \"zs\": [";
    for (size_t idx = 0; idx < zs.size(); ++idx)
        text << (idx ? ", " : "") << double(zs[idx]);
    text << "]}\n" << python_status_script;
    TemporaryFiles files;
    const std::filesystem::path script = files.make("status.py");
    write_text(script, text.str());

    const size_t objects_before = storage.size();
    CHECK(run_python_script(python, script));
    // The Python run freed what it allocated, also on its failing calls.
    CHECK(storage.size() == objects_before);
    CHECK(never.calls.load() > 0);
    CHECK(always.calls.load() > 0);
}
TEST_CASE("The Python remove_collinear gives the points of the native Polygon::remove_collinear",
          "[plugins][slice-volume][remove-collinear][python-real-host]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PythonEntryPoints python = find_python_entry_points();
    // The gate must not pass silently without the embedded interpreter.
    REQUIRE(Test::Plugins::python_plugin_test_runtime_available());
    REQUIRE(python.run_simple_string != nullptr);
    // Sol case first, then rings that are random, near aligned, duplicated and listed edge cases.
    std::vector<Points> rings{
        {{0, 0}, {1000, 99}, {1100, -9}, {2000, 0}, {0, 1000}},
        {}, {{0, 0}}, {{0, 0}, {10, 10}}, {{0, 0}, {10, 0}, {20, 0}, {30, 0}, {40, 0}},
        {{0, 0}, {0, 0}, {10, 0}, {10, 0}, {10, 10}, {0, 10}}, {{0, 0}, {100, 0}, {50, 1}, {200, 0}, {0, 100}},
        {{0, 0}, {100, 0}, {-50, 1}, {200, 0}, {0, 100}}};
    std::mt19937 random(20261019);
    auto uniform = [&random](int low, int high) { return std::uniform_int_distribution<int>(low, high)(random); };
    for (int test = 0; test < 600; ++test) {
        Points ring;
        Point walk(uniform(-60, 60), uniform(-60, 60));
        for (int idx = 0, count = uniform(0, 14); idx < count; ++idx) {
            if (test % 2 == 0) {
                ring.emplace_back(uniform(-60, 60), uniform(-60, 60));
            } else {
                walk = Point(walk.x() + uniform(-2, 2) * 20 + uniform(-1, 1), walk.y() + uniform(-2, 2) * 20 + uniform(-1, 1));
                ring.push_back(walk);
            }
        }
        rings.push_back(std::move(ring));
    }
    auto numbers = [](const Points &points) {
        std::string out;
        for (const Point &point : points)
            out += (out.empty() ? "" : " ") + std::to_string((long long) point.x()) + " " + std::to_string((long long) point.y());
        return out;
    };
    std::ostringstream lines;
    for (const Points &ring : rings)
        for (const coord_t offset : {coord_t(0), coord_t(1), coord_t(5), coord_t(100), coord_t(5000)}) {
            Polygon polygon;
            polygon.points = ring;
            polygon.remove_collinear(offset);
            lines << offset << "|" << numbers(ring) << "|" << numbers(polygon.points) << "\n";
        }

    TemporaryFiles files;
    const std::filesystem::path script = files.make("collinear.py");
    const std::filesystem::path input = files.make("collinear.txt");
    write_text(input, lines.str());
    write_text(script, "PARAMS = {\"input\": r\'" + input.string() + "\'}\n" + python_collinear_script);
    CHECK(run_python_script(python, script));
}

#endif // SLIC3R_TEST_PYTHON_PLUGINS

// ---- timing bench of the object slicing (hidden: run by name) ----
//
//   SLIC3R_SLICING_BENCH_REPS=<n>     repetitions of each path (default 5)
//   SLIC3R_SLICING_BENCH_THREADS=<n>  tbb max_allowed_parallelism (0 or unset: no limit)
// The native path (slice_mesh_ex + ensure_valid with the copy and flip of the host), the C++ helper and the raw loops alone are
// timed in turn, the order rotating at every repetition. One untimed run of each comes first.

namespace {

int bench_environment_int(const char *name, int fallback)
{
    const char *value = std::getenv(name);
    return value != nullptr && *value != 0 ? std::atoi(value) : fallback;
}

// Z every step millimeters over the whole height of the mesh, in the middle of each layer.
std::vector<float> bench_slice_zs(const TriangleMesh &mesh, double step)
{
    const BoundingBoxf3 box = mesh.bounding_box();
    std::vector<float> zs;
    for (double z = box.min.z() + step / 2.; z < box.max.z(); z += step)
        zs.push_back(float(z));
    return zs;
}

// Wall time of one call, in milliseconds.
template<class Fn> double bench_time_ms(Fn &&fn)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    fn();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double bench_median(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    return values.empty() ? 0. : values[values.size() / 2];
}

double bench_min(const std::vector<double> &values)
{
    return values.empty() ? 0. : *std::min_element(values.begin(), values.end());
}

// The default-like settings of the object slicing.
c_mesh_slicing_params bench_params()
{
    return helper_slicing_params({RAW_MESH_SLICING_MODE_REGULAR, RAW_MESH_SLICING_MODE_REGULAR, 0, 0.049f, 0.f, 0.0001, 0.0125}, false);
}

} // namespace

TEST_CASE("Object slicing timing bench", "[.][slicing-bench]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const int reps = std::max(1, bench_environment_int("SLIC3R_SLICING_BENCH_REPS", 5));
    const int threads = bench_environment_int("SLIC3R_SLICING_BENCH_THREADS", 0);
    std::optional<tbb::global_control> limit;
    if (threads > 0)
        limit.emplace(tbb::global_control::max_allowed_parallelism, size_t(threads));

    for (const char *mesh_name : {"extruder_idler", "A", "frog_legs"}) {
        const TriangleMesh mesh = slicing_dump_mesh(mesh_name);
        const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
        const c_mesh_slicing_params params = bench_params();
        const std::vector<float> zs = bench_slice_zs(mesh, 0.2);
        const std::vector<std::function<void()>> paths{
            [&] { native_slices(mesh, params, zs); },
            [&] {
                PluginStorage storage;
                view.slice_to_expolygons(reinterpret_cast<storage_handle *>(&storage), params, zs, nullptr);
            },
            [&] {
                PluginStorage storage;
                view.slice_to_polygons(reinterpret_cast<storage_handle *>(&storage), params, zs, nullptr);
            }};
        for (const std::function<void()> &path : paths)
            path();
        std::vector<std::vector<double>> times(paths.size());
        for (int rep = 0; rep < reps; ++rep)
            for (size_t turn = 0; turn < paths.size(); ++turn) {
                const size_t path = (turn + size_t(rep)) % paths.size();
                times[path].push_back(bench_time_ms(paths[path]));
            }
        std::cerr << "bench mesh=" << mesh_name << " layers=" << zs.size() << " threads=" << threads << " reps=" << reps
                  << " native_med=" << bench_median(times[0]) << " native_min=" << bench_min(times[0])
                  << " helper_med=" << bench_median(times[1]) << " helper_min=" << bench_min(times[1])
                  << " raw_med=" << bench_median(times[2]) << " raw_min=" << bench_min(times[2])
                  << " ratio=" << bench_median(times[1]) / bench_median(times[0]) << "\n";
    }
    SUCCEED();
}

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

namespace {

// The Python script that times the Python slice_to_expolygons (result freed inside the time) and writes the times in ms.
const char *const python_bench_script = R"PY(
import time
import slic3r_api
import slic3r_api_generated as generated
import slic3r_datatree_views as views

api = slic3r_api._registered_apis[0]
mesh = views.TriangleMesh(api, PARAMS["mesh"])
params = generated.CMeshSlicingParams()
params.mode = PARAMS["mode"]
params.mode_below = PARAMS["mode_below"]
params.slicing_mode_normal_below_layer = PARAMS["below_layer"]
for index, value in enumerate(PARAMS["transform"]):
    params.transform.value[index] = value
params.closing_radius = PARAMS["closing_radius"]
params.extra_offset = PARAMS["extra_offset"]
params.model_resolution = PARAMS["model_resolution"]
params.resolution = PARAMS["resolution"]

def run():
    layers = mesh.slice_to_expolygons(PARAMS["storage"], params.transform, PARAMS["zs"], params)
    for layer in layers:
        layer.free_from_storage()

run()
times = []
for _repetition in range(PARAMS["reps"]):
    start = time.perf_counter()
    run()
    times.append((time.perf_counter() - start) * 1000.0)
with open(PARAMS["output"], "w", encoding="utf-8", newline="\n") as stream:
    stream.write(" ".join(repr(value) for value in times) + "\n")
)PY";

} // namespace

TEST_CASE("Python slicing timing bench on the real host", "[.][slicing-bench][python-real-host]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const PythonEntryPoints python = find_python_entry_points();
    REQUIRE(Test::Plugins::python_plugin_test_runtime_available());
    REQUIRE(python.run_simple_string != nullptr);
    const int reps = std::max(1, bench_environment_int("SLIC3R_SLICING_BENCH_REPS", 5));
    const int threads = bench_environment_int("SLIC3R_SLICING_BENCH_THREADS", 0);
    std::optional<tbb::global_control> limit;
    if (threads > 0)
        limit.emplace(tbb::global_control::max_allowed_parallelism, size_t(threads));

    for (const char *mesh_name : {"extruder_idler", "A"}) {
        const TriangleMesh mesh = slicing_dump_mesh(mesh_name);
        const slic3r_api::TriangleMesh view(reinterpret_cast<const triangle_mesh_handle *>(&mesh));
        const c_mesh_slicing_params params = bench_params();
        const std::vector<float> zs = bench_slice_zs(mesh, 0.2);
        PluginStorage storage;
        storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);

        view.slice_to_expolygons(handle, params, zs, nullptr);
        std::vector<double> cpp_times;
        for (int rep = 0; rep < reps; ++rep)
            cpp_times.push_back(bench_time_ms([&] { view.slice_to_expolygons(handle, params, zs, nullptr); }));

        TemporaryFiles files;
        const std::filesystem::path script = files.make("bench.py");
        const std::filesystem::path output = files.make("bench.txt");
        std::ostringstream text;
        text.precision(17);
        text << "PARAMS = {\"mesh\": " << reinterpret_cast<uintptr_t>(&mesh) << ", \"storage\": " << reinterpret_cast<uintptr_t>(handle)
             << ", \"reps\": " << reps << ", \"mode\": " << int(params.mode) << ", \"mode_below\": " << int(params.mode_below)
             << ", \"below_layer\": " << params.slicing_mode_normal_below_layer << ", \"transform\": [";
        for (int idx = 0; idx < 16; ++idx)
            text << (idx ? ", " : "") << params.transform.value[idx];
        text << "], \"closing_radius\": " << double(params.closing_radius) << ", \"extra_offset\": " << double(params.extra_offset)
             << ", \"model_resolution\": " << params.model_resolution << ", \"resolution\": " << params.resolution << ", \"zs\": [";
        for (size_t idx = 0; idx < zs.size(); ++idx)
            text << (idx ? ", " : "") << double(zs[idx]);
        text << "], \"output\": r\'" << output.string() << "\'}\n" << python_bench_script;
        write_text(script, text.str());
        REQUIRE(run_python_script(python, script));

        std::istringstream result(read_text(output.string()));
        std::vector<double> python_times;
        for (double value; result >> value;)
            python_times.push_back(value);
        REQUIRE(python_times.size() == size_t(reps));
        std::cerr << "bench-python mesh=" << mesh_name << " layers=" << zs.size() << " threads=" << threads << " reps=" << reps
                  << " py_med=" << bench_median(python_times) << " py_min=" << bench_min(python_times)
                  << " cpp_helper_med=" << bench_median(cpp_times) << " cpp_helper_min=" << bench_min(cpp_times)
                  << " ratio=" << bench_median(python_times) / bench_median(cpp_times) << "\n";
    }
    SUCCEED();
}

#endif // SLIC3R_TEST_PYTHON_PLUGINS

TEST_CASE("Post-slicing XY compensation owns regional dimensional settings",
          "[plugins][xy-size-compensation][config]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin("slicing.xy_size_compensation");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_POST_SLICING);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 5);

    // Profile keys keep their types and defaults while moving into regional
    // scope. The plugin owns all five; their FFF edits rerun post-slicing.
    const struct ExpectedOption {
        const char *key;
        ConfigOptionType type;
        const char *default_value;
    } expected[] = {
        { "xy_size_compensation", coFloat, "0" },
        { "xy_inner_size_compensation", coFloat, "0" },
        { "first_layer_size_compensation", coFloat, "0" },
        { "first_layer_size_compensation_layers", coInt, "1" },
        { "first_layer_size_compensation_no_collapse", coBool, "1" }
    };
    for (const ExpectedOption &option : expected) {
        CAPTURE(option.key);
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), option.key) != defined_keys.end());
        const ConfigOptionDef *def = PrintConfigDef::instance().get(option.key);
        REQUIRE(def != nullptr);
        CHECK(def->type == option.type);
        CHECK(def->container_type == ConfigOptionContainerType::Region);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(def->invalidates_step == STEP_POST_SLICING);
        CHECK(def->default_value->serialize() == option.default_value);
        CHECK(PrintRegionConfig::defaults().option(option.key) != nullptr);
        CHECK(PrintObjectConfig::defaults().option(option.key) == nullptr);
        if (option.type == coFloat)
            CHECK(def->precision == 6);
    }
}

TEST_CASE("SLA first-layer compensation has its own native definition",
          "[plugins][xy-size-compensation][config]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto &definitions = PrintConfigDef::instance();
    const auto *sla = definitions.get("sla_first_layer_size_compensation");
    const auto *material = definitions.get("material_ow_sla_first_layer_size_compensation");
    REQUIRE(sla != nullptr);
    REQUIRE(material != nullptr);
    CHECK(sla->type == coFloat);
    CHECK((sla->printer_technology & ptSLA) == ptSLA);
    CHECK(material->type == sla->type);
    CHECK(material->default_value->get_float() == sla->default_value->get_float());
    // SLA material overrides are disabled until explicitly enabled by the user.
    CHECK_FALSE(material->default_value->is_enabled());
    CHECK(definitions.get("material_ow_first_layer_size_compensation") == nullptr);

    // SLA printer profiles and Prusa imports are renamed; FFF profiles keep the plugin key.
    const auto convert = [](std::map<std::string, std::string> entries) {
        std::unordered_map<t_config_option_key, std::pair<t_config_option_key, std::string>> dict;
        for (const auto &[key, value] : entries)
            dict[key] = { key, value };
        PrintConfigDef::handle_legacy_map(dict, false);
        std::map<std::string, std::string> out;
        for (const auto &[key, entry] : dict)
            if (!entry.first.empty())
                out[entry.first] = entry.second;
        return out;
    };
    const auto sla_printer = convert({ { "printer_technology", "SLA" }, { "first_layer_size_compensation", "-0.1" } });
    CHECK(sla_printer.count("sla_first_layer_size_compensation") == 1);
    CHECK(sla_printer.count("first_layer_size_compensation") == 0);
    const auto prusa_sla = convert({ { "elefant_foot_min_width", "0.2" }, { "elefant_foot_compensation", "0.2" } });
    REQUIRE(prusa_sla.count("sla_first_layer_size_compensation") == 1);
    CHECK(prusa_sla.at("sla_first_layer_size_compensation") == "-0.2");
    const auto material_profile = convert({ { "material_ow_first_layer_size_compensation", "-0.1" } });
    CHECK(material_profile.count("material_ow_sla_first_layer_size_compensation") == 1);
    const auto fff = convert({ { "printer_technology", "FFF" }, { "elefant_foot_compensation", "0.2" } });
    CHECK(fff.count("first_layer_size_compensation") == 1);
    CHECK(fff.count("sla_first_layer_size_compensation") == 0);
}

TEST_CASE("XY compensation declares its external-flow inputs and invalidates post-slicing",
          "[plugins][xy-size-compensation][config]")
{
    using namespace Slic3r;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin("slicing.xy_size_compensation");
    REQUIRE(plugin != nullptr);

    const std::vector<Plugin::UsedConfigKey> used = plugin->get_used_config_keys();
    for (const char *key : { "external_perimeter_overlap", "external_perimeter_extrusion_change_odd_layers" }) {
        CAPTURE(key);
        const auto found = std::find_if(used.begin(), used.end(), [key](const Plugin::UsedConfigKey &used_key) {
            return used_key.key == key;
        });
        REQUIRE(found != used.end());

        const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
        REQUIRE(definition != nullptr);
        // These native settings traditionally replay at perimeter generation;
        // reading them in this plugin tightens invalidation to post-slicing.
        CHECK(definition->invalidates_step == STEP_POST_SLICING);
    }
}
