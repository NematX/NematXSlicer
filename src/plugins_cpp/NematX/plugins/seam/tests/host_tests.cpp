// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#include "../src/InternalSeam.hpp"

#include <cmath>
#include <filesystem>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <crtdbg.h>
#endif
#include <tbb/global_control.h>
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/internal/ArcPolylineAccess.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Utils.hpp"

/*
Native checks of the independently packaged seam decorator
==========================================================
The real plugin runtime creates a tiny source object and two true instances.
Each isolated case supplies a canonical plan loop and transformed model volumes.
A deliberately simple fallback returns the loop's first point; this makes an
accidental fallback observable rather than accepting a plausible host seam.
The host's SeamPlacer adapter validates every resulting line/arc point.

    main -> initialize production definitions -> make one tiny Print
         -> geometry_cases: roles, height, transforms, instances, ties, arcs
         -> constraint_cases: paint blockers/enforcers and inside volumes
         -> ownership_cases: cancellation, diagnostics and concurrent reads

All temporary meshes and plans belong to a case; the initialized decorator is
destroyed before its borrowed source is changed. Checks throw in all builds.
These are direct service tests; separate exports exercise actual host fallback,
selector metadata, standalone loading and final G-code ordering.
*/
namespace {
using namespace Slic3r;
namespace api = slic3r_api;
size_t checks = 0;
std::filesystem::path replay_output;

// Fail visibly in every configuration and count all checked expectations.
void check(bool condition, const std::string &message);
// Load the production providers before configuration caches become immutable.
void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults,
                const std::filesystem::path &resources);
// Compare platter points after removing the known instance shift.
void point_is(c_point actual, c_point shift, double x, double y, const char *message);
// Build a continuous square with its original seam at the lower-left corner.
ExtrusionEntity square();
// Build a true circle from minor or major analytic arcs, not sampled points.
ExtrusionEntity circle(bool major = false);
// Return a known loop vertex, preserving its platter-coordinate frame.
void base_place(const void *, const extrusion_entity_handle *loop, const layer_region_handle *,
                c_point, raw_seam_placer_result *out);
// Rank regions at a distinct value so tests detect unintentional replacement.
void base_priority(const void *, const extrusion_entity_handle *, const layer_region_handle *,
                   raw_seam_placer_priority_result *out);
// Delete the test fallback and record whether ownership was released.
void base_destroy(void *opaque);
// Cancel preparation at its first explicit cancellation check.
int cancel_now(plugin_host_context *);

struct BaseState { size_t *destroyed; bool fail = false; };
const raw_seam_placer_vtable base_vtable{sizeof(raw_seam_placer_vtable), base_destroy, base_priority, base_place};

struct World {
    Model model;
    Print print;
    DynamicPrintConfig config;
    // Process one tiny object, sharing its source layers across all cases.
    World();
};

struct Case {
    World &world;
    PrintObject &object;
    LayerRegion &region;
    std::unique_ptr<LayerRegionIsland> source;
    Printing::PrintingPlan native_plan;
    api::PrintingPlan plan;
    PluginStorage storage;
    size_t original_volumes;
    size_t destroyed = 0;
    // Borrow one source layer and give each case its own plan and mesh additions.
    explicit Case(World &world, size_t layer = 0);
    // Remove only this case's in-memory modifiers after its sessions are gone.
    ~Case();
    // Add a canonical plan loop, mark its role and shift it to the exact instance.
    api::ExtrusionEntity add(const ExtrusionEntity &loop, raw_extrusion_role role,
                             uint16_t instance = 0, bool skirt = false);
    // Add model-space constraints by mapping a desired centered-object box backwards.
    ModelVolume *box(double x, double y, double sx, double sy, double z = 0,
                     double height = 2, ModelVolumeType type = ModelVolumeType::SEAM_POSITION_INTERNAL_CENTER_Z,
                     EnforcerBlockerType paint = EnforcerBlockerType::NONE);
    // Publish the decorated stub through the actual host-owned service adapter.
    api::SeamPlacer session(bool fail = false, const plugin_run_context *context = nullptr);
    // Return a borrowed regional view suitable for this plan's source layer.
    api::LayerRegion view() const;
    // Return the exact platter shift recorded in the source instance catalogue.
    c_point shift(uint16_t instance = 0) const;
};

// Exercise all geometry rules, including segment roles beneath collection loops.
void geometry_cases(World &world);
// Prove stronger source constraints are not overwritten by an attractor.
void constraint_cases(World &world);
// Check session lifetime, concurrent determinism and visible ABI errors.
void ownership_cases(World &world);
// Compare moved/converted volume replays against fresh Prints and the original plan.
void replay_cases();
// Snapshot complete ordered geometry, not just selected seam coordinates.
std::string signature(Print &print);

void check(bool condition, const std::string &message)
{
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void initialize(const std::filesystem::path &data, const std::filesystem::path &defaults,
                const std::filesystem::path &resources)
{
    std::filesystem::create_directories(data / "plugins");
    replay_output = data / "replay-exports";
    std::filesystem::create_directories(replay_output);
    std::filesystem::copy_file(defaults, data / "plugins/activated.ini", std::filesystem::copy_options::overwrite_existing);
    set_data_dir(data.string()); set_resources_dir(resources.string()); set_logging_level(0);
    PrintConfigDef::instance_mutable().init_common_params();
    init_fff_params(PrintConfigDef::instance_mutable()); init_sla_params(PrintConfigDef::instance_mutable());
    load_plugins();
    initialize_fff_print_config_cache(); initialize_sla_print_config_cache();
    PrintConfigDef::instance_mutable().finalize();
    check(Orchestrator::instance().is_plugin_active("seam_placer.cost"), "missing Cost fallback");
    const Plugin *provider = Orchestrator::instance().get_plugin("nematx_seam");
    check(provider != nullptr && Orchestrator::instance().is_plugin_active("nematx_seam"), "missing NematX seam provider");
    check(provider->get_defined_config_keys().empty(), "package unexpectedly defines an untested setting");
    std::set<std::string> used;
    for (const Plugin::UsedConfigKey &key : provider->get_used_config_keys()) used.insert(key.key);
    check(used == std::set<std::string>{"seam_angle_cost", "seam_travel_cost", "seam_visibility"},
          "declared setting lacks a domain in the export matrix");
    check(provider->get_priority() == 600 && provider->get_exclusive_group_priority() == -50,
          "seam execution/default selection priorities changed");
    const std::string selector = Steps::plugin_exclusive_group_option_key(Orchestrator::instance(), SEAM_PLACER, "seam_placer_plugin");
    check(DynamicPrintConfig::full_print_config().option(selector)->serialize() == "nematx_seam", "decorator is not the default seam provider");
}

void point_is(c_point actual, c_point shift, double x, double y, const char *message)
{
    const double dx = unscaled(actual.x - shift.x) - x;
    const double dy = unscaled(actual.y - shift.y) - y;
    check(std::hypot(dx, dy) < 0.0001, std::string(message) + ": got " +
          std::to_string(dx + x) + "," + std::to_string(dy + y));
}

ExtrusionEntity square()
{
    return ExtrusionEntity(false, ArcPolyline(Points{Point::new_scale(-2, -2), Point::new_scale(2, -2),
        Point::new_scale(2, 2), Point::new_scale(-2, 2), Point::new_scale(-2, -2)}));
}

ExtrusionEntity circle(bool major)
{
    const coord_t r = scale_i(2.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-r, 0), 0.f, Geometry::ArcWelder::Orientation::Unknown);
    if (major) {
        path.emplace_back(Point(0, r), -float(r), Geometry::ArcWelder::Orientation::CCW);
        path.emplace_back(Point(-r, 0), float(r), Geometry::ArcWelder::Orientation::CCW);
    } else {
        for (const Point &p : Points{Point(0, -r), Point(r, 0), Point(0, r), Point(-r, 0)})
            path.emplace_back(p, float(r), Geometry::ArcWelder::Orientation::CCW);
    }
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    return ExtrusionEntity(false, polyline);
}

void base_place(const void *opaque, const extrusion_entity_handle *loop, const layer_region_handle *,
                c_point, raw_seam_placer_result *out)
{
    out->status = RAW_SEAM_PLACER_STATUS_SUCCESS; out->error_message = nullptr;
    if (static_cast<const BaseState *>(opaque)->fail) {
        out->status = RAW_SEAM_PLACER_STATUS_ERROR; out->error_message = "deliberate fallback failure";
    } else out->point = api::ExtrusionEntity(loop).front();
}

void base_priority(const void *, const extrusion_entity_handle *, const layer_region_handle *,
                   raw_seam_placer_priority_result *out)
{
    out->status = RAW_SEAM_PLACER_STATUS_SUCCESS; out->priority = 123.f; out->error_message = nullptr;
}

void base_destroy(void *opaque)
{
    std::unique_ptr<BaseState> state(static_cast<BaseState *>(opaque));
    ++*state->destroyed;
}

int cancel_now(plugin_host_context *) { return 1; }

World::World()
{
    ModelObject *object = model.add_object("tiny", "tiny.stl", make_cube(4, 4, 1.5));
    object->add_instance()->set_offset(Vec3d(10, 10, 0));
    object->add_instance()->set_offset(Vec3d(30, 10, 0));
    config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.3"}, {"first_layer_height", "0.3"},
        {"seam_visibility", "0"}, {"skirts", "0"}, {"brim_width", "0"}, {"perimeters", "2"},
        {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}});
    print.auto_assign_extruders(object); print.apply(model, config); print.set_status_silent(); print.process();
    check(print.object(0).instances().size() == 2, "fixture does not have real copies");
    check(print.object(0).layer_count() == 5, "fixture is not five layers");
}

Case::Case(World &input, size_t layer) : world(input), object(world.print.object(0)),
    region(object.layer(layer).region(0)), source(new LayerRegionIsland({&region}, 0)),
    plan(reinterpret_cast<printing_plan_handle *>(&native_plan)), original_volumes(object.model_object()->volumes.size())
{
    plan.append_group().append_layer_group(view().layer().print_z()).append_tool_group(0);
}

Case::~Case()
{
    ModelObject *model = const_cast<ModelObject *>(object.model_object());
    while (model->volumes.size() > original_volumes) model->delete_volume(model->volumes.size() - 1);
}

api::ExtrusionEntity Case::add(const ExtrusionEntity &loop, raw_extrusion_role role, uint16_t instance, bool skirt)
{
    const uint32_t catalog = plan.find_or_add_object_instance(view().layer().object(), instance);
    api::PrintingExtrusion extrusion = plan.group(0).layer_group(0).tool_group(0).append_extrusion_clone(
        plan, api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(source.get())),
        RAW_EXTRUSION_ROLE_PERIMETER, api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&loop)), catalog);
    api::MutableExtrusionEntity root = extrusion.mutable_root();
    c_extrusion_property_attributes *attributes = static_cast<c_extrusion_property_attributes *>(
        root.get_or_add_property_data_mutable(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    attributes->role = role; attributes->width = 0.4f;
    c_extrusion_property_perimeter *marker = static_cast<c_extrusion_property_perimeter *>(root.get_or_add_property_data_mutable(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()), EXTRUSION_PROPERTY_TYPE_PERIMETER));
    marker->perimeter_flags = skirt ? C_EXTRUSION_PERIMETER_FLAG_SKIRT : C_EXTRUSION_PERIMETER_FLAG_LOOP;
    std::function<void(api::MutableExtrusionEntity)> translate;
    translate = [&](api::MutableExtrusionEntity entity) {
        if (entity.point_count() != 0) check(entity.translate(shift(instance)), "cannot translate canonical loop");
        for (uint32_t i = 0; i < entity.child_count(); ++i) translate(entity.child_mutable(i));
    };
    translate(root);
    return root;
}

ModelVolume *Case::box(double x, double y, double sx, double sy, double z, double height,
                       ModelVolumeType type, EnforcerBlockerType paint)
{
    TriangleMesh mesh = make_cube(sx, sy, height);
    mesh.translate(Vec3f(float(x), float(y), float(z)));
    mesh.transform(object.trafo_centered().inverse());
    ModelVolume *volume = const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh), type, false);
    if (paint != EnforcerBlockerType::NONE) {
        TriangleSelector selector(volume->mesh());
        for (size_t i = 0; i < volume->mesh().its.indices.size(); ++i) selector.set_facet(int(i), paint);
        volume->facets_annotation_mutable("builtin:seam").set(selector);
    }
    return volume;
}

api::SeamPlacer Case::session(bool fail, const plugin_run_context *context)
{
    raw_seam_placer_instance base{sizeof(raw_seam_placer_instance), new BaseState{&destroyed, fail}, &base_vtable};
    raw_seam_placer_instance result = nematx_seam::make_internal_seam_instance(base, plan,
        {view().print_region()}, reinterpret_cast<storage_handle *>(&storage), context);
    check(base.session == nullptr && base.vtable == nullptr, "factory did not transfer fallback ownership");
    return api::SeamPlacer(result);
}

api::LayerRegion Case::view() const { return api::LayerRegion(reinterpret_cast<const layer_region_handle *>(&region)); }
c_point Case::shift(uint16_t instance) const { return view().layer().object().instance_shift(instance); }

void geometry_cases(World &world)
{
    for (const raw_extrusion_role role : {RAW_EXTRUSION_ROLE_PERIMETER, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER,
                                         RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER, RAW_EXTRUSION_ROLE_SUPPORT}) {
        Case test(world); test.box(3, -0.5, 1, 1);
        const api::ExtrusionEntity loop = test.add(square(), role);
        api::SeamPlacer session = test.session();
        const bool internal = RAW_EXTRUSION_ROLE_IS_PERIMETER(role) && !RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(role);
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), internal ? 2 : -2, internal ? 0 : -2,
                 "internal-only role selection");
        check(session.get_priority(loop, test.view()) == 123, "changed host priority");
    }
    for (const double z : {-2., 0., 0.31}) {
        Case test(world); test.box(3, -0.5, 1, 1, z, 0.6);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), z == 0 ? 2 : -2, z == 0 ? 0 : -2,
                 "attractor escaped its sliced height");
    }
    {
        Case test(world); const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), -2, -2, "no-volume fallback changed");
    }
    {
        Case test(world); test.box(3, -0.5, 1, 1);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER, 0, true);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), -2, -2, "technical skirt was attracted");
    }
    {
        Case test(world); test.box(3, -0.5, 1, 1);
        const api::ExtrusionEntity a = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER, 0);
        const api::ExtrusionEntity b = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER, 1);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(a, test.view(), {999, 999}), test.shift(0), 2, 0, "first instance frame");
        point_is(session.place_seam(b, test.view(), {-999, -999}), test.shift(1), 2, 0, "second instance frame");
    }
    {
        Case test(world); ModelVolume *volume = test.box(3, -0.5, 1, 1);
        volume->set_offset(Vec3d(-3.5, 3.5, 0));
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), 0, 2, "volume transform ignored");
    }
    {
        Case test(world); ModelVolume *volume = test.box(3, -0.5, 1, 1);
        volume->set_mirror(Vec3d(-1, 1, 1));
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), -2, 0, "volume mirror ignored");
    }
    {
        Case test(world); ModelVolume *volume = test.box(-0.5, 3, 1, 1);
        volume->set_scaling_factor(Vec3d(2, 1, 1));
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), 2, 2, "nonuniform volume scale ignored");
    }
    {
        Case test(world); test.box(3, -0.5, 1, 1); test.box(-0.5, 3, 1, 1);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), 2, 0, "volume-order tie is unstable");
    }
    {
        Case test(world); test.box(8, -0.5, 1, 1); test.box(-0.5, 3, 1, 1);
        const api::ExtrusionEntity loop = test.add(circle(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), 0, 2, "nearest attractor / analytic arc");
    }
    for (const bool external : {false, true}) {
        Case test(world); test.box(3, -0.5, 1, 1);
        ExtrusionEntity::Children children;
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(-2, -2), Point::new_scale(2, -2), Point::new_scale(2, 2)})));
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(2, 2), Point::new_scale(-2, 2), Point::new_scale(-2, -2)})));
        const ExtrusionEntity composed(std::move(children), false, false, false);
        const api::ExtrusionEntity loop = test.add(composed, RAW_EXTRUSION_ROLE_MIXED);
        api::MutableExtrusionEntity root(const_cast<extrusion_entity_handle *>(loop.handle()));
        for (uint32_t i = 0; i < 2; ++i) {
            c_extrusion_property_attributes *attributes = static_cast<c_extrusion_property_attributes *>(
                root.child_mutable(i).get_or_add_property_data_mutable(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                    EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
            attributes->role = external && i == 1 ? RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER : RAW_EXTRUSION_ROLE_PERIMETER;
            attributes->width = 0.4f;
        }
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), external ? -2 : 2, external ? -2 : 0,
                 "descendant role inheritance / mixed loop protection");
    }
    for (const bool major : {false, true})
        for (const bool clockwise : {false, true}) {
            Case test(world); test.box(3, -0.5, 1, 1);
            ExtrusionEntity geometry = circle(major);
            if (clockwise) geometry.reverse();
            const api::ExtrusionEntity loop = test.add(geometry, RAW_EXTRUSION_ROLE_PERIMETER);
            api::SeamPlacer session = test.session();
            point_is(session.place_seam(loop, test.view(), {}), test.shift(), 2, 0, "clockwise/major analytic arc was replaced by a chord");
        }
}

void constraint_cases(World &world)
{
    for (const bool painted : {false, true}) {
        Case test(world); test.box(3, -0.5, 1, 1);
        test.box(-3, -0.5, 2, 1, 0, 2,
                 painted ? ModelVolumeType::MODEL_PART : ModelVolumeType::SEAM_POSITION_INSIDE,
                 painted ? EnforcerBlockerType::ENFORCER : EnforcerBlockerType::NONE);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), -2, 0, "enforcer lost to attractor");
    }
    {
        Case test(world); test.box(3, -0.5, 1, 1);
        test.box(1, -1, 2, 2, 0, 2, ModelVolumeType::MODEL_PART, EnforcerBlockerType::BLOCKER);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        const c_point point = session.place_seam(loop, test.view(), {});
        const double y = unscaled(point.y - test.shift().y);
        check(std::abs(y) > 1.39, "rounded result remained inside the closed painted blocker");
        check(std::abs(y) < 1.41, "blocker projection skipped the nearest admissible portion");
    }
    {
        Case test(world); test.box(3, -0.5, 1, 1);
        test.box(-4, -4, 8, 8, 0, 2, ModelVolumeType::MODEL_PART, EnforcerBlockerType::BLOCKER);
        const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
        api::SeamPlacer session = test.session();
        point_is(session.place_seam(loop, test.view(), {}), test.shift(), -2, -2, "all-blocked host fallback changed");
    }
}

void ownership_cases(World &world)
{
    Case test(world); test.box(3, -0.5, 1, 1);
    const api::ExtrusionEntity loop = test.add(square(), RAW_EXTRUSION_ROLE_PERIMETER);
    {
        api::SeamPlacer session = test.session();
        std::vector<std::future<c_point>> jobs;
        for (size_t i = 0; i < 16; ++i)
            jobs.push_back(std::async(std::launch::async, [&] { return session.place_seam(loop, test.view(), {}); }));
        for (std::future<c_point> &job : jobs) point_is(job.get(), test.shift(), 2, 0, "concurrent seam changed");
        check(test.destroyed == 0, "fallback destroyed while published");
    }
    check(test.destroyed == 1, "fallback not destroyed exactly once");
    {
        api::SeamPlacer failed = test.session(true);
        try { failed.place_seam(loop, test.view(), {}); check(false, "fallback failure was swallowed"); }
        catch (const std::runtime_error &error) { check(std::string(error.what()) == "deliberate fallback failure", "fallback diagnostic changed"); }
    }
    plugin_run_context cancelled{}; cancelled.is_cancelled = cancel_now;
    try { test.session(false, &cancelled); check(false, "cancelled initialization published a session"); }
    catch (const api::PluginCancelled &) { check(test.destroyed == 3, "cancelled initialization leaked fallback"); }
}

std::string signature(Print &print)
{
    const api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan()));
    std::ostringstream result;
    size_t points = 0;
    for (uint32_t g = 0; g < plan.group_count(); ++g)
        for (uint32_t l = 0; l < plan.group(g).layer_group_count(); ++l) {
            const api::ConstPrintingLayerGroup layer = plan.group(g).layer_group(l);
            result << '@' << layer.print_z();
            for (uint32_t t = 0; t < layer.tool_group_count(); ++t)
                for (uint32_t e = 0; e < layer.tool_group(t).extrusion_count(); ++e)
                    for (const c_point point : layer.tool_group(t).extrusion(e).root().collect_points()) {
                        ++points;
                        result << '[' << point.x << ',' << point.y << ']';
                    }
        }
    check(points > 0, "replay did not run export-side ordering; the plan is empty");
    return result.str();
}

void replay_cases()
{
    World world;
    Steps::StepPipeline::run_gcode(Orchestrator::instance(), world.print, (replay_output / "baseline.gcode").string());
    const std::string original = signature(world.print);
    TriangleMesh mesh = make_cube(1, 1, 0.7);
    mesh.translate(Vec3f(5.f, 1.5f, 0.35f));
    ModelVolume *volume = world.model.objects().front().add_volume(std::move(mesh), ModelVolumeType::SEAM_POSITION_INTERNAL_CENTER_Z, false);
    std::string previous = original;
    for (int change = 0; change < 3; ++change) {
        if (change == 1) volume->set_offset(Vec3d(-5.5, 3.5, 0));
        if (change == 2) volume->set_type(ModelVolumeType::SEAM_POSITION_CENTER_Z);
        world.print.apply(world.model, world.config); world.print.process();
        Steps::StepPipeline::run_gcode(Orchestrator::instance(), world.print,
            (replay_output / ("replay-" + std::to_string(change) + ".gcode")).string());
        Print fresh;
        fresh.apply(world.model, world.config); fresh.set_status_silent(); fresh.process();
        Steps::StepPipeline::run_gcode(Orchestrator::instance(), fresh,
            (replay_output / ("fresh-" + std::to_string(change) + ".gcode")).string());
        const std::string actual = signature(world.print);
        check(actual == signature(fresh), "moving/converting a volume did not recompute the same plan as a fresh Print");
        check(actual != previous, "adding/moving/converting the attractor left stale ordered geometry: change=" + std::to_string(change));
        if (change == 2) check(actual == original, "conversion to an unsupported attractor did not restore the host plan");
        previous = actual;
    }
}
} // namespace

// Run in a fresh process so production registration cannot alter another test runtime.
int main(int argc, char **argv)
{
#ifdef _WIN32
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE); _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
    try {
        if (argc != 4) throw std::runtime_error("Expected data directory, default activation file and resources.");
        tbb::global_control workers(tbb::global_control::max_allowed_parallelism, 1);
        initialize(argv[1], argv[2], argv[3]);
        World world;
        geometry_cases(world); constraint_cases(world); ownership_cases(world); replay_cases();
        std::cout << "Internal seam host checks passed: " << checks << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Internal seam check failed: " << error.what() << '\n'; return 1;
    }
}
