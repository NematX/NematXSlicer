// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
// Run the regional fan tree pass on stored entities through the public SDK,
// linked to the existing host library: role choice, protected requests,
// excluded connectors, splits at region boundaries and one-shot events.
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "RegionalFanTree.hpp"

namespace api = slic3r_api;
using namespace nematx_regional_fan;

namespace {
void check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

c_point cp(double x, double y) { return {scale_i(x), scale_i(y)}; }

// One mask from x0 to x1 and y from -1 to 1 (mm).
api::StoredExPolygonCollection band(storage_handle *storage, double x0, double x1)
{
    api::StoredExPolygonCollection area(storage);
    api::StoredExPolygon polygon(storage);
    for (c_point point : {cp(x0, -1), cp(x1, -1), cp(x1, 1), cp(x0, 1)})
        multipoint_push_back(polygon_as_multipoint(expolygon_contour(polygon.mutable_handle())), point);
    area.push_back_move(std::move(polygon));
    return area;
}

// A root holding one printable leaf of the given role, from x0 to x1 (mm).
api::StoredExtrusionEntity tree(storage_handle *storage, raw_extrusion_role role, double x0 = -10., double x1 = 10.)
{
    api::StoredExtrusionEntity root(storage);
    api::StoredExtrusionEntity leaf(storage, std::vector<c_point>{cp(x0, 0), cp(x1, 0)});
    leaf.get_or_add(api::EPropertyAttributes::key).extrusion_role(int32_t(role)).mm3_per_mm(0.04).width(0.45f).height(0.2f);
    root.append_child_move(leaf.mutable_view());
    return root;
}

void apply(api::StoredExtrusionEntity &root, const SourceRequests &source)
{
    std::vector<api::ExPolygonCollection> views;
    for (const api::StoredExPolygonCollection &area : source.areas)
        views.push_back(area.readonly());
    apply_tree(root.mutable_view(), std::nullopt, api::EPropertyFan{}, source, views, nullptr);
}

// The direct request of a leaf, or -1 without an active request.
float request(api::MutableExtrusionEntity leaf, fan_request_mode mode = FAN_REQUEST_MODE_OVERRIDE)
{
    const api::EPropertyFan *fan = leaf.get(api::EPropertyFan::key);
    if (fan == nullptr || !fan->is_set())
        return -1.f;
    check(fan->mode == mode, "unexpected fan request mode");
    return fan->percent;
}

void test(storage_handle *storage)
{
    SourceRequests uniform;
    uniform.uniform.percent[General] = 70.f;
    uniform.uniform.percent[Perimeters] = 45.f;
    uniform.uniform.percent[Bridge] = 0.f;
    const auto leaf_request = [&](raw_extrusion_role role) {
        api::StoredExtrusionEntity root = tree(storage, role);
        apply(root, uniform);
        return request(root.child_mutable(0));
    };
    check(leaf_request(RAW_EXTRUSION_ROLE_INTERNAL_INFILL) == 70.f, "general override");
    check(leaf_request(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER) == 45.f, "perimeter override wins");
    check(leaf_request(RAW_EXTRUSION_ROLE_OVERHANG_PERIMETER) == 70.f, "overhangs are not perimeters");
    check(leaf_request(RAW_EXTRUSION_ROLE_BRIDGE_INFILL) == 0.f, "an enabled 0% stops the fan");
    check(leaf_request(RAW_EXTRUSION_ROLE_SOLID_INFILL) == 70.f, "disabled solid override falls back");
    check(leaf_request(RAW_EXTRUSION_ROLE_SKIRT) == -1.f, "skirt uses the object's default region");
    check(leaf_request(RAW_EXTRUSION_ROLE_TRAVEL) == -1.f, "travel gets no request");

    // A zero-flow connector keeps the fan state of its neighbours.
    api::StoredExtrusionEntity connector = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    connector.child_mutable(0).get_or_add(api::EPropertySpecialCommand::key).set(C_EXTRUSION_SPECIAL_COMMAND_EXTRUSION, 0.);
    apply(connector, uniform);
    check(request(connector.child_mutable(0)) == -1.f, "raw extrusion connector excluded");

    // Automatic is replaced; Override, inherited or direct, is kept.
    api::StoredExtrusionEntity automatic = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    automatic.child_mutable(0).get_or_add(api::EPropertyFan::key).set(40.f, FAN_REQUEST_MODE_AUTOMATIC);
    apply(automatic, uniform);
    check(request(automatic.child_mutable(0)) == 70.f, "automatic request replaced");
    api::StoredExtrusionEntity inherited = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    inherited.get_or_add(api::EPropertyFan::key).set(20.f, FAN_REQUEST_MODE_OVERRIDE);
    apply(inherited, uniform);
    check(request(inherited.child_mutable(0)) == -1.f, "inherited override kept");

    // Two regions: 70% on the left, no override on the right.
    SourceRequests split;
    split.areas.push_back(band(storage, -11., 0.));
    split.areas.push_back(band(storage, 0., 11.));
    split.requests.resize(2);
    split.requests[0].percent[General] = 70.f;
    split.outside.percent[General] = 50.f;
    api::StoredExtrusionEntity crossing = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    apply(crossing, split);
    api::MutableExtrusionEntity cut = crossing.child_mutable(0);
    check(cut.child_count() == 2, "path split at the region boundary");
    check(request(cut.child_mutable(0)) == 70.f && request(cut.child_mutable(1)) == -1.f,
          "each fragment gets its region's request, in path order");
    api::StoredExtrusionEntity skirt = tree(storage, RAW_EXTRUSION_ROLE_SKIRT);
    apply(skirt, split);
    check(skirt.child_mutable(0).child_count() == 0 && request(skirt.child_mutable(0)) == 50.f,
          "skirt takes the object's value without split");
    split.requests[1].percent[General] = 70.f;
    api::StoredExtrusionEntity agreeing = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    apply(agreeing, split);
    check(agreeing.child_mutable(0).child_count() == 0 && request(agreeing.child_mutable(0)) == 70.f,
          "regions agreeing for the role need no split");

    // A one-shot event would be repeated by the split: the export is rejected.
    split.requests[1].percent[General].reset();
    api::StoredExtrusionEntity heated = tree(storage, RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
    heated.child_mutable(0).get_or_add(api::EPropertyHeater::key).set_tool(200);
    bool rejected = false;
    try {
        apply(heated, split);
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    check(rejected && heated.child_mutable(0).child_count() == 0, "one-shot event blocks the split");
}

// Never propagate an exception through the C callback.
void callback(uint32_t, storage_handle *storage, void *user)
{
    try {
        test(storage);
    } catch (...) {
        *static_cast<std::exception_ptr *>(user) = std::current_exception();
    }
}
} // namespace

int main()
{
    try {
        std::exception_ptr error;
        slic3r_parallel_for_storage(0, 1, &error, callback);
        if (error)
            std::rethrow_exception(error);
        std::cout << "regional fan SDK tests passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
