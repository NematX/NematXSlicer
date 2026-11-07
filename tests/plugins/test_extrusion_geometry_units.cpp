///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#include <catch2/catch.hpp>

#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/ExtrusionGeometryUnits.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/ConflictChecker.hpp"
#include "libslic3r/GCode/Travels.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/SupportSpotsGenerator.hpp"

/* Exercise stored-geometry readers without starting the GUI or a full print.
   Hand-built legacy and generic roots carry the same geometry and inherited
   attributes. Queries must restore sibling scopes, keep source handles and
   properties unchanged, and split unsupported paths or incompatible XY/Z joins.
   Consumer probes check collision values, obstacle identities, support integrals
   and the core-owned CPU inputs used by preview meshing, not GUI rendering.

       TEST_CASE
           |-- reader_root() [same stored content under distinct legacy shells]
           |-- collect_extrusion_geometry_units() -> extrusion_geometry_runs()
           |-- project_extrusion_geometry() [owned CPU mesh inputs]
           `-- reference_contour_area() [independent stored-segment area sum]
*/

using namespace Slic3r;

namespace {

// Create different native shells with identical stored permissions and scope.
ExtrusionEntityUPtr reader_root(const std::string &form, const ExtrusionAttributes &attributes);

// Sum the supplied segments independently, including arc tessellation, without a closing edge.
double reference_contour_area(const ArcPolylines &paths);

ExtrusionEntityUPtr reader_root(const std::string &form, const ExtrusionAttributes &attributes)
{
    ExtrusionEntityUPtr root;
    if (form == "path") root = std::make_unique<ExtrusionPath>(attributes);
    else if (form == "loop") root = std::make_unique<ExtrusionLoop>();
    else if (form == "multipath") root = std::make_unique<ExtrusionMultiPath>();
    else if (form == "collection") root = std::make_unique<ExtrusionEntityCollection>(false, false);
    else root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    root->children();
    root->add_property(attributes);
    root->set_can_sort_reverse(false, false);
    root->set_can_rotate(false);
    return root;
}

double reference_contour_area(const ArcPolylines &paths)
{
    double area = 0.;
    for (const ArcPolyline &path : paths) {
        REQUIRE(path.size() >= 2);
        if (path.has_arc()) {
            const Polyline polyline = path.to_polyline();
            Point previous = polyline.front();
            for (size_t idx = 1; idx < polyline.size(); ++idx) {
                const Point &current = polyline[idx];
                area += cross2(previous.cast<double>(), current.cast<double>());
                previous = current;
            }
        } else {
            Point previous = path.front();
            for (size_t idx = 1; idx < path.size(); ++idx) {
                const Point &current = path.get_point(idx);
                area += cross2(previous.cast<double>(), current.cast<double>());
                previous = current;
            }
        }
    }
    return area * 0.5;
}

} // namespace

TEST_CASE("Geometry readers preserve continuous units and inherited attribute scopes",
          "[plugins][native-readers]")
{
    const ExtrusionAttributes normal{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const ExtrusionAttributes wide{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.2, 1.f, 0.2f)};
    for (const std::string form : {"base", "path", "loop", "multipath", "collection"}) {
        CAPTURE(form);
        ExtrusionEntityUPtr root = reader_root(form, normal);
        ExtrusionEntityUPtr before = std::make_unique<ExtrusionEntity>(false);
        before->add_property(ExtrusionAttributes{ExtrusionRole::BridgeInfill, ExtrusionFlow(0., 100.f, 0.2f)});
        before->add_property(ExtrusionPropertyCustomGcodeText("M117 retained command scope"));
        const ExtrusionEntity *before_handle = before.get();
        root->append_child(std::move(before));
        ExtrusionEntityUPtr first = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        const ExtrusionEntity *first_handle = first.get();
        root->append_child(std::move(first));
        ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        nested->add_property(wide);
        ExtrusionEntityUPtr second = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {10000, 10000}}));
        const ExtrusionEntity *second_handle = second.get();
        nested->append_child(std::move(second));
        root->append_child(std::move(nested));
        ExtrusionEntityUPtr travel = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 10000}, {0, 10000}}));
        travel->add_property(ExtrusionAttributes{ExtrusionRole::Travel, ExtrusionFlow(0., 0.4f, 0.2f)});
        const ExtrusionEntity *travel_handle = travel.get();
        root->append_child(std::move(travel));
        ExtrusionEntityUPtr last = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 10000}, {0, 0}}));
        const ExtrusionEntity *last_handle = last.get();
        root->append_child(std::move(last));
        const uint64_t root_id = root->get_id();
        REQUIRE(root->ExtrusionEntity::is_loop());
        const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(*root);
        REQUIRE(units.size() == 1);
        CHECK(units[0].root == root.get());
        REQUIRE(units[0].leaves.size() == 4);
        CHECK(units[0].leaves[0].entity == first_handle);
        CHECK(units[0].leaves[1].entity == second_handle);
        CHECK(units[0].leaves[2].entity == travel_handle);
        CHECK(units[0].leaves[3].entity == last_handle);
        CHECK(units[0].leaves[0].attributes->width == normal.width);
        CHECK(units[0].leaves[1].attributes->width == wide.width);
        CHECK(units[0].leaves[2].attributes->extrusion_role() == ExtrusionRole::Travel);
        CHECK(units[0].leaves[3].attributes == root->get_property<ExtrusionAttributes>());
        const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
        REQUIRE(runs.size() == 1);
        CHECK(runs[0].unit_root == root.get());
        CHECK(extrusion_geometry_role(runs[0]) == ExtrusionRole::ExternalPerimeter);
        const ArcPolyline copied = extrusion_geometry_polyline(runs[0]);
        REQUIRE(copied.size() == 5);
        CHECK(copied.front() == Point(0, 0));
        CHECK(copied.back() == Point(0, 0));
        CHECK(root->get_id() == root_id);
        CHECK_FALSE(root->can_sort());
        CHECK_FALSE(root->can_reverse());
        CHECK_FALSE(root->can_rotate());
        CHECK(before_handle->custom_gcode_string(*before_handle->get_property<ExtrusionPropertyCustomGcode>()) ==
              "M117 retained command scope");
        CHECK(first_handle->polyline_ref().front() == Point(0, 0));
        CHECK(last_handle->polyline_ref().back() == Point(0, 0));
    }
}

TEST_CASE("Geometry readers do not join sortable or disconnected branches", "[plugins][native-readers]")
{
    for (const bool sortable : {false, true}) {
        for (const bool gap : {false, true}) {
            CAPTURE(sortable, gap);
            ExtrusionEntity root(ExtrusionEntity::Children(), sortable, false, false);
            root.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
            root.append_child(std::make_unique<ExtrusionEntity>(false,
                ArcPolyline(Points{{gap ? 20000 : 10000, 0}, {30000, 0}})));
            const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
            REQUIRE(units.size() == ((sortable || gap) ? 2 : 1));
            if (sortable || gap) {
                CHECK(units[0].root == &root.child(0));
                CHECK(units[1].root == &root.child(1));
                REQUIRE(units[0].leaves.size() == 1);
                REQUIRE(units[1].leaves.size() == 1);
            } else {
                CHECK(units[0].root == &root);
                REQUIRE(units[0].leaves.size() == 2);
            }
            CHECK(root.can_sort() == sortable);
        }
    }
    ExtrusionEntity command(false);
    command.add_property(ExtrusionPropertyCustomGcodeText("M117 no geometry"));
    CHECK(collect_extrusion_geometry_units(command).empty());
    ExtrusionEntity commands(ExtrusionEntity::Children(), false, false, true);
    commands.append_child(command);
    CHECK(collect_extrusion_geometry_units(commands).empty());
    REQUIRE(commands.child_count() == 1);
    CHECK(commands.child(0).has_properties());
    ExtrusionEntity point(false, ArcPolyline(Points{{1, 2}}));
    point.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(point);
    REQUIRE(units.size() == 1);
    REQUIRE(units[0].leaves.size() == 1);
    CHECK(units[0].leaves[0].polyline->size() == 1);
    CHECK(extrusion_geometry_runs(units[0]).empty());
    CHECK(point.polyline_ref().size() == 1);
}

TEST_CASE("Geometry analysis separates unsupported paths instead of borrowing a sibling role", "[plugins][native-readers]")
{
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    for (const coord_t x : {0, 10000, 20000}) {
        ExtrusionEntityUPtr leaf = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{x, 0}, {x + 10000, 0}}));
        if (x != 10000)
            leaf->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        root.append_child(std::move(leaf));
    }
    const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
    REQUIRE(units.size() == 1);
    REQUIRE(units[0].leaves.size() == 3);
    CHECK(units[0].leaves[1].attributes == nullptr);
    const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
    REQUIRE(runs.size() == 2);
    for (const ExtrusionGeometryRun &run : runs) {
        CHECK(run.unit_root == &root);
        REQUIRE(run.leaves.size() == 1);
        CHECK(extrusion_geometry_role(run) == ExtrusionRole::Perimeter);
        CHECK(extrusion_geometry_polyline(run).size() == 2);
    }
    CHECK(runs[0].leaves.back().polyline->back() == Point(10000, 0));
    CHECK(runs[1].leaves.front().polyline->front() == Point(20000, 0));
    CHECK_FALSE(root.child(1).has_properties());
}

TEST_CASE("Geometry analysis separates local Z jumps without changing XY identity", "[plugins][native-readers]")
{
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    for (size_t i = 0; i < 4; ++i) {
        ArcPolyline path(Points{{coord_t(i * 10000), 0}, {coord_t((i + 1) * 10000), 0}});
        if (i != 2) {
            path.set_z_offset(0, i == 1 ? 100 : 0);
            path.set_z_offset(1, i == 1 ? 100 : i == 3 ? 20 : 0);
        }
        root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(path)));
    }
    REQUIRE(root.is_continuous());
    const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
    REQUIRE(units.size() == 1);
    const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
    REQUIRE(runs.size() == 3);
    CHECK(runs[0].leaves.size() == 1);
    CHECK(runs[1].leaves.size() == 1);
    REQUIRE(runs[2].leaves.size() == 2);
    for (const ExtrusionGeometryRun &run : runs) CHECK(run.unit_root == &root);
    const ArcPolyline last = extrusion_geometry_polyline(runs[2]);
    REQUIRE(last.size() == 3);
    REQUIRE(last.has_z_offset());
    CHECK(last.z_offset(0) == 0);
    CHECK(last.z_offset(1) == 0);
    CHECK(last.z_offset(2) == 20);
    CHECK(root.child(1).polyline_ref().z_offset(0) == 100);
    CHECK_FALSE(root.child(2).polyline_ref().has_z_offset());
}

TEST_CASE("Geometry copies keep shared vertices arcs and local Z samples", "[plugins][native-readers]")
{
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    SECTION("rvalue append retains the shared vertex and short terminal segment") {
        root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
        root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {11000, 0}})));
        REQUIRE(root.child(0).polyline_ref().is_valid());
        REQUIRE(root.child(1).polyline_ref().is_valid());
        const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
        REQUIRE(units.size() == 1);
        const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
        REQUIRE(runs.size() == 1);
        const ArcPolyline copy = extrusion_geometry_polyline(runs[0]);
        REQUIRE(copy.size() == 3);
        CHECK(copy.is_valid());
        CHECK(copy.get_point(0) == Point(0, 0));
        CHECK(copy.get_point(1) == Point(10000, 0));
        CHECK(copy.get_point(2) == Point(11000, 0));
        CHECK(root.child(0).polyline_ref().size() == 2);
        CHECK(root.child(1).polyline_ref().size() == 2);
    }
    SECTION("arc and compatible Z joins are copied without modifying the source") {
        ArcPolyline arc;
        arc.append(Point(10000, 0));
        arc.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
        arc.set_z_offset(0, 10);
        arc.set_z_offset(1, 100);
        ArcPolyline line(Points{{0, 10000}, {-10000, 10000}});
        line.set_z_offset(0, 100);
        line.set_z_offset(1, 200);
        root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(arc)));
        root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(line)));
        const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
        REQUIRE(units.size() == 1);
        const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
        REQUIRE(runs.size() == 1);
        const ArcPolyline copy = extrusion_geometry_polyline(runs[0]);
        REQUIRE(copy.size() == 3);
        REQUIRE(copy.has_arc());
        CHECK(copy.get_arc(1).radius == root.child(0).polyline_ref().get_arc(1).radius);
        CHECK(copy.get_arc(1).orientation == Geometry::ArcWelder::Orientation::CCW);
        CHECK(copy.z_offset(0) == 10);
        CHECK(copy.z_offset(1) == 100);
        CHECK(copy.z_offset(2) == 200);
        CHECK(root.child(0).polyline_ref().size() == 2);
        CHECK(root.child(1).polyline_ref().size() == 2);
    }
}

TEST_CASE("Collision readers retain effective flow and local properties without changing sources", "[plugins][native-readers]")
{
    const ExtrusionAttributes normal{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const ExtrusionAttributes wide{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.2, 1.f, 0.2f)};
    for (const std::string form : {"base", "path", "loop", "multipath", "collection"}) {
        CAPTURE(form);
        ExtrusionEntityUPtr root = reader_root(form, normal);
        ExtrusionEntityUPtr first = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        first->add_property(ExtrusionPropertyCustomGcodeText("M117 local"));
        first->set_can_rotate(false);
        const ExtrusionEntity *first_handle = first.get();
        root->append_child(std::move(first));
        ExtrusionEntityUPtr second = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {20000, 0}}));
        second->add_property(wide);
        root->append_child(std::move(second));
        root->append_child(std::make_unique<ExtrusionEntity>(false));
        root->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{30000, 0}})));
        const ExtrusionEntityUPtr canonical = root->clone_canonical();
        for (const ExtrusionEntity *source : {root.get(), canonical.get()}) {
            REQUIRE(source->child(0).get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            REQUIRE(source->child(0).custom_gcode_string(*source->child(0).get_property<ExtrusionPropertyCustomGcode>()) == "M117 local");
            ExtrusionPaths paths;
            getExtrusionPathsFromEntity(*source, paths);
            REQUIRE(paths.size() == 2);
            CHECK(paths[0].polyline().front() == Point(0, 0));
            CHECK(paths[0].width() == normal.width);
            CHECK(paths[0].height() == normal.height);
            CHECK(paths[0].role() == normal.extrusion_role());
            CHECK_FALSE(paths[0].can_reverse());
            CHECK_FALSE(paths[0].can_rotate());
            REQUIRE(paths[0].get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(paths[0].custom_gcode_string(*paths[0].get_property<ExtrusionPropertyCustomGcode>()) == "M117 local");
            CHECK(paths[1].width() == wide.width);
            CHECK(paths[1].role() == wide.extrusion_role());
            CHECK(paths[0].get_id() == source->child(0).get_id());
            paths[0].add_property(ExtrusionPropertyCustomGcodeText("M117 copied value"));
            CHECK(source->child(0).custom_gcode_string(*source->child(0).get_property<ExtrusionPropertyCustomGcode>()) == "M117 local");
        }
        CHECK(&root->child(0) == first_handle);
        CHECK_FALSE(first_handle->get_property<ExtrusionAttributes>() != nullptr);
        REQUIRE(root->child_count() == 4);
    }
}

TEST_CASE("Travel obstacles select external segments and share outer unit identities with marking", "[plugins][native-readers]")
{
    ExtrusionEntity domain(ExtrusionEntity::Children(), true, false, false);
    domain.add_property(ExtrusionAttributes{ExtrusionRole::ExternalPerimeter});
    ExtrusionEntityUPtr unit = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    nested->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
    unit->append_child(std::move(nested));
    ExtrusionEntityUPtr interior = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {20000, 0}}));
    interior->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    unit->append_child(std::move(interior));
    ExtrusionEntityUPtr travel = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{20000, 0}, {30000, 0}}));
    travel->add_property(ExtrusionAttributes{ExtrusionRole::Travel});
    unit->append_child(std::move(travel));
    unit->append_child(std::make_unique<ExtrusionEntity>(false));
    unit->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{30000, 0}, {40000, 0}})));
    const ExtrusionEntity *unit_handle = unit.get();
    REQUIRE(unit->is_continuous());
    REQUIRE_FALSE(unit->can_sort());
    domain.append_child(std::move(unit));
    domain.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{50000, 0}, {60000, 0}})));
    REQUIRE(domain.can_sort());
    const std::vector<GCode::ObjectOrExtrusionLinef> lines = GCode::external_perimeter_lines(domain, Point(1000, 2000), 2, 3);
    REQUIRE(lines.size() == 3);
    CHECK(lines[0].extrusion_entity_id == unit_handle->get_id());
    CHECK(lines[1].extrusion_entity_id == unit_handle->get_id());
    CHECK(lines[2].extrusion_entity_id == domain.child(1).get_id());
    CHECK(lines[0].a == unscale_p(Point(1000, 2000)));
    CHECK(lines[0].b == unscale_p(Point(11000, 2000)));
    CHECK(lines[1].a == unscale_p(Point(31000, 2000)));
    GCode::TravelObstacleTracker tracker;
#ifdef _DEBUG
    for (const GCode::ObjectOrExtrusionLinef &line : lines)
        tracker.m_registered_extrusion.insert({line.object_layer_idx, line.instance_idx, line.extrusion_entity_id});
#endif
    const ExtrusionEntityUPtr descendant = unit_handle->child(0).clone_canonical();
    tracker.mark_extruded(domain, *descendant, 2, 3);
    CHECK_FALSE(tracker.is_extruded(lines[0]));
    const ExtrusionEntityUPtr emitted = unit_handle->clone_canonical();
    tracker.mark_extruded(domain, *emitted, 2, 3);
    CHECK(tracker.is_extruded(lines[0]));
    CHECK(tracker.is_extruded(lines[1]));
    CHECK_FALSE(tracker.is_extruded(lines[2]));
    ExtrusionEntity group(false);
    group.append_child(*emitted);
    group.append_child(domain.child(1));
    tracker.mark_extruded(domain, group, 2, 3);
    CHECK(tracker.is_extruded(lines[2]));
    CHECK(&domain.child(0) == unit_handle);
    CHECK_FALSE(domain.child(0).child(0).child(0).has_properties());
}

TEST_CASE("Support integrals use inherited widths and restore sibling scopes", "[plugins][native-readers]")
{
    using SupportSpotsGenerator::Integrals;
    using SupportSpotsGenerator::ObjectPart;
    const ExtrusionAttributes normal{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const ExtrusionAttributes wide{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.2, 1.f, 0.2f)};
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.add_property(normal);
    const Polyline first(Points{Point::new_scale(0., 0.), Point::new_scale(1., 0.)});
    const Polyline second(Points{Point::new_scale(1., 0.), Point::new_scale(2., 0.)});
    const Polyline last(Points{Point::new_scale(2., 0.), Point::new_scale(3., 0.)});
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(first)));
    ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    nested->add_property(wide);
    nested->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(second)));
    root.append_child(std::move(nested));
    ExtrusionEntityUPtr command = std::make_unique<ExtrusionEntity>(false);
    command->add_property(ExtrusionAttributes{ExtrusionRole::Travel, ExtrusionFlow(0., 100.f, 0.2f)});
    root.append_child(std::move(command));
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(last)));
    REQUIRE(root.is_continuous());
    const Integrals expected{{first, second, last}, {normal.width, wide.width, normal.width}};
    const ExtrusionEntityUPtr canonical = root.clone_canonical();
    for (const ExtrusionEntity *source : {static_cast<const ExtrusionEntity *>(&root),
                                         static_cast<const ExtrusionEntity *>(canonical.get())}) {
        const ObjectPart part({source}, true, 0.2, 0.2, std::nullopt);
        CHECK(part.volume == Approx(expected.area * 0.2f));
        CHECK(part.sticking_area == Approx(expected.area));
        CHECK(part.sticking_centroid_accumulator.x() == Approx(expected.x_i.x()));
        CHECK(part.sticking_centroid_accumulator.y() == Approx(expected.x_i.y()));
        CHECK(part.sticking_second_moment_of_area_accumulator.x() == Approx(expected.x_i_squared.x()));
        CHECK(part.sticking_second_moment_of_area_accumulator.y() == Approx(expected.x_i_squared.y()));
        CHECK(part.volume_centroid_accumulator.z() == Approx(part.volume * 0.1f));
    }
    const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
    REQUIRE(units.size() == 1);
    const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
    REQUIRE(runs.size() == 1);
    SupportSpotsGenerator::ExtrusionLine line(Vec2f::Zero(), Vec2f::Ones(), 1.f, &root, ExtrusionRole::ExternalPerimeter);
    CHECK(line.is_external_perimeter());
    CHECK(root.role() == ExtrusionRole::Perimeter);
    CHECK_FALSE(root.child(3).has_properties());
}

TEST_CASE("Solid-role scans retain direct child presence and terminal attribute rules",
          "[plugins][native-readers][native-preview-roles]")
{
    for (const std::string form : {"base", "collection"})
        for (size_t scenario = 0; scenario < 8; ++scenario) {
            CAPTURE(form, scenario);
            const ExtrusionRole parent_role = scenario < 5 ? ExtrusionRole::SolidInfill : ExtrusionRole::InternalInfill;
            ExtrusionEntityUPtr root = reader_root(form, ExtrusionAttributes{parent_role});
            const std::string text = std::string(180, 'p') + '-' + std::to_string(scenario);
            root->add_property(ExtrusionPropertyCustomGcodeText(text));
            const auto make_leaf = [](ExtrusionRole role, bool geometric) {
                ExtrusionEntityUPtr leaf = geometric ?
                    std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})) :
                    std::make_unique<ExtrusionEntity>(false);
                leaf->add_property(ExtrusionAttributes{role});
                return leaf;
            };
            if (scenario == 1)
                root->append_child(std::make_unique<ExtrusionEntity>(false));
            else if (scenario == 2) {
                ExtrusionEntityUPtr empty = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, false, false);
                empty->add_property(ExtrusionAttributes{ExtrusionRole::SolidInfill});
                root->append_child(std::move(empty));
            } else if (scenario == 3)
                root->append_child(make_leaf(ExtrusionRole::InternalInfill, true));
            else if (scenario == 4) {
                ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, false, false);
                nested->add_property(ExtrusionAttributes{ExtrusionRole::SolidInfill});
                nested->append_child(make_leaf(ExtrusionRole::InternalInfill, true));
                root->append_child(std::move(nested));
            } else if (scenario == 5) {
                root->append_child(make_leaf(ExtrusionRole::SolidInfill, false));
                root->append_child(make_leaf(ExtrusionRole::InternalInfill, true));
            } else if (scenario == 6)
                root->append_child(make_leaf(ExtrusionRole::SolidInfill, true));
            else if (scenario == 7) {
                root->append_child(make_leaf(ExtrusionRole::InternalInfill, true));
                root->append_child(make_leaf(ExtrusionRole::SolidInfill, true));
            }
            ExtrusionEntitiesPtr borrowed;
            for (const ExtrusionEntityUPtr &child : root->children())
                borrowed.push_back(child.get());
            Points before;
            root->collect_points(before);
            const uint64_t id = root->get_id();
            const bool sort = root->can_sort();
            const bool reverse = root->can_reverse();
            const bool rotate = root->can_rotate();
            const bool expected = scenario == 2 || scenario >= 5;
            CHECK((root->child_count() > 0) == (scenario != 0));
            CHECK((!borrowed.empty()) == (root->child_count() > 0));
            CHECK(HasRoleVisitor::search(borrowed, HasSolidInfillVisitor{}) == expected);
            CHECK((root->child_count() > 0 && HasRoleVisitor::search(*root, HasSolidInfillVisitor{})) == expected);
            if (form == "collection") {
                const ExtrusionEntityCollection &native = static_cast<const ExtrusionEntityCollection &>(*root);
                CHECK(native.entities() == borrowed);
                CHECK(HasRoleVisitor::search(native.entities(), HasSolidInfillVisitor{}) == expected);
            }
            if (scenario == 0)
                CHECK(HasRoleVisitor::search(*root, HasSolidInfillVisitor{}));
            if (scenario == 1 || scenario == 2)
                CHECK(root->ExtrusionEntity::empty());
            Points after;
            root->collect_points(after);
            CHECK(after == before);
            CHECK(root->get_id() == id);
            CHECK(root->can_sort() == sort);
            CHECK(root->can_reverse() == reverse);
            CHECK(root->can_rotate() == rotate);
            REQUIRE(root->get_property<ExtrusionAttributes>() != nullptr);
            CHECK(root->get_property<ExtrusionAttributes>()->extrusion_role() == parent_role);
            REQUIRE(root->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(root->custom_gcode_string(*root->get_property<ExtrusionPropertyCustomGcode>()) == text);
            REQUIRE(root->child_count() == borrowed.size());
            for (size_t index = 0; index < borrowed.size(); ++index)
                CHECK(&root->child(index) == borrowed[index]);
        }
}

TEST_CASE("Preview CPU meshes preserve dimensions destinations closure and one translation", "[plugins][native-readers]")
{
    const ExtrusionAttributes normal{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const ExtrusionAttributes wide{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.2, 1.f, 0.35f)};
    const Point shift(1000, 2000);
    for (const std::string form : {"base", "path", "loop", "multipath", "collection"}) {
        CAPTURE(form);
        ExtrusionEntityUPtr root = reader_root(form, normal);
        root->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
        ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        nested->add_property(wide);
        nested->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {10000, 10000}})));
        root->append_child(std::move(nested));
        ExtrusionEntityUPtr command = std::make_unique<ExtrusionEntity>(false);
        command->add_property(ExtrusionAttributes{ExtrusionRole::Travel, ExtrusionFlow(0., 100.f, 0.8f)});
        root->append_child(std::move(command));
        root->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 10000}, {0, 0}})));
        REQUIRE(root->is_continuous());
        const uint64_t id = root->get_id();
        const std::vector<ExtrusionGeometryMesh> plain = project_extrusion_geometry(*root, shift);
        REQUIRE(plain.size() == 1);
        REQUIRE(plain[0].lines.size() == 3);
        CHECK(plain[0].closed);
        CHECK(plain[0].destination == 0);
        CHECK(plain[0].widths == std::vector<double>({normal.width, wide.width, normal.width}));
        CHECK(plain[0].heights == std::vector<double>({normal.height, wide.height, normal.height}));
        CHECK(plain[0].lines[0].a == Point(1000, 2000));
        CHECK(plain[0].lines[0].b == Point(11000, 2000));
        CHECK(plain[0].lines[2].b == Point(1000, 2000));
        const std::map<GCodeExtrusionRole, size_t> separate{{GCodeExtrusionRole::Perimeter, 1}, {GCodeExtrusionRole::ExternalPerimeter, 2}};
        const std::vector<ExtrusionGeometryMesh> colored = project_extrusion_geometry(*root, shift, separate);
        REQUIRE(colored.size() == 3);
        for (size_t i = 0; i < colored.size(); ++i) {
            REQUIRE(colored[i].lines.size() == 1);
            CHECK_FALSE(colored[i].closed);
            CHECK(colored[i].destination == (i == 1 ? 2 : 1));
            CHECK(colored[i].widths[0] == plain[0].widths[i]);
            CHECK(colored[i].heights[0] == plain[0].heights[i]);
            CHECK(colored[i].lines[0].a == plain[0].lines[i].a);
            CHECK(colored[i].lines[0].b == plain[0].lines[i].b);
        }
        const std::map<GCodeExtrusionRole, size_t> shared{{GCodeExtrusionRole::Perimeter, 1}, {GCodeExtrusionRole::ExternalPerimeter, 1}};
        const std::vector<ExtrusionGeometryMesh> joined = project_extrusion_geometry(*root, shift, shared);
        REQUIRE(joined.size() == 1);
        CHECK(joined[0].closed);
        CHECK(joined[0].destination == 1);
        REQUIRE(joined[0].lines.size() == 3);
        CHECK(root->get_id() == id);
        CHECK(root->child(0).polyline_ref().front() == Point(0, 0));
        CHECK_FALSE(root->child(3).has_properties());
        CHECK_FALSE(root->can_sort());
        CHECK_FALSE(root->can_reverse());
        CHECK_FALSE(root->can_rotate());
    }
}

TEST_CASE("Preview CPU meshes keep unsupported paths and Z discontinuities as barriers", "[plugins][native-readers]")
{
    SECTION("unsupported paths split meshes even when both neighboring XY joins agree") {
        for (const bool point_only : {false, true}) {
            CAPTURE(point_only);
            ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
            ExtrusionEntityUPtr first = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
            first->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::move(first));
            ExtrusionEntityUPtr barrier = std::make_unique<ExtrusionEntity>(false,
                ArcPolyline(point_only ? Points{{10000, 0}} : Points{{10000, 0}, {10000, 10000}, {10000, 0}}));
            if (point_only)
                barrier->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::move(barrier));
            ExtrusionEntityUPtr last = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {20000, 0}}));
            last->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::move(last));
            REQUIRE(root.is_continuous());
            const std::vector<ExtrusionGeometryUnit> units = collect_extrusion_geometry_units(root);
            REQUIRE(units.size() == 1);
            REQUIRE(units[0].leaves.size() == 3);
            CHECK((units[0].leaves[1].attributes != nullptr) == point_only);
            const std::vector<ExtrusionGeometryRun> runs = extrusion_geometry_runs(units[0]);
            REQUIRE(runs.size() == 2);
            const std::vector<ExtrusionGeometryMesh> meshes = project_extrusion_geometry(root, Point(0, 0));
            REQUIRE(meshes.size() == 2);
            REQUIRE(meshes[0].lines.size() == 1);
            REQUIRE(meshes[1].lines.size() == 1);
            CHECK(meshes[0].lines.back().b == meshes[1].lines.front().a);
            CHECK(meshes[0].lines.back().b == Point(10000, 0));
            CHECK(root.child(1).polyline_ref().size() == (point_only ? 1 : 3));
        }
    }
    SECTION("a missing attribute or point-only leaf never reconnects its neighbors") {
        for (const bool point_only : {false, true}) {
            CAPTURE(point_only);
            ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
            ExtrusionEntityUPtr first = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
            first->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::move(first));
            root.append_child(std::make_unique<ExtrusionEntity>(false,
                ArcPolyline(point_only ? Points{{10000, 0}} : Points{{10000, 0}, {20000, 0}})));
            ExtrusionEntityUPtr last = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{20000, 0}, {30000, 0}}));
            last->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            root.append_child(std::move(last));
            const std::vector<ExtrusionGeometryMesh> meshes = project_extrusion_geometry(root, Point(0, 0));
            REQUIRE(meshes.size() == 2);
            REQUIRE(meshes[0].lines.size() == 1);
            REQUIRE(meshes[1].lines.size() == 1);
            CHECK(meshes[0].lines.back().b == Point(10000, 0));
            CHECK(meshes[1].lines.front().a == Point(20000, 0));
            CHECK_FALSE(meshes[0].closed);
            CHECK_FALSE(meshes[1].closed);
            CHECK(root.child(1).polyline_ref().size() == (point_only ? 1 : 2));
            CHECK_FALSE(root.child(1).has_properties());
        }
    }
    SECTION("an exact XY join with a Z jump stays in two meshes") {
        ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
        root.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        ArcPolyline first(Points{{0, 0}, {10000, 0}});
        first.set_z_offset(0, 0);
        first.set_z_offset(1, 10);
        ArcPolyline second(Points{{10000, 0}, {20000, 0}});
        second.set_z_offset(0, 20);
        second.set_z_offset(1, 20);
        root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(first)));
        root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(second)));
        REQUIRE(root.is_continuous());
        const std::vector<ExtrusionGeometryMesh> meshes = project_extrusion_geometry(root, Point(0, 0));
        REQUIRE(meshes.size() == 2);
        CHECK(meshes[0].lines[0].b == meshes[1].lines[0].a);
        CHECK(root.child(0).polyline_ref().z_offset(1) == 10);
        CHECK(root.child(1).polyline_ref().z_offset(0) == 20);
    }
    SECTION("the name Loop does not force a nearly closed mesh shut") {
        ExtrusionEntityUPtr root = reader_root("loop", ExtrusionAttributes{ExtrusionRole::Perimeter});
        root->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}, {0, 1}})));
        REQUIRE(root->is_loop());
        REQUIRE_FALSE(root->ExtrusionEntity::is_loop());
        const std::vector<ExtrusionGeometryMesh> meshes = project_extrusion_geometry(*root, Point(0, 0));
        REQUIRE(meshes.size() == 1);
        CHECK_FALSE(meshes[0].closed);
        CHECK(meshes[0].lines.back().b == Point(0, 1));
    }
}

TEST_CASE("Contour area preserves independent segment sums without forcing closure", "[plugins][native-readers][native-arachne]")
{
    const Points rectangle{{10000, 20000}, {40000, 20000}, {40000, 60000}, {10000, 60000}, {10000, 20000}};
    ArcPolylines paths;
    for (size_t idx = 1; idx < rectangle.size(); ++idx)
        paths.emplace_back(Points{rectangle[idx - 1], rectangle[idx]});
    double expected = 1200000000.;
    SECTION("offset multisegment rectangle has an exact signed area") {}
    SECTION("inverse order keeps the same exact magnitude") {
        std::reverse(paths.begin(), paths.end());
        for (ArcPolyline &path : paths)
            path.reverse();
        expected = -1200000000.;
    }
    SECTION("an epsilon-close offset endpoint contributes no added closing edge") {
        paths.back().set_back(Point(10000, 20001));
        expected = 1200005000.;
    }
    SECTION("arc tessellation follows the independent segment reference") {
        ArcPolyline arc;
        arc.append(Point(10000, 0));
        arc.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
        arc.set_z_offset(0, 10);
        arc.set_z_offset(1, 20);
        paths.clear();
        paths.emplace_back(std::move(arc));
        paths.emplace_back(Points{{0, 10000}, {0, 0}, {10000, 0}});
        expected = reference_contour_area(paths);
        REQUIRE(expected > 0.);
    }
    SECTION("zero signed area keeps the clockwise predicate unchanged") {
        paths.clear();
        paths.emplace_back(Points{{10000, 20000}, {40000, 20000}});
        paths.emplace_back(Points{{40000, 20000}, {10000, 20000}});
        expected = 0.;
    }
    const double reference = reference_contour_area(paths);
    REQUIRE(reference == expected);
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    ExtrusionPaths native_paths;
    for (const ArcPolyline &path : paths) {
        root.append_child(std::make_unique<ExtrusionEntity>(false, path));
        native_paths.emplace_back(path, ExtrusionAttributes{ExtrusionRole::Perimeter}, nullptr, false);
    }
    const ExtrusionLoop native(native_paths);
    const uint64_t id = root.get_id();
    CHECK(extrusion_geometry_area(root) == expected);
    CHECK(native.area() == reference);
    CHECK(native.is_clockwise() == !(reference > 0.));
    CHECK(root.get_id() == id);
    REQUIRE(root.child_count() == paths.size());
    for (size_t idx = 0; idx < paths.size(); ++idx) {
        const ArcPolyline &stored = root.child(idx).polyline_ref();
        REQUIRE(stored.size() == paths[idx].size());
        CHECK_FALSE(root.child(idx).has_properties());
        CHECK(stored.has_z_offset() == paths[idx].has_z_offset());
        for (size_t point_idx = 0; point_idx < stored.size(); ++point_idx) {
            CHECK(stored.get_point(point_idx) == paths[idx].get_point(point_idx));
            CHECK(stored.get_arc(point_idx).radius == paths[idx].get_arc(point_idx).radius);
            CHECK(stored.get_arc(point_idx).orientation == paths[idx].get_arc(point_idx).orientation);
            if (stored.has_z_offset())
                CHECK(stored.z_offset(point_idx) == paths[idx].z_offset(point_idx));
        }
    }
}
