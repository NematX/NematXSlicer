/* Extrusion-tree tests check ordered continuity, wrapper simplification and collection access.
   Concurrent collection readers share immutable owned children and compare const accessors against those children.
   Freshly appended collections exercise first-read pointer-cache construction; assertions run only after workers join.
   Hybrid trees retain a legacy C++ dynamic type after generic mutations replace its content with ordered children.
   Content snapshots read const local polylines and stored children, ignoring geometrically empty event nodes.

       TEST_CASE
           |-- hybrid_square() / hybrid_wrap(): build or mutate owned legacy views
           |-- hybrid_geometry(): snapshot stored polylines without legacy accessors
           `-- hybrid_check_geometry(): compare every point, arc and Z offset

   C handles borrow those owned nodes. Geometry queries must not alter resources,
   dynamic types or child identities; allocation and assertion failures stay visible. */
#include <catch2/catch.hpp>

#include <atomic>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/Flow.hpp"

namespace {

// Checks const collection views against its owned children without changing the collection.
bool collection_reads_match(const Slic3r::ExtrusionEntityCollection &collection);

// Borrows a nonempty collection of nonempty paths and compares all const views with its immutable owned children;
// returns false for a size, pointer, iteration or endpoint mismatch, owns nothing and propagates accessor failures.
bool collection_reads_match(const Slic3r::ExtrusionEntityCollection &collection)
{
    const Slic3r::ExtrusionEntity::Children &children = collection.children();
    const Slic3r::ExtrusionEntitiesPtr &entities = collection.entities();
    if (entities.size() != children.size()) return false;
    for (size_t idx = 0; idx < children.size(); ++idx)
        if (entities[idx] != children[idx].get()) return false;
    if (collection.size() != children.size() || collection.empty()) return false;
    Slic3r::ExtrusionEntitiesPtr::const_iterator it = collection.begin();
    const Slic3r::ExtrusionEntitiesPtr::const_iterator end = collection.end();
    for (size_t idx = 0; idx < children.size(); ++idx, ++it) {
        if (it == end || *it != children[idx].get()) return false;
    }
    if (it != end) return false;
    Slic3r::ExtrusionEntitiesPtr::const_iterator cit = collection.cbegin();
    const Slic3r::ExtrusionEntitiesPtr::const_iterator cend = collection.cend();
    for (size_t idx = 0; idx < children.size(); ++idx, ++cit) {
        if (cit == cend || *cit != children[idx].get()) return false;
    }
    return cit == cend && collection.first_point() == children.front()->first_point() &&
        collection.last_point() == children.back()->last_point();
}

Slic3r::ExtrusionPath path_between(const Slic3r::Point &from, const Slic3r::Point &to)
{
    Slic3r::ExtrusionPath path(
        Slic3r::ExtrusionAttributes(
            Slic3r::ExtrusionRole::Perimeter,
            Slic3r::ExtrusionFlow(0.1, 0.4f, 0.2f)),
        nullptr,
        true);
    path.polyline().append(from);
    path.polyline().append(to);
    return path;
}

} // namespace

namespace {
using namespace Slic3r;

// Borrow a mutable entity as a C handle; ownership and lifetime remain with the caller.
extrusion_entity_handle *hybrid_handle(ExtrusionEntity &entity)
{
    return reinterpret_cast<extrusion_entity_handle *>(&entity);
}

// Borrow immutable content, copying each local polyline in stored order without virtual geometry queries;
// empty events contribute no polyline, ownership stays with the caller and allocation failures propagate.
void hybrid_geometry(const ExtrusionEntity &entity, std::vector<ArcPolyline> &out)
{
    if (const ArcPolyline *polyline = entity.polyline_or_null()) {
        if (!polyline->empty()) out.push_back(*polyline);
    } else if (!entity.is_leaf()) {
        for (const ExtrusionEntityUPtr &child : entity.children()) {
            REQUIRE(child != nullptr);
            hybrid_geometry(*child, out);
        }
    }
}

// Compare borrowed content against an owned snapshot, including every arc and Z offset; require sizes
// before indexing, mutate neither input and report differences through Catch assertions.
void hybrid_check_geometry(const ExtrusionEntity &entity, const std::vector<ArcPolyline> &expected)
{
    std::vector<ArcPolyline> actual;
    hybrid_geometry(entity, actual);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        REQUIRE(actual[i].size() == expected[i].size());
        CHECK(actual[i].has_z_offset() == expected[i].has_z_offset());
        for (size_t j = 0; j < actual[i].size(); ++j) {
            CHECK(actual[i].get_point(j) == expected[i].get_point(j));
            CHECK(actual[i].z_offset(j) == expected[i].z_offset(j));
            CHECK(actual[i].get_arc(j).radius == expected[i].get_arc(j).radius);
            CHECK(actual[i].get_arc(j).orientation == expected[i].get_arc(j).orientation);
        }
    }
}

// Build an owned legacy square with one closed Path; positive scaled coordinates avoid sentinel endpoints,
// direct speed belongs to the Loop and construction/allocation failures propagate.
std::unique_ptr<ExtrusionLoop> hybrid_square()
{
    ExtrusionPath path = path_between(Point(1000, 1000), Point(11000, 1000));
    path.polyline().append(Point(11000, 11000));
    path.polyline().append(Point(1000, 11000));
    path.polyline().append(Point(1000, 1000));
    auto loop = std::make_unique<ExtrusionLoop>(std::move(path));
    loop->add_property(ExtrusionPropertySpeed(23.f));
    loop->add_property(ExtrusionPropertyCustomGcodeText("M117 preserved contour"));
    return loop;
}

// Wrap borrowed content through the real ordered-leaf mutation, preserving the outer address and old child
// addresses; add a resource-owning empty event, require success and propagate allocation failures.
void hybrid_wrap(ExtrusionEntity &entity)
{
    ExtrusionEntity *address = &entity;
    const ExtrusionEntity &before = entity;
    const ExtrusionEntity *old_child = before.child_count() ? &before.child(0) : nullptr;
    const auto *old_speed = before.get_property<ExtrusionPropertySpeed>();
    const bool had_speed = old_speed != nullptr;
    const float speed = had_speed ? old_speed->speed_mm_per_s : 0.f;
    const auto *old_text = before.get_property<ExtrusionPropertyCustomGcode>();
    const std::string preserved_text = old_text != nullptr ? before.custom_gcode_string(*old_text) : std::string();
    ExtrusionEntity *event = entity.emplace_ordered_leaf(
        OrderedLeafPosition::Before, ExistingPropertyPlacement::MoveWithExistingContent);
    REQUIRE(event != nullptr);
    event->add_property(ExtrusionPropertyCustomGcodeText("M117 empty transition"));
    const ExtrusionEntity &after = entity;
    CHECK(&after == address);
    REQUIRE(after.child_count() == 2);
    if (old_child != nullptr) CHECK(&after.child(1).child(0) == old_child);
    if (had_speed) {
        CHECK(after.get_property<ExtrusionPropertySpeed>() == nullptr);
        REQUIRE(after.child(1).get_property<ExtrusionPropertySpeed>() != nullptr);
        CHECK(after.child(1).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == speed);
    }
    if (old_text != nullptr) {
        CHECK(after.get_property<ExtrusionPropertyCustomGcode>() == nullptr);
        const auto *moved_text = after.child(1).get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(moved_text != nullptr);
        CHECK(after.child(1).custom_gcode_string(*moved_text) == preserved_text);
    }
    const auto *text = after.child(0).get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(text != nullptr);
    CHECK(after.child(0).custom_gcode_string(*text) == "M117 empty transition");
}
} // namespace

// Own a generic parent and legacy Loop, wrap the Loop in place and compare const geometry before querying
// the parent through C; failures remain visible as assertions and no child ownership escapes the test.
TEST_CASE("Generic parent queries a wrapped legacy Loop by content",
          "[plugins][extrusion][extrusion-hybrid][hybrid-generic-loop]")
{
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    auto loop = hybrid_square();
    ExtrusionEntity *stable = loop.get();
    parent.append_child(std::move(loop));
    std::vector<ArcPolyline> expected;
    hybrid_geometry(parent, expected);
    hybrid_wrap(*stable);
    hybrid_check_geometry(parent, expected);
    const ExtrusionEntity &view = parent;
    CHECK(&view.child(0) == stable);
    CHECK(extrusion_is_loop(hybrid_handle(parent)) == 1);
    CHECK(view.first_point() == Point(1000, 1000));
    CHECK(view.last_point() == Point(1000, 1000));
    CHECK_FALSE(view.empty());
}

// Own a generic-to-Collection-to-Loop chain and wrap only the deepest node; const snapshots retain geometry
// and the C query on the Collection must not enter the Loop path view.
TEST_CASE("Collection parent queries a wrapped legacy Loop by content",
          "[plugins][extrusion][extrusion-hybrid][hybrid-collection-loop]")
{
    auto collection = std::make_unique<ExtrusionEntityCollection>(false, false);
    auto loop = hybrid_square();
    ExtrusionEntity *stable = loop.get();
    collection->append(std::move(loop));
    ExtrusionEntityCollection *collection_address = collection.get();
    ExtrusionEntity outer(ExtrusionEntity::Children(), false, false, false);
    outer.append_child(std::move(collection));
    std::vector<ArcPolyline> expected;
    hybrid_geometry(outer, expected);
    hybrid_wrap(*stable);
    hybrid_check_geometry(outer, expected);
    CHECK(extrusion_is_loop(hybrid_handle(*collection_address)) == 1);
    CHECK(extrusion_is_loop(hybrid_handle(outer)) == 1);
}

// Own a sortable Collection whose first nonempty child is a wrapped Loop; reading flags must neither
// traverse legacy paths nor alter stored geometry, event resources or effective permissions.
TEST_CASE("Sortable Collection flags tolerate a hybrid first geometric child",
          "[plugins][extrusion][extrusion-hybrid][hybrid-collection-flags]")
{
    ExtrusionEntityCollection parent;
    auto loop = hybrid_square();
    ExtrusionEntity *stable = loop.get();
    parent.append(std::move(loop));
    hybrid_wrap(*stable);
    std::vector<ArcPolyline> expected;
    hybrid_geometry(parent, expected);
    CHECK(extrusion_flags(hybrid_handle(parent)) ==
          (RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE));
    hybrid_check_geometry(parent, expected);
}

// Place a nonempty sibling before a wrapped Loop with a matching join; querying continuity exercises
// both recursive emptiness and endpoint comparisons without using either query as the geometry oracle.
TEST_CASE("Continuity reaches a hybrid Loop after a nonempty sibling",
          "[plugins][extrusion][extrusion-hybrid][hybrid-following-loop]")
{
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    parent.append_child(std::make_unique<ExtrusionPath>(path_between(Point(100, 1000), Point(1000, 1000))));
    auto loop = hybrid_square();
    ExtrusionEntity *stable = loop.get();
    parent.append_child(std::move(loop));
    hybrid_wrap(*stable);
    CHECK(extrusion_is_continuous(hybrid_handle(parent)) == 1);
    CHECK(extrusion_is_loop(hybrid_handle(parent)) == 0);
}

// Split a real Path below a legacy sequence while retaining its address, then query only its generic
// parent; the two sections independently cover Loop and MultiPath with exact stored split endpoints.
TEST_CASE("Generic parent queries a legacy sequence after a real Path split",
          "[plugins][extrusion][extrusion-hybrid][hybrid-split-path]")
{
    std::unique_ptr<ExtrusionEntity> sequence;
    bool closed = false;
    SECTION("Loop") { sequence = hybrid_square(); closed = true; }
    SECTION("MultiPath") {
        ExtrusionPaths paths;
        paths.push_back(path_between(Point(1000, 1000), Point(11000, 1000)));
        sequence = std::make_unique<ExtrusionMultiPath>(std::move(paths));
    }
    REQUIRE(sequence != nullptr);
    ExtrusionEntity *path = &sequence->child(0);
    path->add_property(ExtrusionPropertySpeed(41.f));
    const ExtrusionEntity &source_path = *path;
    const Points expected_points = source_path.polyline_ref().to_polyline().points;
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    parent.append_child(std::move(sequence));
    // Cut the first edge at (6000, 1000). The wedge below y = x keeps the closing edge of the Loop outside the first area.
    const ExPolygons start_area{ExPolygon({Point(500, 500), Point(6000, 500), Point(6000, 1500), Point(1500, 1500)})};
    ExPolygon rest({Point(-100000, -100000), Point(100000, -100000), Point(100000, 100000), Point(-100000, 100000)});
    rest.holes.push_back(start_area.front().contour);
    rest.holes.back().reverse();
    const ExPolygons rest_area{rest};
    const expolygon_collection_handle *areas[] = {reinterpret_cast<const expolygon_collection_handle *>(&start_area),
                                                   reinterpret_cast<const expolygon_collection_handle *>(&rest_area)};
    REQUIRE(extrusion_split_leaf_by_areas(hybrid_handle(*path), areas, 2, 0, nullptr, nullptr) ==
            RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
    const ExtrusionEntity &view = parent;
    CHECK(&view.child(0).child(0) == path);
    REQUIRE(view.child(0).child(0).child_count() == 2);
    CHECK(view.child(0).child(0).child(0).polyline_ref().back() == Point(6000, 1000));
    CHECK(view.child(0).child(0).child(1).polyline_ref().front() == Point(6000, 1000));
    std::vector<ArcPolyline> fragments;
    hybrid_geometry(view, fragments);
    REQUIRE(fragments.size() == 2);
    Points actual_points;
    for (const ArcPolyline &fragment : fragments)
        for (size_t i = 0; i < fragment.size(); ++i)
            if (actual_points.empty() || actual_points.back() != fragment.get_point(i))
                actual_points.push_back(fragment.get_point(i));
    Points expected_split_points = expected_points;
    expected_split_points.insert(expected_split_points.begin() + 1, Point(6000, 1000));
    CHECK(actual_points == expected_split_points);
    for (const ExtrusionEntityUPtr &fragment : view.child(0).child(0).children()) {
        REQUIRE(fragment->get_property<ExtrusionPropertySpeed>() != nullptr);
        CHECK(fragment->get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 41.f);
    }
    CHECK(extrusion_is_continuous(hybrid_handle(parent)) == 1);
    CHECK(extrusion_is_loop(hybrid_handle(parent)) == int(closed));
}

// Follow an owned hybrid Loop with a nonempty sibling joined at its actual stored exit; compare const
// geometry and stable handles, then query parent continuity so the preceding child's last point is required.
TEST_CASE("Continuity reads the stored exit of a preceding hybrid Loop",
          "[plugins][extrusion][extrusion-hybrid][hybrid-previous-loop]")
{
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    auto loop = hybrid_square();
    ExtrusionEntity *loop_address = loop.get();
    auto following = std::make_unique<ExtrusionPath>(path_between(Point(1000, 1000), Point(21000, 1000)));
    ExtrusionEntity *following_address = following.get();
    parent.append_child(std::move(loop));
    parent.append_child(std::move(following));
    std::vector<ArcPolyline> expected;
    hybrid_geometry(parent, expected);
    hybrid_wrap(*loop_address);
    hybrid_check_geometry(parent, expected);
    const ExtrusionEntity &view = parent;
    CHECK(&view.child(0) == loop_address);
    CHECK(&view.child(1) == following_address);
    CHECK(extrusion_is_continuous(hybrid_handle(parent)) == 1);
    CHECK(extrusion_is_loop(hybrid_handle(parent)) == 0);
}

// Compare identical owned content and raw permissions under legacy and generic roots; classification
// is geometric through the C head too; SDK flag rewrites retain the read REVERSIBLE bit.
TEST_CASE("Sortable legacy Loop follows generic classification and flag rewrites",
          "[plugins][extrusion][extrusion-hybrid][hybrid-loop-flags]")
{
    auto loop = hybrid_square();
    ExtrusionEntity generic(*loop);
    REQUIRE(extrusion_flags(hybrid_handle(*loop)) == 0);
    REQUIRE(extrusion_flags(hybrid_handle(generic)) == 0);
    for (ExtrusionEntity *entity : {static_cast<ExtrusionEntity *>(loop.get()), &generic}) {
        REQUIRE(extrusion_set_flags(hybrid_handle(*entity), RAW_EXTRUSION_FLAG_SORTABLE) == 1);
        CHECK(entity->is_collection());
        CHECK_FALSE(entity->is_continuous());
        // The legacy Loop formerly read 0 here; generic content already read both bits.
        CHECK(extrusion_flags(hybrid_handle(*entity)) == 3);
    }
    CHECK(loop->is_loop()); // The legacy C++ override remains unchanged, but no longer governs the C API.
    CHECK(extrusion_is_loop(hybrid_handle(*loop)) == 0);
    CHECK(extrusion_is_loop(hybrid_handle(generic)) == 0);
    CHECK(loop->can_sort());
    CHECK(extrusion_is_continuous(hybrid_handle(*loop)) == 0);
    for (ExtrusionEntity *entity : {static_cast<ExtrusionEntity *>(loop.get()), &generic}) {
        slic3r_api::MutableExtrusionEntity(hybrid_handle(*entity)).disable_sort();
        CHECK_FALSE(entity->can_sort());
        CHECK(entity->is_continuous());
        CHECK_FALSE(entity->is_collection());
        CHECK(extrusion_flags(hybrid_handle(*entity)) == RAW_EXTRUSION_FLAG_REVERSIBLE);
        CHECK(extrusion_is_loop(hybrid_handle(*entity)) == 1);
    }
}

// Mutate owned legacy Loops through legal generic APIs without invoking their legacy endpoint views;
// clearing then inserting enables sorting, while a separated second path breaks ordered continuity.
TEST_CASE("Cleared and discontinuous legacy Loops use structural collection classification",
          "[plugins][extrusion][extrusion-hybrid][hybrid-loop-mutations]")
{
    SECTION("clear then insert") {
        auto loop = hybrid_square();
        REQUIRE(extrusion_clear_content(hybrid_handle(*loop)) == 1);
        auto path = path_between(Point(1000, 1000), Point(11000, 1000));
        REQUIRE(extrusion_insert_child_copy(hybrid_handle(*loop), 0, hybrid_handle(path)) == 0);
        CHECK(loop->is_collection());
        CHECK(extrusion_flags(hybrid_handle(*loop)) == 3);
        CHECK(loop->is_loop());
        CHECK_FALSE(loop->ExtrusionEntity::is_loop());
        CHECK(extrusion_is_loop(hybrid_handle(*loop)) == 0);
    }
    SECTION("gap between paths") {
        auto loop = hybrid_square();
        loop->append_child(std::make_unique<ExtrusionPath>(path_between(Point(21000, 1000), Point(31000, 1000))));
        CHECK_FALSE(loop->is_continuous());
        CHECK(loop->is_collection());
        CHECK(extrusion_flags(hybrid_handle(*loop)) == 0);
        CHECK(loop->is_loop());
        CHECK_FALSE(loop->ExtrusionEntity::is_loop());
        CHECK(extrusion_is_loop(hybrid_handle(*loop)) == 0);
    }
}

// Keep empty event resources and child order while testing Collection closure against actual positive
// coordinates; historical results were false for a leading event and true for two empty boundary phases.
TEST_CASE("Collection loop queries ignore empty events rather than comparing sentinels",
          "[plugins][extrusion][extrusion-hybrid][hybrid-collection-endpoints]")
{
    SECTION("leading event then closed Loop") {
        ExtrusionEntityCollection collection(false, false);
        auto event = std::make_unique<ExtrusionEntity>(false);
        event->add_property(ExtrusionPropertyCustomGcodeText("M117 before closed loop"));
        const ExtrusionEntity *event_address = event.get();
        collection.append(std::move(event));
        collection.append(hybrid_square());
        CHECK(extrusion_is_loop(hybrid_handle(collection)) == 1);
        const ExtrusionEntity &view = collection;
        CHECK(&view.child(0) == event_address);
        CHECK(view.child_count() == 2);
        const auto *text = view.child(0).get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(text != nullptr);
        CHECK(view.child(0).custom_gcode_string(*text) == "M117 before closed loop");
    }
    SECTION("open Collection with empty incoming and outgoing phases") {
        ExtrusionEntityCollection collection(false, false);
        collection.append(path_between(Point(1000, 1000), Point(11000, 1000)));
        hybrid_wrap(collection);
        ExtrusionEntity *after = collection.emplace_ordered_leaf(
            OrderedLeafPosition::After, ExistingPropertyPlacement::KeepOnParent);
        REQUIRE(after != nullptr);
        after->add_property(ExtrusionPropertySpeed(17.f));
        CHECK(extrusion_is_loop(hybrid_handle(collection)) == 0);
        CHECK(collection.is_continuous());
        const ExtrusionEntity &view = collection;
        REQUIRE(view.child_count() == 3);
        REQUIRE(view.child(2).get_property<ExtrusionPropertySpeed>() != nullptr);
        CHECK(view.child(2).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 17.f);
    }
}

// Own stationary, arc/Z and travel content with empty positioned commands; query exact XY closure and
// preserve every copied segment, Z offset, resource and legacy top-level boolean without geometric repair.
TEST_CASE("Content queries preserve stationary points arcs Z travel and legacy heads",
          "[plugins][extrusion][extrusion-hybrid][hybrid-content-cases]")
{
    SECTION("one point and positioned monostate") {
        ArcPolyline point;
        point.append(Point(1000, 2000));
        ExtrusionEntity root(false, std::move(point));
        CHECK(extrusion_is_loop(hybrid_handle(root)) == 1);
        auto nop = std::make_unique<ExtrusionNop>();
        nop->position = Point(9000, 9000);
        nop->add_property(ExtrusionPropertySpeed(11.f));
        ExtrusionNop *stable = nop.get();
        ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
        parent.append_child(std::move(nop));
        CHECK(parent.empty());
        CHECK(extrusion_is_loop(hybrid_handle(parent)) == 0);
        CHECK(stable->position == Point(9000, 9000));
        CHECK(stable->get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 11.f);
    }
    SECTION("arc with unequal endpoint Z but equal XY") {
        ArcPolyline polyline;
        polyline.append(Point(10000, 0));
        polyline.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
        polyline.append(Point(10000, 0));
        polyline.set_z_offset(0, 100);
        polyline.set_z_offset(1, 300);
        polyline.set_z_offset(2, 500);
        ExtrusionEntity root(false, std::move(polyline));
        std::vector<ArcPolyline> expected;
        hybrid_geometry(root, expected);
        hybrid_wrap(root);
        CHECK(extrusion_is_loop(hybrid_handle(root)) == 1);
        hybrid_check_geometry(root, expected);
    }
    SECTION("materialized travel changes the actual entry") {
        auto loop = hybrid_square();
        ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
        auto travel = std::make_unique<ExtrusionPath>(path_between(Point(100, 1000), Point(1000, 1000)));
        travel->get_property<ExtrusionAttributes>()->set_role(ExtrusionRole::Travel);
        parent.append_child(std::move(travel));
        parent.append_child(std::move(loop));
        CHECK(parent.is_continuous());
        CHECK_FALSE(parent.is_loop());
    }
    SECTION("intact legacy heads") {
        auto loop = hybrid_square();
        ExtrusionPaths closed_paths;
        closed_paths.push_back(path_between(Point(1000, 1000), Point(11000, 1000)));
        closed_paths.push_back(path_between(Point(11000, 1000), Point(1000, 1000)));
        ExtrusionMultiPath closed(std::move(closed_paths));
        ExtrusionPaths open_paths;
        open_paths.push_back(path_between(Point(1000, 1000), Point(11000, 1000)));
        ExtrusionMultiPath open(std::move(open_paths));
        CHECK(extrusion_is_loop(hybrid_handle(*loop)) == 1);
        CHECK_FALSE(closed.is_loop());
        CHECK(extrusion_is_loop(hybrid_handle(closed)) == 1);
        CHECK(extrusion_can_rotate(hybrid_handle(closed)) == 0);
        CHECK(extrusion_is_loop(hybrid_handle(open)) == 0);
        CHECK(closed.ExtrusionEntity::is_loop());
        CHECK_FALSE(open.ExtrusionEntity::is_loop());
    }
}

// Convert a real near-closed variable-width centerline and own every returned pointer immediately;
// preserve its unequal endpoints and compare generic closure to stored XY, not to the legacy Loop override.
TEST_CASE("Thin variable width output keeps exact XY closure semantics",
          "[plugins][extrusion][extrusion-hybrid][hybrid-thin-width]")
{
    ThickPolyline line;
    line.points = {Point(0, 0), Point(scale_i(5.), 0), Point(scale_i(5.), scale_i(5.)),
                   Point(0, scale_i(5.)), Point(1, 0)};
    line.points_width.assign(line.points.size(), scale_i(0.45));
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    ExtrusionEntitiesPtr raw = Geometry::thin_variable_width(
        {line}, ExtrusionRole::GapFill, flow, flow.scaled_width() / 8, true);
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    for (ExtrusionEntity *entity : raw) parent.append_child(ExtrusionEntityUPtr(entity));
    REQUIRE(parent.child_count() == 1);
    const ExtrusionEntity &view = parent;
    REQUIRE(dynamic_cast<const ExtrusionLoop *>(&view.child(0)) != nullptr);
    CHECK(raw.front()->is_loop());
    CHECK_FALSE(raw.front()->ExtrusionEntity::is_loop());
    CHECK(extrusion_is_loop(hybrid_handle(*raw.front())) == 0);
    std::vector<ArcPolyline> geometry;
    hybrid_geometry(view, geometry);
    REQUIRE_FALSE(geometry.empty());
    REQUIRE(geometry.front().front() != geometry.back().back());
    CHECK(extrusion_is_loop(hybrid_handle(parent)) == 0);
    hybrid_check_geometry(view, geometry);
}

// Creates owned collections before starting readers, gates each cold read on all eight workers being ready, and checks worker-local
// results after joining. Children outlive every reader; accessor exceptions are retained for the test thread, and startup failures join workers.
TEST_CASE("Concurrent const collection access matches owned children", "[plugins][extrusion][extrusion-collection-threads]")
{
    constexpr size_t collection_count = 200, child_count = 256, worker_count = 8;
    std::vector<std::unique_ptr<Slic3r::ExtrusionEntityCollection>> collections;
    collections.reserve(collection_count);
    for (size_t round = 0; round < collection_count; ++round) {
        std::unique_ptr<Slic3r::ExtrusionEntityCollection> collection = std::make_unique<Slic3r::ExtrusionEntityCollection>();
        for (size_t idx = 0; idx < child_count; ++idx)
            collection->append(path_between(Slic3r::Point(int(idx), 0), Slic3r::Point(int(idx + 1), 0)));
        // Appending leaves the pointer cache empty; no const collection accessor runs before the synchronized readers.
        collections.push_back(std::move(collection));
    }
    std::atomic<size_t> ready{0}, phase{0};
    std::atomic<bool> stop{false};
    std::vector<size_t> mismatches(worker_count, 0);
    std::vector<std::exception_ptr> errors(worker_count);
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    try {
        for (size_t worker = 0; worker < worker_count; ++worker) {
            // Reads each collection only after the release gate, records failures in this worker's slots and owns no geometry;
            // each round reaches the next barrier even after an accessor exception so the coordinator can finish and join.
            workers.emplace_back([&, worker] {
                for (size_t round = 0; round < collection_count; ++round) {
                    ready.fetch_add(1, std::memory_order_release);
                    while (phase.load(std::memory_order_acquire) < round + 1 && !stop.load(std::memory_order_acquire))
                        std::this_thread::yield();
                    if (stop.load(std::memory_order_acquire)) return;
                    try {
                        const Slic3r::ExtrusionEntityCollection &collection = *collections[round];
                        for (size_t pass = 0; pass < 3; ++pass)
                            if (!collection_reads_match(collection)) ++mismatches[worker];
                    } catch (...) {
                        if (!errors[worker]) errors[worker] = std::current_exception();
                    }
                }
            });
        }
    } catch (...) {
        stop.store(true, std::memory_order_release);
        for (std::thread &worker : workers) worker.join();
        throw;
    }
    for (size_t round = 0; round < collection_count; ++round) {
        while (ready.load(std::memory_order_acquire) < (round + 1) * worker_count)
            std::this_thread::yield();
        phase.store(round + 1, std::memory_order_release);
    }
    for (std::thread &worker : workers) worker.join();
    for (size_t worker = 0; worker < worker_count; ++worker) {
        INFO("reader " << worker);
        if (errors[worker]) std::rethrow_exception(errors[worker]);
        CHECK(mismatches[worker] == 0);
    }
}

TEST_CASE("Extrusion continuity is computed from tree order", "[plugins][extrusion][continuity]")
{
    const Slic3r::Point p0(0, 0);
    const Slic3r::Point p1(100, 0);
    const Slic3r::Point p2(200, 0);
    const Slic3r::Point p3(300, 0);

    SECTION("empty entities and leaf polylines are continuous")
    {
        // A leaf has no child order to validate. It is therefore continuous by
        // nature unless a caller explicitly marks it sortable, which is an
        // invalid state for local-polyline entities and rejected by the C API.
        Slic3r::ExtrusionEntity empty(true);
        Slic3r::ExtrusionPath leaf = path_between(p0, p1);

        CHECK(empty.is_continuous());
        CHECK(leaf.is_continuous());
    }

    SECTION("sortable collections are never continuous")
    {
        // Sorting gives path planners permission to reorder children. The
        // current children happen to touch, but the collection is not a forced
        // continuous path while m_can_sort is enabled.
        Slic3r::ExtrusionEntityCollection sortable;
        sortable.append(path_between(p0, p1));
        sortable.append(path_between(p1, p2));

        CHECK(sortable.can_sort());
        CHECK_FALSE(sortable.is_continuous());
    }

    SECTION("non-sortable collections are continuous when children touch")
    {
        // Disabling sorting makes the current child order authoritative. The
        // continuity check then walks non-empty children and verifies that each
        // child ends exactly where the next one starts.
        Slic3r::ExtrusionEntityCollection ordered(false, true);
        ordered.append(path_between(p0, p1));
        ordered.append(path_between(p1, p2));

        CHECK_FALSE(ordered.can_sort());
        CHECK(ordered.is_continuous());
    }

    SECTION("a gap between two children breaks continuity")
    {
        Slic3r::ExtrusionEntityCollection ordered(false, true);
        ordered.append(path_between(p0, p1));
        ordered.append(path_between(p2, p3));

        CHECK_FALSE(ordered.is_continuous());
    }

    SECTION("a continuous closed generic collection is a loop")
    {
        Slic3r::ExtrusionEntityCollection loop(false, true);
        loop.append(path_between(p0, p1));
        loop.append(path_between(p1, p0));

        CHECK(loop.is_continuous());
        CHECK(loop.is_loop());
    }

    SECTION("a non-continuous child makes the parent non-continuous")
    {
        // Continuity is recursive. A parent cannot become continuous just
        // because its child collection has matching first/back points if that
        // child remains sortable internally.
        Slic3r::ExtrusionEntityCollection sortable_child;
        sortable_child.append(path_between(p0, p1));
        sortable_child.append(path_between(p1, p2));

        Slic3r::ExtrusionEntityCollection parent(false, true);
        parent.append(sortable_child);

        CHECK_FALSE(sortable_child.is_continuous());
        CHECK_FALSE(parent.is_continuous());
    }
}

TEST_CASE("Extrusion tree simplification removes only transparent collection wrappers",
          "[plugins][extrusion][simplify]")
{
    const Slic3r::Point p0(0, 0);
    const Slic3r::Point p1(100, 0);
    const Slic3r::Point p2(200, 0);
    const Slic3r::Point p3(300, 0);
    const Slic3r::Point p4(400, 0);

    SECTION("empty transparent collection child is removed")
    {
        // Empty sortable collections sometimes appear as temporary plugin
        // output. If they carry no state, deleting them cannot change the
        // printed geometry or inherited properties.
        Slic3r::ExtrusionEntityCollection root;
        root.append(std::make_unique<Slic3r::ExtrusionEntityCollection>());

        CHECK(simplify_extrusion_tree(root));

        CHECK(root.child_count() == 0);
    }

    SECTION("transparent collection child is spliced into its parent")
    {
        // The wrapper has the same ordering/reversing behavior as its parent
        // and no explicit properties. Its children can therefore inherit from
        // the parent directly after the wrapper is removed.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).first_point() == p0);
        CHECK(root.child(1).first_point() == p2);
    }

    SECTION("unknown explicit child property blocks simplification")
    {
        // A property present on the wrapper but absent from the parent would no
        // longer be inherited by the grandchildren after splicing. The wrapper
        // must stay in the tree.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("different explicit child property blocks simplification")
    {
        // Properties are inherited state. A wrapper that overrides the parent
        // with different bytes is semantically meaningful and cannot be removed.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(20.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("same explicit child property allows simplification")
    {
        // Repeating the exact same property payload on the wrapper does not
        // change what the grandchildren observe. The duplicate wrapper can be
        // removed and the parent remains the source of the inherited property.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).first_point() == p0);
        CHECK(root.child(0).get_property<Slic3r::ExtrusionPropertySpeed>() == nullptr);
        REQUIRE(root.get_property<Slic3r::ExtrusionPropertySpeed>() != nullptr);
        CHECK(root.get_property<Slic3r::ExtrusionPropertySpeed>()->speed_mm_per_s == 10.f);
    }

    SECTION("ordering flags protect collection wrappers")
    {
        // Removing a wrapper with a different sorting contract would give the
        // parent permission to reorder children that were previously protected,
        // or the other way around. Such wrappers are not transparent.
        Slic3r::ExtrusionEntityCollection wrapper(false, true);
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));
        REQUIRE(wrapper.is_collection());

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("grandchild properties survive splicing")
    {
        // Simplification moves existing child objects; it must not clone them
        // into new leaves or drop properties carried by deeper nodes.
        Slic3r::ExtrusionPath first = path_between(p0, p1);
        first.add_property(Slic3r::ExtrusionPropertySpeed(30.f));

        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.append(std::move(first));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        const Slic3r::ExtrusionPropertySpeed *speed =
            root.child(0).get_property<Slic3r::ExtrusionPropertySpeed>();
        REQUIRE(speed != nullptr);
        CHECK(speed->speed_mm_per_s == 30.f);
    }

    SECTION("loops and continuous multipaths are left intact")
    {
        // Closed and continuous non-sortable structures encode printable path
        // order. Even if they contain several child paths, they are not plain
        // structural collections and are therefore outside this simplifier.
        Slic3r::ExtrusionEntityCollection loop(false, true);
        loop.append(path_between(p0, p1));
        loop.append(path_between(p1, p0));
        REQUIRE(loop.is_loop());

        Slic3r::ExtrusionEntityCollection multipath(false, true);
        multipath.append(path_between(p2, p3));
        multipath.append(path_between(p3, p4));
        REQUIRE(multipath.is_continuous());
        REQUIRE_FALSE(multipath.is_loop());

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(loop));
        root.append(std::move(multipath));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).is_loop());
        CHECK(root.child(0).child_count() == 2);
        CHECK(root.child(1).is_continuous());
        CHECK(root.child(1).child_count() == 2);
    }
}
