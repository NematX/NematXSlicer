///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Tests for ordered boundary leaves in the unified extrusion tree.

The helper keeps the caller's outer handle stable while either inserting into
an already fixed collection or preserving the previous logical entity below a
new wrapper. These tests focus on ownership, inherited-property placement and
the strong no-mutation contract of rejected C calls.
Rotation permission is separate from sorting/reversal and geometric closure.
Owned copies, moves, splits and publication exchanges must preserve that value
with the content; ordered command boundaries forbid rotation of their wrapper.

    TEST_CASE
        |-- mutable_view(): borrow a host-owned node through the SDK
        |-- copy/move/swap_content(): retain permissions and owned data
        `-- split or insert ordered leaves: keep geometry and execution boundaries
*/

#include <catch2/catch.hpp>

#include <memory>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

namespace {

Slic3r::ExtrusionEntityUPtr empty_child();
extrusion_entity_handle *entity_handle(Slic3r::ExtrusionEntity &entity);
slic3r_api::MutableExtrusionEntity mutable_view(Slic3r::ExtrusionEntity &entity);

Slic3r::ExtrusionEntityUPtr empty_child()
{
    return std::make_unique<Slic3r::ExtrusionEntity>(false);
}

extrusion_entity_handle *entity_handle(Slic3r::ExtrusionEntity &entity)
{
    return reinterpret_cast<extrusion_entity_handle *>(&entity);
}

slic3r_api::MutableExtrusionEntity mutable_view(Slic3r::ExtrusionEntity &entity)
{
    return slic3r_api::MutableExtrusionEntity(entity_handle(entity));
}

} // namespace

// Compare raw defaults and both SDK functions without changing geometric classification or legacy flags.
TEST_CASE("Rotation permission is independent of geometry and legacy flags",
          "[plugins][extrusion][rotation]")
{
    using namespace Slic3r;
    ExtrusionEntity generic(false);
    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter});
    ExtrusionEntityCollection collection;
    ExtrusionLoop loop;
    ExtrusionMultiPath multi;
    ExtrusionMultiEntity<ExtrusionPath> sequence;
    CHECK(generic.can_rotate());
    CHECK(path.can_rotate());
    CHECK(collection.can_rotate());
    CHECK(loop.can_rotate());
    CHECK_FALSE(multi.can_rotate());
    CHECK_FALSE(sequence.can_rotate());
    CHECK(extrusion_can_rotate(nullptr) == 0);
    CHECK(extrusion_set_can_rotate(nullptr, 1) == 0);

    auto view = mutable_view(generic);
    CHECK(view.can_rotate());
    CHECK_FALSE(view.is_loop());
    REQUIRE(view.set_can_rotate(false));
    CHECK_FALSE(generic.can_rotate());
    REQUIRE(view.set_flags(RAW_EXTRUSION_FLAG_REVERSIBLE));
    CHECK_FALSE(view.can_rotate());
    REQUIRE(view.set_can_rotate(true));
    view.disable_reverse().disable_sort();
    CHECK(view.can_rotate());
    CHECK(view.flags() == 0);
    CHECK_FALSE(view.is_loop());
    generic.clear_content();
    CHECK(generic.can_rotate());
    generic.set_can_rotate(false);
    generic.clear_content();
    CHECK_FALSE(generic.can_rotate());
}

// Exchange distinct raw permissions with geometry, keeping both node identities.
TEST_CASE("Content publication exchanges rotation with geometry without moving identities",
          "[plugins][extrusion][rotation][publication]")
{
    using namespace Slic3r;
    for (bool allowed : {false, true}) {
        ExtrusionEntity first(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        ExtrusionEntity second(true, ArcPolyline(Points{{20000, 0}, {30000, 0}}));
        first.set_can_rotate(allowed);
        second.set_can_rotate(!allowed);
        first.add_property(ExtrusionPropertySpeed(20.f));
        second.add_property(ExtrusionPropertySpeed(30.f));
        const uint64_t first_id = first.get_id();
        const uint64_t second_id = second.get_id();
        first.swap_content(second);
        CHECK(first.get_id() == first_id);
        CHECK(second.get_id() == second_id);
        CHECK(first.can_rotate() == !allowed);
        CHECK(second.can_rotate() == allowed);
        CHECK(first.first_point() == Point(20000, 0));
        CHECK(second.first_point() == Point(0, 0));
        CHECK(first.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 30.f);
        CHECK(second.get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 20.f);
    }
}

// Exercise all base transfers and collection swap with both permission values; copies retain owned geometry independently.
TEST_CASE("Rotation permission follows copies moves and geometry wrappers",
          "[plugins][extrusion][rotation]")
{
    using namespace Slic3r;
    for (const bool allowed : {false, true}) {
        ExtrusionEntity source(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        source.set_can_rotate(allowed);
        ExtrusionEntity copy(source);
        CHECK(copy.can_rotate() == allowed);
        ExtrusionEntity moved(std::move(copy));
        CHECK(moved.can_rotate() == allowed);
        ExtrusionEntity assigned(false);
        assigned = source;
        CHECK(assigned.can_rotate() == allowed);
        assigned = std::move(moved);
        CHECK(assigned.can_rotate() == allowed);
        std::unique_ptr<ExtrusionEntity> clone(source.clone());
        CHECK(clone->can_rotate() == allowed);
        source.set_polyline(ArcPolyline(Points{{0, 0}, {20000, 0}}));
        CHECK(source.can_rotate() == allowed);
        source.append_child(std::make_unique<ExtrusionEntity>(false));
        const ExtrusionEntity &wrapped = source;
        REQUIRE(wrapped.child_count() == 2);
        CHECK(wrapped.child(0).can_rotate() == allowed);
        CHECK(source.can_rotate() == allowed);

        ExtrusionEntityCollection a;
        ExtrusionEntityCollection b;
        a.set_can_rotate(allowed);
        b.set_can_rotate(!allowed);
        a.swap(b);
        CHECK(a.can_rotate() == !allowed);
        CHECK(b.can_rotate() == allowed);
    }
    ExtrusionLoop loop;
    loop.set_can_rotate(false);
    std::unique_ptr<ExtrusionEntity> copy(loop.clone());
    CHECK_FALSE(copy->can_rotate());
    std::unique_ptr<ExtrusionEntity> moved(loop.clone_move());
    CHECK_FALSE(moved->can_rotate());
    ExtrusionMultiPath sequence;
    sequence.set_can_rotate(true);
    std::unique_ptr<ExtrusionEntity> sequence_copy(sequence.clone());
    CHECK(sequence_copy->can_rotate());
}

// Preserve permission on split containers/fragments and wrapped content, while execution phases and event roots stay fixed.
TEST_CASE("Rotation permission separates content from phases and split fragments",
          "[plugins][extrusion][rotation]")
{
    using namespace Slic3r;
    for (const bool allowed : {false, true}) {
        ExtrusionEntity source(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        source.set_can_rotate(allowed);
        const ExPolygons first{ExPolygon({Point(-1000, -1000), Point(5000, -1000), Point(5000, 1000), Point(-1000, 1000)})};
        const ExPolygons second{ExPolygon({Point(5000, -1000), Point(11000, -1000), Point(11000, 1000), Point(5000, 1000)})};
        const expolygon_collection_handle *areas[] = {reinterpret_cast<const expolygon_collection_handle *>(&first),
                                                       reinterpret_cast<const expolygon_collection_handle *>(&second)};
        REQUIRE(extrusion_split_leaf_by_areas(entity_handle(source), areas, 2, 0, nullptr, nullptr) ==
                RAW_EXTRUSION_SPLIT_STATUS_SUCCESS);
        REQUIRE(source.child_count() == 2);
        CHECK(source.can_rotate() == allowed);
        const ExtrusionEntity &split = source;
        for (const auto &fragment : split.children())
            CHECK(fragment->can_rotate() == allowed);

        ExtrusionEntity root(ExtrusionEntity::Children(), true, true, false);
        root.set_can_rotate(allowed);
        auto child = std::make_unique<ExtrusionEntity>(false);
        ExtrusionEntity *stable = child.get();
        root.append_child(std::move(child));
        auto *phase = root.emplace_ordered_leaf(OrderedLeafPosition::Before,
                                               ExistingPropertyPlacement::MoveWithExistingContent);
        REQUIRE(phase != nullptr);
        CHECK_FALSE(phase->can_rotate());
        CHECK_FALSE(root.can_rotate());
        const ExtrusionEntity &view = root;
        CHECK(view.child(1).can_rotate() == allowed);
        CHECK(&view.child(1).child(0) == stable);
    }
    Printing::PrintingScopeEvents events;
    CHECK_FALSE(events.before().can_rotate());
    CHECK_FALSE(events.after().can_rotate());
    events.clear();
    CHECK_FALSE(events.before().can_rotate());
    CHECK_FALSE(events.after().can_rotate());
}

// Keep a structural wrapper whenever removing it would lose an explicit cyclic-start restriction.
TEST_CASE("Extrusion simplification retains differing rotation permissions",
          "[plugins][extrusion][rotation]")
{
    using namespace Slic3r;
    ExtrusionEntity root(ExtrusionEntity::Children(), true, true, false);
    auto child = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), true, true, false);
    child->set_can_rotate(false);
    root.append_child(std::move(child));
    CHECK_FALSE(simplify_extrusion_tree(root));
    REQUIRE(root.child_count() == 1);
    root.child(0).set_can_rotate(true);
    CHECK(simplify_extrusion_tree(root));
    CHECK(root.child_count() == 0);
}

TEST_CASE("Ordered leaves insert directly into fixed parents",
          "[plugins][extrusion][ordered-leaf][rotation]")
{
    Slic3r::ExtrusionEntity::Children children;
    children.emplace_back(empty_child());
    children.emplace_back(empty_child());
    Slic3r::ExtrusionEntity root(std::move(children), false, false, true);
    root.set_can_rotate(true);
    root.child(0).set_can_rotate(false);
    root.child(1).set_can_rotate(true);
    root.add_property(Slic3r::ExtrusionPropertySpeed(35.f, 700.f));

    Slic3r::ExtrusionEntity *first_original = &root.child(0);
    Slic3r::ExtrusionEntity *second_original = &root.child(1);
    slic3r_api::MutableExtrusionEntity inserted = mutable_view(root).emplace_ordered_leaf(
        slic3r_api::OrderedLeafPosition::Before,
        slic3r_api::ExistingPropertyPlacement::KeepOnParent);

    REQUIRE(inserted.valid());
    REQUIRE(root.child_count() == 3);
    CHECK(&root.child(0) == reinterpret_cast<Slic3r::ExtrusionEntity *>(inserted.mutable_handle()));
    CHECK(&root.child(1) == first_original);
    CHECK(&root.child(2) == second_original);
    CHECK(root.get_property<Slic3r::ExtrusionPropertySpeed>() != nullptr);
    CHECK_FALSE(root.can_sort());
    CHECK_FALSE(root.can_reverse());
    CHECK_FALSE(root.can_rotate());
    CHECK_FALSE(inserted.can_rotate());
    CHECK_FALSE(first_original->can_rotate());
    CHECK(second_original->can_rotate());
    CHECK((inserted.flags() & RAW_EXTRUSION_FLAG_SORTABLE) == 0);
    CHECK((inserted.flags() & RAW_EXTRUSION_FLAG_REVERSIBLE) == 0);
}

// Repeated fixed-parent insertions keep the first wrapper's permission and its child objects intact.
TEST_CASE("Repeated ordered leaves retain the preserved content permission",
          "[plugins][extrusion][ordered-leaf][rotation]")
{
    using namespace Slic3r;
    for (const bool allowed : {false, true}) {
        ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
        root.set_can_rotate(allowed);
        auto child = empty_child();
        child->set_can_rotate(!allowed);
        ExtrusionEntity *original_child = child.get();
        root.append_child(std::move(child));
        ExtrusionEntity *after = root.emplace_ordered_leaf(
            OrderedLeafPosition::After, ExistingPropertyPlacement::MoveWithExistingContent);
        REQUIRE(after != nullptr);
        ExtrusionEntity *content = &root.child(0);
        ExtrusionEntity *before = root.emplace_ordered_leaf(
            OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
        REQUIRE(before != nullptr);
        ExtrusionEntity *travel = root.emplace_ordered_leaf(
            OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
        REQUIRE(travel != nullptr);
        REQUIRE(root.child_count() == 4);
        CHECK(&root.child(0) == travel);
        CHECK(&root.child(1) == before);
        CHECK(&root.child(2) == content);
        CHECK(&root.child(3) == after);
        CHECK_FALSE(root.can_rotate());
        CHECK_FALSE(travel->can_rotate());
        CHECK_FALSE(before->can_rotate());
        CHECK_FALSE(after->can_rotate());
        CHECK(content->can_rotate() == allowed);
        REQUIRE(content->child_count() == 1);
        CHECK(&content->child(0) == original_child);
        CHECK(original_child->can_rotate() == !allowed);
    }
}

// A multi-path native loop split must copy the selected path's permission, not the loop's default.
TEST_CASE("Native loop splits retain each source path rotation permission",
          "[plugins][extrusion][rotation]")
{
    using namespace Slic3r;
    for (const bool allowed : {false, true}) {
        ExtrusionPath first(ExtrusionAttributes{ExtrusionRole::Perimeter});
        first.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}}));
        first.set_can_rotate(allowed);
        ExtrusionPath second(ExtrusionAttributes{ExtrusionRole::Perimeter});
        second.set_polyline(ArcPolyline(Points{{10000, 10000}, {0, 10000}, {0, 0}}));
        second.set_can_rotate(!allowed);
        ExtrusionLoop loop(ExtrusionPaths{first, second});
        loop.set_can_rotate(!allowed);
        loop.split_at(Point(5000, 0), false, 0.);

        REQUIRE(loop.child_count() == 3);
        CHECK(loop.can_rotate() == !allowed);
        const ExtrusionEntity &split = loop;
        CHECK(split.child(0).can_rotate() == allowed);
        CHECK(split.child(1).can_rotate() == !allowed);
        CHECK(split.child(2).can_rotate() == allowed);
        const Points expected_first{{5000, 0}, {10000, 0}, {10000, 10000}};
        const Points expected_middle{{10000, 10000}, {0, 10000}, {0, 0}};
        const Points expected_last{{0, 0}, {5000, 0}};
        CHECK(split.child(0).polyline_ref().to_polyline().points == expected_first);
        CHECK(split.child(1).polyline_ref().to_polyline().points == expected_middle);
        CHECK(split.child(2).polyline_ref().to_polyline().points == expected_last);
    }
}

TEST_CASE("Ordered leaves wrap content whose ordering may change",
          "[plugins][extrusion][ordered-leaf]")
{
    Slic3r::ExtrusionEntity::Children children;
    children.emplace_back(empty_child());
    Slic3r::ExtrusionEntity root(std::move(children), true, true, false);
    root.add_property(Slic3r::ExtrusionPropertySpeed(20.f, 400.f));
    Slic3r::ExtrusionEntity *original_child = &root.child(0);

    slic3r_api::MutableExtrusionEntity inserted = mutable_view(root).emplace_ordered_leaf(
        slic3r_api::OrderedLeafPosition::After,
        slic3r_api::ExistingPropertyPlacement::KeepOnParent);

    REQUIRE(inserted.valid());
    REQUIRE(root.child_count() == 2);
    const Slic3r::ExtrusionEntity &preserved = root.child(0);
    CHECK(&root.child(1) == reinterpret_cast<Slic3r::ExtrusionEntity *>(inserted.mutable_handle()));
    REQUIRE(preserved.child_count() == 1);
    CHECK(&preserved.child(0) == original_child);
    CHECK(preserved.can_sort());
    CHECK(preserved.can_reverse());
    CHECK(preserved.get_property<Slic3r::ExtrusionPropertySpeed>() == nullptr);
    CHECK(root.get_property<Slic3r::ExtrusionPropertySpeed>() != nullptr);
    CHECK_FALSE(root.can_sort());
    CHECK_FALSE(root.can_reverse());
}

TEST_CASE("Moving existing properties always creates a preserved-content wrapper",
          "[plugins][extrusion][ordered-leaf]")
{
    Slic3r::ExtrusionEntity::Children children;
    children.emplace_back(empty_child());
    Slic3r::ExtrusionEntity root(std::move(children), false, false, true);
    Slic3r::ExtrusionEntity *original_child = &root.child(0);

    slic3r_api::MutableExtrusionEntity inserted = mutable_view(root).emplace_ordered_leaf(
        slic3r_api::OrderedLeafPosition::Before,
        slic3r_api::ExistingPropertyPlacement::MoveWithExistingContent);

    REQUIRE(inserted.valid());
    REQUIRE(root.child_count() == 2);
    REQUIRE(root.child(1).child_count() == 1);
    CHECK(&root.child(1).child(0) == original_child);
    CHECK(&root.child(0) == reinterpret_cast<Slic3r::ExtrusionEntity *>(inserted.mutable_handle()));
}

TEST_CASE("Ordered leaf wrapping moves property resources with leaf content",
          "[plugins][extrusion][ordered-leaf]")
{
    const Slic3r::coord_t unit = 1000;
    Slic3r::ArcPolyline polyline;
    polyline.append(Slic3r::Point(unit, 0));
    polyline.append(Slic3r::Geometry::ArcWelder::Segment(
        Slic3r::Point(0, unit),
        float(unit),
        Slic3r::Geometry::ArcWelder::Orientation::CCW));
    polyline.set_z_offset(0, 100);
    polyline.set_z_offset(1, 900);
    Slic3r::ExtrusionEntity root(false, std::move(polyline));
    Slic3r::ExtrusionPropertyCustomGcode &custom = root.add_property(
        Slic3r::ExtrusionPropertyCustomGcodeText("M117 preserved custom G-code"));
    REQUIRE(root.custom_gcode_string(custom) == "M117 preserved custom G-code");

    slic3r_api::MutableExtrusionEntity inserted = mutable_view(root).emplace_ordered_leaf(
        slic3r_api::OrderedLeafPosition::After,
        slic3r_api::ExistingPropertyPlacement::MoveWithExistingContent);

    REQUIRE(inserted.valid());
    REQUIRE(root.child_count() == 2);
    CHECK(root.get_property<Slic3r::ExtrusionPropertyCustomGcode>() == nullptr);
    const Slic3r::ExtrusionEntity &preserved = root.child(0);
    const Slic3r::ExtrusionPropertyCustomGcode *preserved_custom =
        preserved.get_property<Slic3r::ExtrusionPropertyCustomGcode>();
    REQUIRE(preserved_custom != nullptr);
    CHECK(preserved.custom_gcode_string(*preserved_custom) == "M117 preserved custom G-code");
    CHECK(preserved.has_polyline());
    CHECK(preserved.polyline_ref().size() == 2);
    CHECK(preserved.polyline_ref().has_arc());
    CHECK(preserved.polyline_ref().has_z_offset());
    CHECK(preserved.polyline_ref().z_offset(0) == 100);
    CHECK(preserved.polyline_ref().z_offset(1) == 900);
    CHECK(preserved.polyline_ref().get_arc().back().orientation ==
          Slic3r::Geometry::ArcWelder::Orientation::CCW);
    CHECK(&root.child(1) == reinterpret_cast<Slic3r::ExtrusionEntity *>(inserted.mutable_handle()));
}

TEST_CASE("Invalid ordered leaf arguments leave the entity unchanged",
          "[plugins][extrusion][ordered-leaf]")
{
    Slic3r::ExtrusionEntity root(false);
    root.add_property(Slic3r::ExtrusionPropertySpeed(12.f, 300.f));

    extrusion_entity_handle *result = extrusion_emplace_ordered_leaf(
        entity_handle(root),
        static_cast<raw_extrusion_ordered_leaf_position>(99),
        RAW_EXTRUSION_EXISTING_PROPERTIES_KEEP_ON_PARENT);

    CHECK(result == nullptr);
    CHECK(root.is_leaf());
    CHECK(root.child_count() == 0);
    CHECK(root.get_property<Slic3r::ExtrusionPropertySpeed>() != nullptr);
}
