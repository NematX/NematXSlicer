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
        |-- split or insert ordered leaves: keep geometry and execution boundaries
        `-- check_canonical_copy(): compare owned copies recursively by content

Canonical-copy tests compare stored geometry and properties recursively, without
requiring the virtual behavior of legacy path and collection views.
*/

#include <catch2/catch.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <typeinfo>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntityOrdering.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

namespace {

Slic3r::ExtrusionEntityUPtr empty_child();
extrusion_entity_handle *entity_handle(Slic3r::ExtrusionEntity &entity);
slic3r_api::MutableExtrusionEntity mutable_view(Slic3r::ExtrusionEntity &entity);
// Compare stored nodes and exact copied types without entering legacy path views.
void check_canonical_copy(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &copy);

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

void check_canonical_copy(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &copy)
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    CHECK(&source != &copy);
    if (const ExtrusionNop *nop = dynamic_cast<const ExtrusionNop *>(&source)) {
        REQUIRE(typeid(copy) == typeid(ExtrusionNop));
        const ExtrusionNop &copied_nop = static_cast<const ExtrusionNop &>(copy);
        CHECK(copied_nop.position == nop->position);
        CHECK(copied_nop.role() == nop->role());
    } else {
        CHECK(typeid(copy) == typeid(ExtrusionEntity));
    }
    CHECK(copy.get_id() == source.get_id());
    CHECK(copy.can_sort() == source.can_sort());
    CHECK(copy.can_reverse() == source.can_reverse());
    CHECK(copy.can_rotate() == source.can_rotate());
    CHECK(copy.is_nop() == source.is_nop());
    CHECK(copy.is_leaf() == source.is_leaf());
    REQUIRE(copy.has_polyline() == source.has_polyline());
    if (source.has_polyline()) {
        const ArcPolyline &original = source.polyline_ref();
        const ArcPolyline &copied = copy.polyline_ref();
        REQUIRE(copied.size() == original.size());
        REQUIRE(copied.has_arc() == original.has_arc());
        REQUIRE(copied.has_z_offset() == original.has_z_offset());
        for (size_t index = 0; index < original.size(); ++index) {
            CHECK(copied.get_point(index) == original.get_point(index));
            if (original.has_z_offset())
                CHECK(copied.z_offset(index) == original.z_offset(index));
            if (original.has_arc()) {
                CHECK(copied.get_arc(index).radius == original.get_arc(index).radius);
                CHECK(copied.get_arc(index).orientation == original.get_arc(index).orientation);
            }
        }
    }
    REQUIRE(Access::property_count(copy) == Access::property_count(source));
    for (size_t index = 0; index < Access::property_count(source); ++index) {
        const extrusion_property_type type = Access::property_type_at(source, index);
        CHECK(Access::property_type_at(copy, index) == type);
        CHECK(Access::same_property_payload(source, type, copy));
    }
    REQUIRE(copy.child_count() == source.child_count());
    for (size_t index = 0; index < source.child_count(); ++index)
        check_canonical_copy(source.child(index), copy.child(index));
}

} // namespace

// Seam eligibility uses stored geometry and permission, never the legacy type's unconditional loop answer.
TEST_CASE("Seam units use exact closure and an independent rotation permission",
          "[plugins][extrusion][seam-unit-preparation]")
{
    using namespace Slic3r;
    const Points closed{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}};
    ExtrusionEntity generic(false, ArcPolyline(closed));
    CHECK(is_seam_unit(generic));
    generic.set_can_rotate(false);
    CHECK_FALSE(is_seam_unit(generic));
    CHECK(generic.ExtrusionEntity::is_loop());

    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter});
    path.set_polyline(ArcPolyline(closed));
    ExtrusionMultiPath sequence(path);
    CHECK_FALSE(sequence.is_loop());
    CHECK(sequence.ExtrusionEntity::is_loop());
    CHECK_FALSE(is_seam_unit(sequence));
    sequence.set_can_rotate(true);
    CHECK(is_seam_unit(sequence));
    CHECK_FALSE(sequence.is_loop());

    ExtrusionLoop empty;
    CHECK(empty.is_loop());
    CHECK_FALSE(is_seam_unit(empty));
    ExtrusionLoop almost_closed;
    std::unique_ptr<ExtrusionPath> near = std::make_unique<ExtrusionPath>(path);
    near->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {1, 0}}));
    almost_closed.append_child(std::move(near));
    CHECK(almost_closed.is_loop());
    CHECK_FALSE(is_seam_unit(almost_closed));
    CHECK(almost_closed.child(0).polyline_ref().back() == Point(1, 0));
    ExtrusionEntityUPtr canonical = almost_closed.clone_canonical();
    CHECK_FALSE(is_seam_unit(*canonical));
    CHECK(canonical->child(0).polyline_ref().back() == Point(1, 0));
}

// A fixed execution wrapper does not acquire a seam merely because its geometric child closes.
TEST_CASE("Protected extrusion wrappers retain separate eligible contour children",
          "[plugins][extrusion][seam-unit-preparation]")
{
    using namespace Slic3r;
    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter});
    path.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}}));
    ExtrusionLoop loop(std::move(path));
    REQUIRE(is_seam_unit(loop));
    ExtrusionEntity *before = loop.emplace_ordered_leaf(
        OrderedLeafPosition::Before, ExistingPropertyPlacement::MoveWithExistingContent);
    REQUIRE(before != nullptr);
    const ExtrusionEntity &view = loop;
    REQUIRE(view.child_count() == 2);
    CHECK(loop.is_loop());
    CHECK(loop.ExtrusionEntity::is_loop());
    CHECK_FALSE(is_seam_unit(loop));
    CHECK_FALSE(is_seam_unit(*before));
    CHECK(is_seam_unit(view.child(1)));
    ExtrusionEntityUPtr copy = loop.clone_canonical();
    CHECK_FALSE(is_seam_unit(*copy));
    CHECK(is_seam_unit(copy->child(1)));
    CHECK(copy->child(1).first_point() == view.child(1).ExtrusionEntity::first_point());
}

// Copy mixed native and hybrid trees once without changing their source handles or legacy clone behavior.
TEST_CASE("Canonical extrusion copies remove legacy views at every depth",
          "[plugins][extrusion][canonical-copy]")
{
    using namespace Slic3r;
    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter});
    path.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}}));
    path.set_can_rotate(false);
    ExtrusionEntityCollection source;
    source.append_child(std::make_unique<ExtrusionEntity>(false));
    source.append_child(std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true));
    source.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline()));
    source.append_child(std::make_unique<ExtrusionPath>(path));
    source.append_child(std::make_unique<ExtrusionMultiPath>(path));
    source.append_child(std::make_unique<ExtrusionMultiEntity<ExtrusionPath>>(path));
    std::unique_ptr<ExtrusionLoop> loop = std::make_unique<ExtrusionLoop>(path, elrSkirt);
    loop->emplace_ordered_leaf(OrderedLeafPosition::Before, ExistingPropertyPlacement::MoveWithExistingContent);
    source.append_child(std::move(loop));
    std::unique_ptr<ExtrusionEntityCollection> nested = std::make_unique<ExtrusionEntityCollection>();
    nested->append_child(std::make_unique<ExtrusionPath>(path));
    source.append_child(std::move(nested));
    const ExtrusionEntity &view = source;
    const ExtrusionEntity *first_path = &view.child(3);
    ExtrusionEntityUPtr copy = source.clone_canonical();
    check_canonical_copy(source, *copy);
    CHECK(&view.child(3) == first_path);
    CHECK(typeid(view.child(3)) == typeid(ExtrusionPath));
    CHECK(typeid(view.child(6)) == typeid(ExtrusionLoop));
    CHECK(copy->child(0).is_nop());
    CHECK_FALSE(copy->child(1).is_nop());
    CHECK_FALSE(copy->child(1).is_leaf());
    CHECK(copy->child(2).has_polyline());
    CHECK(copy->child(2).polyline_ref().empty());
    copy->child(3).polyline_ref().translate(Point(123, 456));
    CHECK(view.child(3).polyline_ref().front() == Point(0, 0));
    CHECK(copy->child(3).polyline_ref().front() == Point(123, 456));
    ExtrusionEntityUPtr legacy(source.clone());
    CHECK(typeid(*legacy) == typeid(ExtrusionEntityCollection));
    CHECK(typeid(legacy->child(3)) == typeid(ExtrusionPath));
}

// Owned property buffers, their IDs and their alignment survive source destruction independently of the copied arc geometry.
TEST_CASE("Canonical extrusion copies own unknown properties scripts and aligned buffers",
          "[plugins][extrusion][canonical-copy]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    struct alignas(64) Payload { extrusion_data_id data_id; uint64_t marker; };
    const extrusion_property_type custom_type = SLIC3R_PROPERTY_TYPE_CUSTOM_BEGIN + 931;
    const std::array<uint64_t, 16> expected{{1, 23, 456, 7890}};
    const std::string text = "M117 {layer_num}";
    const std::string config = "layer_num = 7\n";
    ExtrusionEntityUPtr copy;
    extrusion_data_id next_id = EXTRUSION_DATA_ID_INVALID;
    {
        ExtrusionPath source(ExtrusionAttributes{ExtrusionRole::Perimeter});
        ArcPolyline polyline;
        polyline.append(Point(1000, 0));
        polyline.append(Geometry::ArcWelder::Segment(Point(0, 1000), 1000.f, Geometry::ArcWelder::Orientation::CCW));
        polyline.set_z_offset(0, 100);
        polyline.set_z_offset(1, 900);
        source.set_polyline(std::move(polyline));
        source.set_can_rotate(false);
        ExtrusionPropertyCustomGcode &script = source.add_property(ExtrusionPropertyCustomGcodeText(
            ExtrusionPropertyCustomGcode::Code::SCRIPT, text, GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM));
        REQUIRE(Access::store_property_data_aligned(source, ExtrusionPropertyCustomGcode::property_type,
            &script.config_id, config.c_str(), config.size() + 1, 1) != EXTRUSION_DATA_ID_INVALID);
        Payload *payload = static_cast<Payload *>(Access::get_or_add_property_data_mutable(source, custom_type, sizeof(Payload), alignof(Payload)));
        REQUIRE(payload != nullptr);
        payload->marker = 123456789;
        REQUIRE(Access::store_property_data_aligned(source, custom_type, &payload->data_id,
            expected.data(), sizeof(expected), 64) != EXTRUSION_DATA_ID_INVALID);
        const uint32_t discarded = Access::store_data_aligned(source, expected.data(), sizeof(expected), 64);
        REQUIRE(discarded != EXTRUSION_DATA_ID_INVALID);
        REQUIRE(Access::free_data(source, discarded));
        next_id = discarded + 1;
        copy = source.clone_canonical();
        check_canonical_copy(source, *copy);
        const Payload *copied_payload = static_cast<const Payload *>(Access::property_data(*copy, custom_type));
        REQUIRE(copied_payload != nullptr);
        CHECK(copied_payload != payload);
        uint32_t size = 0;
        CHECK(Access::stored_data(*copy, copied_payload->data_id, &size) != Access::stored_data(source, payload->data_id, nullptr));
        payload->marker = 42;
        CHECK(copied_payload->marker == 123456789);
    }
    const Payload *payload = static_cast<const Payload *>(Access::property_data(*copy, custom_type));
    REQUIRE(payload != nullptr);
    CHECK(reinterpret_cast<uintptr_t>(payload) % 64 == 0);
    CHECK(payload->marker == 123456789);
    uint32_t size = 0;
    const void *buffer = Access::stored_data(*copy, payload->data_id, &size);
    REQUIRE(buffer != nullptr);
    CHECK(size == sizeof(expected));
    CHECK(reinterpret_cast<uintptr_t>(buffer) % 64 == 0);
    CHECK(std::memcmp(buffer, expected.data(), sizeof(expected)) == 0);
    const ExtrusionPropertyCustomGcode *script = copy->get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(script != nullptr);
    CHECK(script->code() == ExtrusionPropertyCustomGcode::Code::SCRIPT);
    CHECK(script->script_type == GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM);
    CHECK(copy->custom_gcode_string(*script) == text);
    const char *arguments = static_cast<const char *>(Access::stored_data(*copy, script->config_id, &size));
    REQUIRE(arguments != nullptr);
    CHECK(std::string(arguments) == config);
    CHECK(copy->polyline_ref().get_point(0) == Point(1000, 0));
    CHECK(copy->polyline_ref().get_point(1) == Point(0, 1000));
    CHECK(copy->polyline_ref().z_offset(0) == 100);
    CHECK(copy->polyline_ref().z_offset(1) == 900);
    CHECK(copy->polyline_ref().get_arc().back().radius == 1000.f);
    CHECK_FALSE(copy->can_rotate());
    CHECK(Access::store_data_aligned(*copy, expected.data(), sizeof(expected), 64) == next_id);
}

// Keep positioned commands as exact Nop nodes, including their sentinel and any accepted base children.
TEST_CASE("Canonical Nop copies retain command state and normalize their children",
          "[plugins][extrusion][canonical-copy]")
{
    using namespace Slic3r;
    for (const bool positioned : {false, true}) {
        for (const bool allowed : {false, true}) {
            ExtrusionNop source(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::EXTRUSION, -17.5));
            source.set_role(ExtrusionRole::WipeTowerWipe);
            source.set_can_rotate(allowed);
            if (positioned)
                source.position = Point(1200, 3400);
            std::unique_ptr<ExtrusionPath> child = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Travel});
            child->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}}));
            source.append_child(std::move(child));
            source.add_property(ExtrusionPropertyCustomGcodeText("M117 Nop command"));
            ExtrusionEntityUPtr copy = source.clone_canonical();
            check_canonical_copy(source, *copy);
            REQUIRE(typeid(*copy) == typeid(ExtrusionNop));
            ExtrusionNop &nop = static_cast<ExtrusionNop &>(*copy);
            CHECK(nop.first_point() == source.position);
            CHECK(nop.last_point() == source.position);
            CHECK(nop.empty());
            CHECK(nop.length() == 0.);
            const ExtrusionPropertySpecialCommand *command = nop.get_property<ExtrusionPropertySpecialCommand>();
            REQUIRE(command != nullptr);
            CHECK(command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION);
            CHECK(command->extra_data == -17.5);
            if (nop.position != ExtrusionNop::NOT_A_POINT)
                nop.position += Point(100, 200);
            CHECK(nop.position == (positioned ? Point(1300, 3600) : ExtrusionNop::NOT_A_POINT));
            CHECK(source.position == (positioned ? Point(1200, 3400) : ExtrusionNop::NOT_A_POINT));
            CHECK(nop.can_rotate() == allowed);
        }
    }
}

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
