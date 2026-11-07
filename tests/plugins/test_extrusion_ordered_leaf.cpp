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
        |-- check_canonical_copy(): compare owned copies recursively by content
        `-- check_ordinary_copy(): retain native descendants and independent resources

Canonical-copy tests compare stored geometry and properties recursively, without
requiring the virtual behavior of legacy path and collection views.
SDK copies are alias-safe and canonical; generic SDK moves remain distinct from
fresh plan acquisitions. Command state is checked through properties and owned
resources rather than legacy Nop position or role fields.
Full-unit reversal probes compare native and SDK operations on fixed descendants,
command phases and arc/Z paths, then reverse again to check exact restoration.
Seam plans are inspected before any materialization: local paths, nested sole
contours, open atomic composites and event barriers must offer only legal entries
without editing geometry, endpoint Z, identities or permissions. Native split
previews distinguish the rounded input from the exact point they will publish.
Ordinary-copy tests retain native descendants and distinguish root destruction effects.
Loop-builder probes reject short polygons before invariant checks and preserve
borrowed point arrays, while consuming overloads clear all processed inputs.
Direct overhang tests compare copied native descendants and the exact fields of numeric fragments.
Flattening tests distinguish whole-wrapper appends from child prefixes. Donation materializes local geometry as an owned child.
*/

#include <catch2/catch.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <typeinfo>
#include <stdexcept>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_entity.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntityOrdering.hpp"
#include "libslic3r/ExtrusionSeamPlanning.hpp"
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/ExtrusionSourcePublication.hpp"
#include "libslic3r/NativeAdhesionLayerHelpers.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/GCode/ExtrusionProcessor.hpp"

namespace {

Slic3r::ExtrusionEntityUPtr empty_child();
extrusion_entity_handle *entity_handle(Slic3r::ExtrusionEntity &entity);
slic3r_api::MutableExtrusionEntity mutable_view(Slic3r::ExtrusionEntity &entity);
// Compare stored nodes and exact copied types without entering legacy path views.
void check_canonical_copy(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &copy);
// Compare independently owned stored state, retaining the dynamic types of descendants but not requiring the same root shell.
void check_ordinary_copy(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &copy);
// Build a native fixture path with optional dynamic overhang fields and an explicit Z profile.
Slic3r::ExtrusionPath overhang_fixture_path(const Slic3r::Points &points, bool dynamic, bool with_z);
// Record the flat native result's full attributes, overhang fields and geometry with trace-local identity names.
// Reject other properties or shapes rather than silently omitting state; inputs remain borrowed and unchanged.
std::string overhang_fragment_state(const Slic3r::ExtrusionEntityCollection &result,
                                    const std::array<uint64_t, 2> &source_ids);
// Copy borrowed points without modifying them into an owned native Path with complete attributes; construction failures propagate.
Slic3r::ExtrusionPath flatten_fixture_path(const Slic3r::Points &points);
// Return an owned discontinuous native tree with the requested root sorting permission; construction failures propagate.
// Its fixed nested group has disjoint paths, so flattening must traverse it rather than copy it atomically.
Slic3r::ExtrusionEntityCollection flatten_fixture(bool sortable);
// Emit REQUIREs for a multi-child source collection with properties and for the rebuilt Collection's type.
// Emit CHECKs for distinct addresses/IDs, matching permissions and absent group properties; neither borrowed input is modified.
void check_rebuilt_flatten_group(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &group);

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
    REQUIRE(typeid(copy) == typeid(ExtrusionEntity));
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

TEST_CASE("Source publication retains property scopes for legacy and canonical transport roots",
          "[plugins][extrusion][consumer-qualification][canonical-ingress]")
{
    using namespace Slic3r;
    for (const bool canonical : {false, true}) {
        CAPTURE(canonical);
        std::unique_ptr<ExtrusionEntityCollection> native = std::make_unique<ExtrusionEntityCollection>(false, true);
        native->add_property(ExtrusionPropertyCustomGcodeText("M117 source scope"));
        native->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
        path->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}}));
        native->append(std::move(path));
        ExtrusionEntityUPtr source = canonical ? native->clone_canonical() : ExtrusionEntityUPtr(native.release());
        const ExtrusionEntityUPtr before = source->clone_canonical();
        ExtrusionEntityCollection copied;
        Slic3r::detail::append_adhesion_extrusion_copy(copied, *source);
        REQUIRE(copied.child_count() == 1);
        const ExtrusionEntityUPtr copy = copied.child(0).clone_canonical();
        check_canonical_copy(*before, *copy);
        CHECK(source->has_properties());
        ExtrusionEntity destination(ExtrusionEntity::Children(), false, false, false);
        append_source_extrusions(destination, *source);
        REQUIRE(destination.child_count() == 1);
        const ExtrusionEntityUPtr moved = destination.child(0).clone_canonical();
        check_canonical_copy(*before, *moved);
        CHECK(source->ExtrusionEntity::empty());
        CHECK_FALSE(source->has_properties());
        const ExtrusionPropertyCustomGcode *text = destination.child(0).get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(text != nullptr);
        CHECK(destination.child(0).custom_gcode_string(*text) == "M117 source scope");
    }
    ExtrusionEntity command(false);
    command.add_property(ExtrusionPropertyCustomGcodeText("M117 geometry-less command"));
    ExtrusionEntityCollection copied;
    Slic3r::detail::append_adhesion_extrusion_copy(copied, command);
    CHECK(copied.child_count() == 1);
    ExtrusionEntity moved(ExtrusionEntity::Children(), false, false, false);
    append_source_extrusions(moved, command);
    REQUIRE(moved.child_count() == 1);
    CHECK(moved.child(0).is_nop());
    CHECK(moved.child(0).has_properties());
    CHECK_FALSE(command.has_properties());
}

TEST_CASE("SDK copies publish canonical content and prepare aliases before replacement",
          "[plugins][extrusion][canonical-ingress]")
{
    using namespace Slic3r;
    const auto make_source = [] {
        std::unique_ptr<ExtrusionEntityCollection> root = std::make_unique<ExtrusionEntityCollection>(false, true);
        root->add_property(ExtrusionPropertyCustomGcodeText("M117 copied root"));
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
        path->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}}));
        path->set_can_rotate(false);
        root->append(std::move(path));
        ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
        event->add_property(ExtrusionPropertyCustomGcodeText("M117 copied event"));
        root->append(std::move(event));
        root->set_can_rotate(false);
        return root;
    };
    PluginStorage storage;
    std::unique_ptr<ExtrusionEntityCollection> source = make_source();
    const ExtrusionEntity *source_child = &source->child(0);
    extrusion_entity_handle *cloned = extrusion_clone(reinterpret_cast<storage_handle *>(&storage), entity_handle(*source));
    REQUIRE(cloned != nullptr);
    const ExtrusionEntity &copy = *reinterpret_cast<const ExtrusionEntity *>(cloned);
    CHECK(CanonicalExtrusion::is_tree(copy));
    check_canonical_copy(*source, copy);
    CHECK(&source->child(0) == source_child);
    CHECK(typeid(source->child(0)) == typeid(ExtrusionPath));
    REQUIRE(copy.get_property<ExtrusionPropertyCustomGcode>() != nullptr);
    CHECK(copy.custom_gcode_string(*copy.get_property<ExtrusionPropertyCustomGcode>()) == "M117 copied root");
    ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, false);
    parent.append_child(std::make_unique<ExtrusionEntity>(false));
    const ExtrusionEntity *sibling = &parent.child(0);
    REQUIRE(extrusion_insert_child_copy(entity_handle(parent), 1, entity_handle(*source)) == 1);
    CHECK(&parent.child(0) == sibling);
    CHECK(CanonicalExtrusion::is_tree(parent.child(1)));
    check_canonical_copy(*source, parent.child(1));
    CHECK(extrusion_insert_child_copy(entity_handle(parent), 99, entity_handle(*source)) == EXTRUSION_INDEX_INVALID);
    CHECK(parent.child_count() == 2);
    for (const int alias : {0, 1, 2}) {
        CAPTURE(alias);
        std::unique_ptr<ExtrusionEntityCollection> owner = make_source();
        ExtrusionEntity *target = alias == 2 ? &owner->child(0) : owner.get();
        const ExtrusionEntity *input = alias == 1 ? &owner->child(0) : owner.get();
        const ExtrusionEntityUPtr expected = input->clone_canonical();
        const ExtrusionEntity *stable = target;
        REQUIRE(extrusion_copy_from(entity_handle(*target), reinterpret_cast<const extrusion_entity_handle *>(input)) != 0);
        CHECK(target == stable);
        const ExtrusionEntityUPtr actual = target->clone_canonical();
        check_canonical_copy(*expected, *actual);
        for (const ExtrusionEntityUPtr &child : static_cast<const ExtrusionEntity &>(*target).children())
            if (child) CHECK(CanonicalExtrusion::is_tree(*child));
        CHECK(extrusion_copy_from(entity_handle(*target), nullptr) == 0);
        const ExtrusionEntityUPtr after_rejection = target->clone_canonical();
        check_canonical_copy(*expected, *after_rejection);
    }
}

TEST_CASE("Generic SDK moves keep legacy children and their handles outside plan acquisition",
          "[plugins][extrusion][canonical-ingress]")
{
    using namespace Slic3r;
    ExtrusionEntity source(ExtrusionEntity::Children(), false, true, true);
    std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
    path->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}}));
    const ExtrusionEntity *identity = path.get();
    source.append_child(std::move(path));
    ExtrusionEntity target(false);
    REQUIRE(extrusion_move_from(entity_handle(target), entity_handle(source)) != 0);
    REQUIRE(target.child_count() == 1);
    CHECK(&target.child(0) == identity);
    CHECK(typeid(target.child(0)) == typeid(ExtrusionPath));
    CHECK(source.ExtrusionEntity::empty());
    CHECK_FALSE(CanonicalExtrusion::is_tree(target));
}

TEST_CASE("Fresh canonical acquisitions preserve command properties resources and base child handles",
          "[plugins][extrusion][canonical-copy][canonical-transfer]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const bool nop_root : {false, true}) {
        for (const bool legacy_child : {false, true}) {
            CAPTURE(nop_root, legacy_child);
            ExtrusionEntityUPtr source;
            if (nop_root) {
                std::unique_ptr<ExtrusionNop> nop = std::make_unique<ExtrusionNop>();
                nop->position = Point(1234, 5678);
                nop->set_role(ExtrusionRole::WipeTowerWipe);
                source = std::move(nop);
            } else source = std::make_unique<ExtrusionEntity>(true);
            ExtrusionEntityUPtr child;
            if (legacy_child) child = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter});
            else child = std::make_unique<ExtrusionEntity>(true);
            child->set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}}));
            const ExtrusionEntity *child_handle = child.get();
            source->append_child(std::move(child));
            source->set_can_sort_reverse(false, true);
            source->set_can_rotate(false);
            source->add_property(ExtrusionPropertyCustomGcodeText("M117 owned transfer"));
            source->add_property(ExtrusionAttributes{ExtrusionRole::SupportMaterial});
            const std::array<uint64_t, 4> resource{{11, 22, 33, 44}};
            const uint32_t detached = Access::store_data_aligned(*source, resource.data(), sizeof(resource), 64);
            REQUIRE(detached != EXTRUSION_DATA_ID_INVALID);
            const ExtrusionEntityUPtr before = source->clone_canonical();
            ExtrusionEntityUPtr acquired = CanonicalExtrusion::acquire(*source);
            REQUIRE(acquired != nullptr);
            CHECK(CanonicalExtrusion::is_tree(*acquired));
            check_canonical_copy(*before, *acquired);
            CHECK((&acquired->child(0) == child_handle) == (!legacy_child && !nop_root));
            CHECK(source->ExtrusionEntity::empty());
            CHECK_FALSE(source->has_properties());
            CHECK(Access::stored_data(*source, detached, nullptr) == nullptr);
            REQUIRE(typeid(*acquired) == typeid(ExtrusionEntity));
            REQUIRE(acquired->get_property<ExtrusionAttributes>() != nullptr);
            CHECK(acquired->get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::SupportMaterial);
            uint32_t size = 0;
            const void *data = Access::stored_data(*acquired, detached, &size);
            REQUIRE(data != nullptr);
            CHECK(size == sizeof(resource));
            CHECK(reinterpret_cast<uintptr_t>(data) % 64 == 0);
            CHECK(std::memcmp(data, resource.data(), sizeof(resource)) == 0);
            source.reset();
            REQUIRE(acquired->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(acquired->custom_gcode_string(*acquired->get_property<ExtrusionPropertyCustomGcode>()) == "M117 owned transfer");
        }
    }
}

TEST_CASE("Source simplification measures stored paths and never prunes command scopes",
          "[plugins][extrusion][consumer-qualification]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    const ConfigOptionFloatOrPercent tolerance(0.01, false);
    for (const bool canonical : {false, true}) {
        for (const bool positioned : {false, true}) {
            CAPTURE(canonical, positioned);
            std::unique_ptr<ExtrusionNop> nop = std::make_unique<ExtrusionNop>(ExtrusionPropertyCustomGcodeText("M117 keep short scope"));
            if (positioned) nop->position = Point(11, 22);
            nop->set_role(ExtrusionRole::SupportMaterial);
            ArcPolyline arc;
            arc.append(Point(10000, 0));
            arc.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
            arc.set_z_offset(0, 10);
            arc.set_z_offset(1, 20);
            nop->set_polyline(arc);
            nop->add_property(ExtrusionAttributes{ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)});
            ExtrusionEntity root(ExtrusionEntity::Children(), true, true, false);
            root.append_child(std::move(nop));
            ExtrusionEntityUPtr tree = canonical ? root.clone_canonical() : ExtrusionEntityUPtr(root.clone());
            const ExtrusionEntityUPtr before = tree->clone_canonical();
            const ExtrusionEntity *handle = &tree->child(0);
            SimplifyVisitor visitor(SCALED_EPSILON, ArcFittingType::Disabled, false, &tolerance, 1000000);
            visitor.traverse(*tree);
            REQUIRE(tree->child_count() == 1);
            CHECK(&tree->child(0) == handle);
            ExtrusionEntityUPtr after = tree->clone_canonical();
            check_canonical_copy(*before, *after);
        }
    }
    for (const bool unknown : {false, true}) {
        CAPTURE(unknown);
        ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
        root.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)});
        root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {1000, 0}})));
        ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
        if (unknown) {
            uint64_t *data = static_cast<uint64_t *>(Access::get_or_add_property_data_mutable(
                *event, extrusion_property_type(0x7fff0102), sizeof(uint64_t), alignof(uint64_t)));
            REQUIRE(data != nullptr);
            *data = 0x123456789;
        } else event->add_property(ExtrusionPropertyCustomGcodeText("M117 middle phase"));
        root.append_child(std::move(event));
        root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{1000, 0}, {2000, 0}})));
        const ExtrusionEntityUPtr before = root.clone_canonical();
        const ExtrusionEntity *middle = &root.child(1);
        SimplifyVisitor visitor(SCALED_EPSILON, ArcFittingType::Disabled, false, &tolerance, 10000);
        visitor.traverse(root);
        REQUIRE(root.child_count() == 3);
        CHECK(&root.child(1) == middle);
        CHECK(visitor.is_valid());
        ExtrusionEntityUPtr after = root.clone_canonical();
        check_canonical_copy(*before, *after);
    }
    for (const bool script : {false, true}) {
        CAPTURE(script);
        ExtrusionEntity stateful(ExtrusionEntity::Children(), true, true, false);
        ExtrusionEntityUPtr path = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {1000, 0}}));
        path->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        if (script) path->add_property(ExtrusionPropertyCustomGcodeText("M117 nonempty state"));
        else {
            uint64_t *data = static_cast<uint64_t *>(Access::get_or_add_property_data_mutable(
                *path, extrusion_property_type(0x7fff0103), sizeof(uint64_t), alignof(uint64_t)));
            REQUIRE(data != nullptr);
            *data = 12345;
        }
        stateful.append_child(std::move(path));
        const ExtrusionEntityUPtr before = stateful.clone_canonical();
        SimplifyVisitor same_visitor(SCALED_EPSILON, ArcFittingType::Disabled, false, &tolerance, 10000);
        same_visitor.traverse(stateful);
        ExtrusionEntityUPtr after = stateful.clone_canonical();
        check_canonical_copy(*before, *after);
        if (script) CHECK(stateful.child(0).custom_gcode_string(
            *stateful.child(0).get_property<ExtrusionPropertyCustomGcode>()) == "M117 nonempty state");
        ExtrusionEntity next_pure(ExtrusionEntity::Children(), true, true, false);
        ExtrusionEntityUPtr small = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {1000, 0}}));
        small->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        next_pure.append_child(std::move(small));
        same_visitor.traverse(next_pure);
        CHECK(next_pure.child_count() == 0);
    }
    // The historical small-path policy remains active for pure generated geometry.
    ExtrusionEntity pure(ExtrusionEntity::Children(), true, true, false);
    ExtrusionEntityUPtr short_path = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {1000, 0}}));
    short_path->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    pure.append_child(std::move(short_path));
    SimplifyVisitor visitor(SCALED_EPSILON, ArcFittingType::Disabled, false, &tolerance, 10000);
    visitor.traverse(pure);
    CHECK(pure.child_count() == 0);

    // A composite geometric neighbour is not a command and cannot be merged
    // through a local-polyline API. Keep the small leaf instead of propagating
    // its deletion marker to an ancestor that would discard the whole contour.
    ExtrusionEntity outer(ExtrusionEntity::Children(), true, true, false);
    ExtrusionEntityUPtr sequence = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    ExtrusionEntityUPtr short_first = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {1000, 0}}));
    short_first->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    sequence->append_child(std::move(short_first));
    ExtrusionEntityUPtr composite = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    ExtrusionEntityUPtr long_last = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{1000, 0}, {20000, 0}}));
    long_last->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
    composite->append_child(std::move(long_last));
    sequence->append_child(std::move(composite));
    outer.append_child(std::move(sequence));
    const ExtrusionEntityUPtr before_composite = outer.clone_canonical();
    visitor.traverse(outer);
    const ExtrusionEntityUPtr after_composite = outer.clone_canonical();
    check_canonical_copy(*before_composite, *after_composite);
}

// Root permission authorizes the whole unit, including fixed descendants and commands.
TEST_CASE("Reversing an authorized unit reverses every stored node without command exceptions",
          "[plugins][extrusion][tree-reverse]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const std::string form : {"base", "collection", "loop", "multipath", "path", "nop"}) {
        for (const bool sdk : {false, true}) {
            CAPTURE(form, sdk);
            ExtrusionEntityUPtr root;
            if (form == "collection") root = std::make_unique<ExtrusionEntityCollection>();
            else if (form == "loop") root = std::make_unique<ExtrusionLoop>();
            else if (form == "multipath") root = std::make_unique<ExtrusionMultiPath>();
            else if (form == "path") root = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter});
            else if (form == "nop") root = std::make_unique<ExtrusionNop>();
            else root = std::make_unique<ExtrusionEntity>(true);
            root->set_can_sort_reverse(false, true);
            root->set_can_rotate(false);
            if (ExtrusionNop *nop = dynamic_cast<ExtrusionNop *>(root.get())) {
                nop->position = Point(77, 88);
                nop->set_role(ExtrusionRole::SupportMaterial);
            }
            ExtrusionEntityUPtr first = std::make_unique<ExtrusionEntity>(false);
            first->add_property(ExtrusionPropertyCustomGcodeText("M117 first phase"));
            root->append_child(std::move(first));
            std::unique_ptr<ExtrusionNop> nested = std::make_unique<ExtrusionNop>();
            nested->position = Point(123, 456);
            nested->set_can_sort_reverse(false, false);
            nested->set_can_rotate(false);
            ArcPolyline arc;
            arc.append(Point(10000, 0));
            arc.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
            arc.set_z_offset(0, 100);
            arc.set_z_offset(1, 900);
            nested->append_child(std::make_unique<ExtrusionEntity>(false, arc));
            ExtrusionEntityUPtr event = std::make_unique<ExtrusionEntity>(false);
            for (const extrusion_property_type type : {EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND, EXTRUSION_PROPERTY_TYPE_HEATER,
                                     EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS})
                REQUIRE(Access::get_or_add_property_data_mutable(*event, type, 32, alignof(uint64_t)) != nullptr);
            nested->append_child(std::move(event));
            nested->set_can_sort_reverse(false, false);
            root->append_child(std::move(nested));
            ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
            path.set_polyline(ArcPolyline(Points{{0, 0}, {5000, 0}, {0, 5000}, {0, 0}}));
            std::unique_ptr<ExtrusionLoop> loop = std::make_unique<ExtrusionLoop>(path);
            loop->set_can_sort_reverse(false, false);
            root->append_child(std::move(loop));
            // Structural materialization may enable sorting; authorize only
            // reversal on the completed unit, not on its descendants.
            root->set_can_sort_reverse(false, true);
            REQUIRE_FALSE(root->can_sort());
            if (ExtrusionEntityCollection *collection = dynamic_cast<ExtrusionEntityCollection *>(root.get()))
                (void)collection->entities();
            const ExtrusionEntityUPtr before = root->clone_canonical();
            std::vector<const ExtrusionEntity *> handles;
            const auto collect = [&](const auto &self, const ExtrusionEntity &node) -> void {
                handles.push_back(&node);
                for (const ExtrusionEntityUPtr &child : node.children()) if (child) self(self, *child);
            };
            collect(collect, *root);
            REQUIRE(root->can_reverse());
            if (sdk) slic3r_api::Ordering::TreeOrderingGeometry::reverse_tree(mutable_view(*root));
            else root->reverse();
            const auto check = [&](const auto &self, const ExtrusionEntity &original, const ExtrusionEntity &actual) -> void {
                CHECK(std::find(handles.begin(), handles.end(), &actual) != handles.end());
                CHECK(actual.get_id() == original.get_id());
                CHECK(actual.can_sort() == original.can_sort());
                CHECK(actual.can_reverse() == original.can_reverse());
                CHECK(actual.can_rotate() == original.can_rotate());
                if (const ExtrusionNop *nop = dynamic_cast<const ExtrusionNop *>(&original)) {
                    const ExtrusionNop *result = dynamic_cast<const ExtrusionNop *>(&actual);
                    REQUIRE(result != nullptr);
                    CHECK(result->position == nop->position);
                    CHECK(result->role() == nop->role());
                }
                REQUIRE(Access::property_count(actual) == Access::property_count(original));
                for (size_t i = 0; i < Access::property_count(original); ++i)
                    CHECK(Access::same_property_payload(actual, Access::property_type_at(original, i), original));
                if (original.has_polyline()) {
                    ArcPolyline expected = original.polyline_ref();
                    expected.reverse();
                    REQUIRE(actual.has_polyline());
                    const ArcPolyline &poly = actual.polyline_ref();
                    REQUIRE(poly.size() == expected.size());
                    CHECK(poly.has_arc() == expected.has_arc());
                    CHECK(poly.has_z_offset() == expected.has_z_offset());
                    for (size_t i = 0; i < poly.size(); ++i) {
                        CHECK(poly.get_point(i) == expected.get_point(i));
                        if (poly.has_z_offset()) CHECK(poly.z_offset(i) == expected.z_offset(i));
                        if (poly.has_arc()) {
                            CHECK(poly.get_arc(i).radius == expected.get_arc(i).radius);
                            CHECK(poly.get_arc(i).orientation == expected.get_arc(i).orientation);
                        }
                    }
                }
                REQUIRE(actual.child_count() == original.child_count());
                for (size_t i = 0; i < original.child_count(); ++i)
                    self(self, original.child(i), actual.child(original.child_count() - 1 - i));
            };
            check(check, *before, *root);
            const ExtrusionEntity &last = root->child(root->child_count() - 1);
            REQUIRE(last.get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(last.custom_gcode_string(*last.get_property<ExtrusionPropertyCustomGcode>()) == "M117 first phase");
            if (ExtrusionEntityCollection *collection = dynamic_cast<ExtrusionEntityCollection *>(root.get())) {
                REQUIRE(collection->entities().size() == root->child_count());
                CHECK(collection->entities().front() == &root->child(0));
            }
            if (sdk) slic3r_api::Ordering::TreeOrderingGeometry::reverse_tree(mutable_view(*root));
            else root->reverse();
            ExtrusionEntityUPtr restored = root->clone_canonical();
            check_canonical_copy(*before, *restored);
        }
    }
}

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

// Child seam eligibility follows the actual stored continuity, not a legacy type or an inherited permission bit.
TEST_CASE("Seam descent preserves fixed contour entries and continuous connections",
          "[plugins][extrusion][seam-unit-preparation]")
{
    using namespace Slic3r;
    ExtrusionPath loop_path(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
    loop_path.set_polyline(ArcPolyline(Points{{0, 0}, {0, 10000}, {10000, 10000}, {0, 0}}));
    ExtrusionMultiPath fixed_contour(loop_path);
    const ExtrusionEntity &fixed_view = fixed_contour;
    REQUIRE(is_seam_unit(fixed_view.child(0)));
    CHECK_FALSE(can_choose_child_seam(fixed_view, fixed_view.child(0), true));
    CHECK(fixed_view.child(0).can_rotate());
    ExtrusionEntityUPtr fixed_copy = fixed_contour.clone_canonical();
    CHECK_FALSE(can_choose_child_seam(*fixed_copy, fixed_copy->child(0), true));

    ExtrusionEntity wrapper(ExtrusionEntity::Children(), false, false, true);
    wrapper.set_can_rotate(false);
    wrapper.append_child(std::make_unique<ExtrusionEntity>(false));
    wrapper.append_child(std::make_unique<ExtrusionPath>(loop_path));
    const ExtrusionEntity &wrapper_view = wrapper;
    CHECK(can_choose_child_seam(wrapper_view, wrapper_view.child(1), true));
    CHECK_FALSE(can_choose_child_seam(wrapper_view, wrapper_view.child(1), false));
    ExtrusionEntityUPtr wrapper_copy = wrapper.clone_canonical();
    CHECK(can_choose_child_seam(*wrapper_copy, wrapper_copy->child(1), true));

    ExtrusionEntity connected(ExtrusionEntity::Children(), false, false, true);
    connected.set_can_rotate(false);
    connected.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{-10000, 0}, {0, 0}})));
    connected.append_child(std::move(wrapper_copy));
    connected.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {20000, 0}})));
    const ExtrusionEntity &connected_view = connected;
    REQUIRE(connected_view.is_continuous());
    const bool child_context = can_choose_child_seam(connected_view, connected_view.child(1), true);
    CHECK_FALSE(child_context);
    CHECK_FALSE(can_choose_child_seam(connected_view.child(1), connected_view.child(1).child(1), child_context));
    CHECK(is_seam_unit(connected_view.child(1).child(1)));

    ExtrusionEntity separated(ExtrusionEntity::Children(), false, false, true);
    separated.set_can_rotate(false);
    separated.append_child(std::make_unique<ExtrusionPath>(loop_path));
    loop_path.polyline().translate(Point(50000, 0));
    separated.append_child(std::make_unique<ExtrusionPath>(loop_path));
    const ExtrusionEntity &separated_view = separated;
    REQUIRE_FALSE(separated_view.is_continuous());
    CHECK(can_choose_child_seam(separated_view, separated_view.child(0), true));
    CHECK(can_choose_child_seam(separated_view, separated_view.child(1), true));
}

// Every one-shot property type is detected on a leaf and makes its parent a seam barrier.
TEST_CASE("One shot seam events make their parent a seam barrier",
          "[plugins][extrusion][seam-unit-preparation]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    struct PropertySpec { extrusion_property_type type; size_t size; size_t alignment; };
    const PropertySpec properties[] = {
        {EXTRUSION_PROPERTY_TYPE_CUSTOM_GCODE, sizeof(c_extrusion_property_custom_gcode), alignof(c_extrusion_property_custom_gcode)},
        {EXTRUSION_PROPERTY_TYPE_SPECIAL_COMMAND, sizeof(c_extrusion_property_special_command), alignof(c_extrusion_property_special_command)},
        {EXTRUSION_PROPERTY_TYPE_HEATER, sizeof(c_extrusion_property_heater), alignof(c_extrusion_property_heater)},
        {EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS, sizeof(c_extrusion_property_extrusion_axis), alignof(c_extrusion_property_extrusion_axis)}
    };
    for (const PropertySpec &property : properties) {
        CAPTURE(property.type);
        ExtrusionEntity leaf(false, ArcPolyline(Points{{0, 0}, {10000, 0}}));
        REQUIRE_FALSE(has_one_shot_extrusion_event(leaf));
        void *const event = Access::get_or_add_property_data_mutable(leaf, property.type, property.size, property.alignment);
        REQUIRE(event != nullptr);
        CHECK(has_one_shot_extrusion_event(leaf));
        ExtrusionEntity parent(ExtrusionEntity::Children(), false, false, true);
        parent.append_child(leaf.clone_canonical());
        CHECK(has_seam_barrier(parent));
    }
}

// Empty opaque events block parent reordering, while a sole closed child may still rotate in place after delegation.
TEST_CASE("Seam barriers distinguish geometry from empty property events",
          "[plugins][extrusion][seam-unit-preparation]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}, {0, 0}})));
    root.child(0).add_property(ExtrusionPropertySpeed(25.f));
    CHECK_FALSE(has_seam_barrier(root));
    root.append_child(std::make_unique<ExtrusionEntity>(false));
    CHECK_FALSE(has_seam_barrier(root));
    REQUIRE(Access::get_or_add_property_data_mutable(root.child(1), SLIC3R_PROPERTY_TYPE_CUSTOM_BEGIN + 932,
                                                    sizeof(uint32_t), alignof(uint32_t)) != nullptr);
    CHECK(has_seam_barrier(root));
    const ExtrusionEntity &view = root;
    CHECK(is_seam_unit(view));
    CHECK(can_choose_child_seam(view, view.child(0), true));
    root.set_can_rotate(false);
    CHECK(can_choose_child_seam(view, view.child(0), true));
    CHECK_FALSE(is_seam_unit(view));
}

// Planning validates the original request before legalizing it, and never consumes any permission.
TEST_CASE("Legal seam plans validate local geometry and keep exact membership separate",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    using Operation = ExtrusionSeamPlan::Operation;
    ExtrusionEntity loop(false, ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}}));
    const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(loop, Point(5000, 0), true);
    REQUIRE(plan);
    CHECK(plan->point == Point(5000, 0));
    CHECK(plan->operation == Operation::RotateLocal);
    CHECK(plan->delegation.empty());
    CHECK(is_directly_legal_seam(loop, Point(5000, 0), true));
    const std::optional<ExtrusionSeamPlan> near = legal_seam_entry(loop, Point(1, 0), true);
    REQUIRE(near);
    CHECK(near->point == Point(0, 0));
    CHECK(near->operation == Operation::KeepStart);
    CHECK_FALSE(is_directly_legal_seam(loop, Point(1, 0), true));
    CHECK_FALSE(legal_seam_entry(loop, Point(5000, -2 * SCALED_EPSILON), true));
    CHECK_FALSE(legal_seam_entry(loop, Point(5000, 0), false));
    CHECK(loop.can_rotate());
    CHECK(loop.polyline_ref().front() == Point(0, 0));
    CHECK(loop.polyline_ref().size() == 4);
    loop.set_can_rotate(false);
    CHECK_FALSE(legal_seam_entry(loop, Point(5000, 0), true));
    ExtrusionLoop empty;
    CHECK_FALSE(legal_seam_entry(empty, Point(0, 0), true));
    ExtrusionEntity positioned(false, ArcPolyline(Points{{123, 456}}));
    const std::optional<ExtrusionSeamPlan> stationary = legal_seam_entry(positioned, Point(123, 456), true);
    REQUIRE(stationary);
    CHECK(stationary->operation == Operation::KeepStart);
}

// A single closed child can carry the parent's seam without moving property-only siblings or adding fragments.
TEST_CASE("Legal seam plans delegate only to the sole eligible contour",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    using Operation = ExtrusionSeamPlan::Operation;
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false));
    root.child(0).add_property(ExtrusionPropertySpeed(25.f));
    std::unique_ptr<ExtrusionEntity> nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    nested->append_child(std::make_unique<ExtrusionEntity>(false,
        ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}})));
    root.append_child(std::move(nested));
    const ExtrusionEntity *event = &root.child(0);
    const ExtrusionEntity *contour = &root.child(1).child(0);
    for (const Point requested : {Point(5000, 0), Point(10000, 5000)}) {
        const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(root, requested, true);
        REQUIRE(plan);
        CHECK(plan->point == requested);
        CHECK(plan->operation == Operation::RotateLocal);
        CHECK(plan->delegation == (std::vector<size_t>{1, 0}));
        CHECK(root.child_count() == 2);
        CHECK(root.child(1).child_count() == 1);
        CHECK(&root.child(0) == event);
        CHECK(&root.child(1).child(0) == contour);
        CHECK(contour->can_rotate());
    }
    ExtrusionEntityUPtr copy = root.clone_canonical();
    const std::optional<ExtrusionSeamPlan> copied = legal_seam_entry(*copy, Point(5000, 0), true);
    REQUIRE(copied);
    CHECK(copied->delegation == (std::vector<size_t>{1, 0}));
    root.child(1).set_can_rotate(false);
    const std::optional<ExtrusionSeamPlan> protected_child = legal_seam_entry(root, Point(5000, 0), true);
    REQUIRE(protected_child);
    CHECK(protected_child->point == Point(0, 0));
    CHECK(protected_child->operation == Operation::KeepStart);
    CHECK(protected_child->delegation.empty());
}

// An interior seam of an open composite is not an interior seam of the parent's cycle.
TEST_CASE("Legal seam plans keep open composite children atomic",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    using Operation = ExtrusionSeamPlan::Operation;
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
    std::unique_ptr<ExtrusionEntity> open = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    open->set_can_rotate(false);
    open->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {10000, 10000}})));
    open->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 10000}, {0, 10000}})));
    root.append_child(std::move(open));
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 10000}, {0, 0}})));
    REQUIRE(is_seam_unit(root));
    const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(root, Point(10000, 5000), true);
    REQUIRE(plan);
    CHECK(plan->point == Point(10000, 0));
    CHECK(plan->operation == Operation::RotateChildren);
    CHECK(plan->child_index == 1);
    CHECK(plan->delegation.empty());
    CHECK_FALSE(is_directly_legal_seam(root, Point(10000, 5000), true));
    CHECK(is_directly_legal_seam(root, Point(10000, 0), true));
    const std::optional<ExtrusionSeamPlan> split = legal_seam_entry(root, Point(0, 5000), true);
    REQUIRE(split);
    CHECK(split->point == Point(0, 5000));
    CHECK(split->operation == Operation::SplitChild);
    CHECK(split->child_index == 2);
    CHECK(root.child(1).child(0).polyline_ref().front() == Point(10000, 0));
    CHECK(root.child(1).child(1).polyline_ref().front() == Point(10000, 10000));
    CHECK_FALSE(legal_seam_entry(root, Point(50000, 50000), true));
}

// With a barrier no direct child may move, but off-contour requests must still be rejected.
TEST_CASE("Legal seam plans preserve barriers without accepting arbitrary provider points",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    using Operation = ExtrusionSeamPlan::Operation;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 0}, {0, 0}})));
    root.child(0).add_property(ExtrusionPropertySpeed(25.f));
    const std::optional<ExtrusionSeamPlan> free = legal_seam_entry(root, Point(5000, 0), true);
    REQUIRE(free);
    CHECK(free->operation == Operation::SplitChild);
    // Equal-distance forward/back segments select the first stored direct child.
    CHECK(free->child_index == 0);
    REQUIRE(Access::get_or_add_property_data_mutable(root.child(0), EXTRUSION_PROPERTY_TYPE_HEATER,
        sizeof(c_extrusion_property_heater), alignof(c_extrusion_property_heater)) != nullptr);
    const std::optional<ExtrusionSeamPlan> blocked = legal_seam_entry(root, Point(5000, 0), true);
    REQUIRE(blocked);
    CHECK(blocked->operation == Operation::KeepStart);
    CHECK(blocked->point == Point(0, 0));
    CHECK_FALSE(legal_seam_entry(root, Point(5000, 5000), true));
    CHECK_FALSE(is_directly_legal_seam(root, Point(5000, 0), true));
    CHECK(is_directly_legal_seam(root, Point(0, 0), true));
    CHECK(root.child_count() == 2);
    CHECK(Access::has_property(root.child(0), EXTRUSION_PROPERTY_TYPE_HEATER));
}

// Projection uses native arcs and never linearizes or rewrites endpoint Z offsets.
TEST_CASE("Legal seam planning keeps arc and Z geometry immutable",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    using Operation = ExtrusionSeamPlan::Operation;
    ArcPolyline path;
    path.append(Point(10001, 1));
    path.append(Geometry::ArcWelder::Segment(Point(1, 10001), 10000.f, Geometry::ArcWelder::Orientation::CCW));
    path.append(Point(1, 1));
    path.append(Point(10001, 1));
    path.set_z_offset(0, 100);
    path.set_z_offset(1, 900);
    path.set_z_offset(2, 600);
    path.set_z_offset(3, 100);
    ExtrusionEntity loop(false, path);
    const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(loop, Point(7072, 7072), true);
    REQUIRE(plan);
    CHECK(plan->operation == Operation::RotateLocal);
    CHECK(plan->point == Point(7072, 7072));
    CHECK(loop.polyline_ref().size() == 4);
    CHECK(loop.polyline_ref().get_arc(1).radius == 10000.f);
    CHECK(loop.polyline_ref().get_arc(1).orientation == Geometry::ArcWelder::Orientation::CCW);
    for (size_t index = 0; index < path.size(); ++index) {
        CHECK(loop.polyline_ref().get_point(index) == path.get_point(index));
        CHECK(loop.polyline_ref().z_offset(index) == path.z_offset(index));
    }
}

TEST_CASE("Seam descriptions announce the native split output rather than its rounded input",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    const ArcPolyline path(Points{{0, 0}, {10000, 20100}, {0, 20100}, {0, 0}});
    ExtrusionEntity local(false, path);
    const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(local, Point(141, 64), true);
    REQUIRE(plan);
    REQUIRE(plan->operation == ExtrusionSeamPlan::Operation::RotateLocal);
    CHECK(plan->split_point == Point(53, 108));
    CHECK(plan->point == Point(54, 108));
    ArcPolyline before;
    ArcPolyline after;
    Point input = plan->split_point;
    path.split_at(input, before, after);
    REQUIRE(before.size() >= 2);
    REQUIRE(after.size() >= 2);
    CHECK(before.back() == plan->point);
    CHECK(after.front() == plan->point);
    CHECK(local.polyline_ref().front() == Point(0, 0));

    ExtrusionEntity composed(ExtrusionEntity::Children(), false, false, true);
    composed.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 20100}})));
    composed.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{10000, 20100}, {0, 20100}, {0, 0}})));
    const std::optional<ExtrusionSeamPlan> direct = legal_seam_entry(composed, Point(141, 64), true);
    REQUIRE(direct);
    CHECK(direct->operation == ExtrusionSeamPlan::Operation::SplitChild);
    CHECK(direct->child_index == 0);
    CHECK(direct->point == plan->point);
    CHECK(direct->split_point == plan->split_point);
}

TEST_CASE("Seam planning never splits a protected closed local child",
          "[plugins][extrusion][seam-domain-preparation]")
{
    using namespace Slic3r;
    for (const bool empty_sibling : {false, true}) {
        CAPTURE(empty_sibling);
        ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
        std::unique_ptr<ExtrusionEntity> contour = std::make_unique<ExtrusionEntity>(false,
            ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}}));
        contour->set_can_rotate(false);
        root.append_child(std::move(contour));
        if (empty_sibling)
            root.append_child(std::make_unique<ExtrusionEntity>(false));
        ExtrusionEntityUPtr copy = root.clone_canonical();
        for (const ExtrusionEntity *view : {&root, copy.get()}) {
            const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(*view, Point(5000, 0), true);
            REQUIRE(plan);
            CHECK(plan->operation == ExtrusionSeamPlan::Operation::KeepStart);
            CHECK(plan->point == Point(0, 0));
            CHECK(plan->delegation.empty());
            CHECK_FALSE(view->child(0).can_rotate());
            CHECK(view->child(0).polyline_ref().size() == 4);
        }
    }
}

// Compare legacy and canonical positioned commands without stopping at their dynamic type.
TEST_CASE("Tree translation preserves positioned commands and translates their stored descendants",
          "[plugins][extrusion][tree-transform]")
{
    using namespace Slic3r;
    const Point shift(1234, -5678);
    for (const bool positioned : {false, true}) {
        CAPTURE(positioned);
        ExtrusionNop source;
        source.position = positioned ? Point(7000, 8000) : ExtrusionNop::NOT_A_POINT;
        source.set_role(ExtrusionRole::SupportMaterial);
        source.add_property(ExtrusionAttributes{ExtrusionRole::SupportMaterial});
        source.set_can_rotate(false);
        source.add_property(ExtrusionPropertyCustomGcodeText("M117 translated command"));
        ArcPolyline arc;
        arc.append(Point(10000, 0));
        arc.append(Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Geometry::ArcWelder::Orientation::CCW));
        arc.set_z_offset(0, 100);
        arc.set_z_offset(1, 900);
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::SupportMaterial}, false);
        path->set_polyline(arc);
        path->set_can_rotate(true);
        source.append_child(std::move(path));
        source.append_child(std::make_unique<ExtrusionEntity>(false));
        ExtrusionEntityUPtr legacy(source.clone());
        ExtrusionEntityUPtr canonical = source.clone_canonical();
        for (ExtrusionEntity *copy : {legacy.get(), canonical.get()}) {
            const ExtrusionEntity *child = &copy->child(0);
            const size_t id = copy->get_id();
            copy->translate(shift);
            ExtrusionNop *nop = dynamic_cast<ExtrusionNop *>(copy);
            if (copy == legacy.get()) {
                REQUIRE(nop != nullptr);
                CHECK(nop->position == (positioned ? source.position + shift : ExtrusionNop::NOT_A_POINT));
                CHECK(nop->role() == ExtrusionRole::SupportMaterial);
            } else {
                CHECK(nop == nullptr);
                REQUIRE(typeid(*copy) == typeid(ExtrusionEntity));
                REQUIRE(copy->get_property<ExtrusionAttributes>() != nullptr);
                CHECK(copy->get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::SupportMaterial);
            }
            CHECK(copy->get_id() == id);
            CHECK(copy->can_sort() == source.can_sort());
            CHECK(copy->can_reverse() == source.can_reverse());
            CHECK_FALSE(copy->can_rotate());
            CHECK(copy->child(0).can_rotate());
            REQUIRE(copy->child_count() == 2);
            CHECK(&copy->child(0) == child);
            const ArcPolyline &translated = child->polyline_ref();
            CHECK(translated.front() == arc.front() + shift);
            CHECK(translated.back() == arc.back() + shift);
            CHECK(translated.get_arc(1).radius == arc.get_arc(1).radius);
            CHECK(translated.get_arc(1).orientation == arc.get_arc(1).orientation);
            CHECK(translated.z_offset(0) == 100);
            CHECK(translated.z_offset(1) == 900);
            REQUIRE(copy->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(copy->custom_gcode_string(*copy->get_property<ExtrusionPropertyCustomGcode>()) == "M117 translated command");
        }
        CHECK(source.position == (positioned ? Point(7000, 8000) : ExtrusionNop::NOT_A_POINT));
        CHECK(source.child(0).polyline_ref().front() == arc.front());
        CHECK(source.child(0).polyline_ref().back() == arc.back());

        // The same positioned command may carry local content instead of
        // children. In particular, it must not be translated twice or wrapped.
        ExtrusionNop local;
        local.position = source.position;
        local.set_role(ExtrusionRole::SupportMaterial);
        local.add_property(ExtrusionAttributes{ExtrusionRole::SupportMaterial});
        local.set_polyline(arc);
        local.set_can_sort_reverse(false, false);
        local.set_can_rotate(false);
        local.add_property(ExtrusionPropertyCustomGcodeText("M117 local command"));
        ExtrusionEntityUPtr local_legacy(local.clone());
        ExtrusionEntityUPtr local_canonical = local.clone_canonical();
        for (ExtrusionEntity *copy : {local_legacy.get(), local_canonical.get()}) {
            const size_t id = copy->get_id();
            copy->translate(shift);
            const ExtrusionNop *nop = dynamic_cast<const ExtrusionNop *>(copy);
            if (copy == local_legacy.get()) {
                REQUIRE(nop != nullptr);
                CHECK(nop->position == (positioned ? local.position + shift : ExtrusionNop::NOT_A_POINT));
                CHECK(nop->role() == local.role());
            } else {
                CHECK(nop == nullptr);
                REQUIRE(typeid(*copy) == typeid(ExtrusionEntity));
                REQUIRE(copy->get_property<ExtrusionAttributes>() != nullptr);
                CHECK(copy->get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::SupportMaterial);
            }
            CHECK(copy->get_id() == id);
            CHECK(copy->child_count() == 0);
            CHECK_FALSE(copy->can_sort());
            CHECK_FALSE(copy->can_reverse());
            CHECK_FALSE(copy->can_rotate());
            const ArcPolyline &translated = copy->polyline_ref();
            CHECK(translated.size() == arc.size());
            CHECK(translated.front() == arc.front() + shift);
            CHECK(translated.back() == arc.back() + shift);
            CHECK(translated.get_arc(1).radius == arc.get_arc(1).radius);
            CHECK(translated.get_arc(1).orientation == arc.get_arc(1).orientation);
            CHECK(translated.z_offset(0) == arc.z_offset(0));
            CHECK(translated.z_offset(1) == arc.z_offset(1));
            REQUIRE(copy->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(copy->custom_gcode_string(*copy->get_property<ExtrusionPropertyCustomGcode>()) == "M117 local command");
        }
        CHECK(local.position == source.position);
        CHECK(local.polyline_ref().front() == arc.front());
        CHECK(local.polyline_ref().back() == arc.back());
    }
}

TEST_CASE("Tree translation leaves positioned commands and geometry unchanged for a zero shift",
          "[plugins][extrusion][tree-transform]")
{
    using namespace Slic3r;
    ExtrusionNop root;
    root.position = Point(1000, 2000);
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
    root.translate(Point(0, 0));
    CHECK(root.position == Point(1000, 2000));
    CHECK(root.child(0).polyline_ref().front() == Point(0, 0));
    CHECK(root.child(0).polyline_ref().back() == Point(10000, 0));
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

// Legacy C++ copies remain outside the canonical model; SDK command copies use properties only.
TEST_CASE("Legacy Nop copies keep private state without reintroducing it into canonical commands",
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
            ExtrusionEntityUPtr canonical = source.clone_canonical();
            check_canonical_copy(source, *canonical);
            ExtrusionEntityUPtr copy(source.clone());
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

TEST_CASE("Ordinary Collection and base copies own identical stored trees and resources",
          "[plugins][extrusion][native-copy-lifetime]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    const std::array<uint64_t, 8> expected{{1, 23, 456, 7890, 12345, 67890, 123456, 789012}};
    std::array<extrusion_data_id, 4> data_ids{};
    std::array<uint64_t, 4> node_ids{};
    std::array<std::string, 4> texts;
    std::unique_ptr<ExtrusionEntityCollection> native_copy;
    ExtrusionEntityUPtr base_copy;
    {
        ExtrusionEntityCollection source(false, true);
        source.set_can_rotate(false);
        std::unique_ptr<ExtrusionEntityCollection> group = std::make_unique<ExtrusionEntityCollection>(false, false);
        group->set_can_rotate(false);
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
        ArcPolyline geometry;
        geometry.append(Point(1000, 0));
        geometry.append(Geometry::ArcWelder::Segment(Point(0, 1000), 1000.f, Geometry::ArcWelder::Orientation::CCW));
        geometry.set_z_offset(0, 100);
        geometry.set_z_offset(1, 900);
        path->set_polyline(std::move(geometry));
        path->set_can_rotate(false);
        std::unique_ptr<ExtrusionNop> command = std::make_unique<ExtrusionNop>(
            ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::EXTRUSION, -17.5));
        command->set_role(ExtrusionRole::WipeTowerWipe);
        command->position = Point(1200, 3400);
        group->append_child(std::move(path));
        group->append_child(std::move(command));
        source.append_child(std::move(group));
        const std::array<ExtrusionEntity *, 4> nodes{{&source, &source.child(0),
            &source.child(0).child(0), &source.child(0).child(1)}};
        for (size_t index = 0; index < nodes.size(); ++index) {
            texts[index] = std::string(180, char('a' + index)) + '-' + std::to_string(index);
            nodes[index]->add_property(ExtrusionPropertyCustomGcodeText(texts[index]));
            nodes[index]->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter,
                ExtrusionFlow(0.125 * (index + 1), 0.5f, index % 2 == 0 ? 0.25f : -2.f)});
            data_ids[index] = Access::store_data_aligned(*nodes[index], expected.data(), sizeof(expected), 64);
            REQUIRE(data_ids[index] != EXTRUSION_DATA_ID_INVALID);
            node_ids[index] = nodes[index]->get_id();
        }
        native_copy = std::make_unique<ExtrusionEntityCollection>(source);
        base_copy = std::make_unique<ExtrusionEntity>(source);
        check_ordinary_copy(source, *native_copy);
        check_ordinary_copy(source, *base_copy);
        check_ordinary_copy(*native_copy, *base_copy);
        const std::array<const ExtrusionEntity *, 4> copied{{base_copy.get(), &base_copy->child(0),
            &base_copy->child(0).child(0), &base_copy->child(0).child(1)}};
        for (size_t index = 0; index < nodes.size(); ++index)
            CHECK(Access::stored_data(*copied[index], data_ids[index], nullptr) !=
                  Access::stored_data(*nodes[index], data_ids[index], nullptr));
    }
    REQUIRE(typeid(*native_copy) == typeid(ExtrusionEntityCollection));
    REQUIRE(typeid(*base_copy) == typeid(ExtrusionEntity));
    const std::array<const ExtrusionEntity *, 2> roots{{native_copy.get(), base_copy.get()}};
    for (const ExtrusionEntity *root : roots) {
        REQUIRE(root->child_count() == 1);
        REQUIRE(root->child(0).child_count() == 2);
        CHECK(typeid(root->child(0)) == typeid(ExtrusionEntityCollection));
        CHECK(typeid(root->child(0).child(0)) == typeid(ExtrusionPath));
        REQUIRE(typeid(root->child(0).child(1)) == typeid(ExtrusionNop));
        const std::array<const ExtrusionEntity *, 4> nodes{{root, &root->child(0),
            &root->child(0).child(0), &root->child(0).child(1)}};
        for (size_t index = 0; index < nodes.size(); ++index) {
            CAPTURE(index);
            CHECK(nodes[index]->get_id() == node_ids[index]);
            const ExtrusionAttributes *attributes = nodes[index]->get_property<ExtrusionAttributes>();
            REQUIRE(attributes != nullptr);
            CHECK(attributes->mm3_per_mm == 0.125 * (index + 1));
            CHECK(attributes->width == 0.5f);
            CHECK(attributes->height == (index % 2 == 0 ? 0.25f : -2.f));
            const ExtrusionPropertyCustomGcode *text = nodes[index]->get_property<ExtrusionPropertyCustomGcode>();
            REQUIRE(text != nullptr);
            CHECK(nodes[index]->custom_gcode_string(*text) == texts[index]);
            uint32_t size = 0;
            const void *buffer = Access::stored_data(*nodes[index], data_ids[index], &size);
            REQUIRE(buffer != nullptr);
            REQUIRE(size == sizeof(expected));
            CHECK(reinterpret_cast<uintptr_t>(buffer) % 64 == 0);
            CHECK(std::memcmp(buffer, expected.data(), sizeof(expected)) == 0);
        }
        const ExtrusionNop &command = static_cast<const ExtrusionNop &>(root->child(0).child(1));
        CHECK(command.role() == ExtrusionRole::WipeTowerWipe);
        CHECK(command.position == Point(1200, 3400));
        const ExtrusionPropertySpecialCommand *action = command.get_property<ExtrusionPropertySpecialCommand>();
        REQUIRE(action != nullptr);
        CHECK(action->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION);
        CHECK(action->extra_data == -17.5);
    }
    native_copy->child(0).child(0).polyline_ref().translate(Point(123, 456));
    CHECK(base_copy->child(0).child(0).polyline_ref().front() == Point(1000, 0));
    CHECK(native_copy->child(0).child(0).polyline_ref().front() == Point(1123, 456));
}

TEST_CASE("Ordinary root copy destruction distinguishes Children from local geometry",
          "[plugins][extrusion][native-copy-lifetime]")
{
    using namespace Slic3r;
    for (const bool local_geometry : {false, true}) {
        ExtrusionEntityCollection source(false, false);
        const Points expected{{0, 0}, {10000, 0}};
        if (local_geometry) {
            ExtrusionEntity geometry(false, ArcPolyline(expected));
            REQUIRE(extrusion_move_from(entity_handle(source), entity_handle(geometry)) != 0);
            REQUIRE_FALSE(geometry.has_polyline());
        } else {
            std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
            path->set_polyline(ArcPolyline(expected));
            source.append_child(std::move(path));
        }
        REQUIRE(source.has_polyline() == local_geometry);
        REQUIRE(source.child_count() == (local_geometry ? 0 : 1));
        const uint64_t source_id = source.get_id();
        for (const bool native_shell : {false, true}) {
            CAPTURE(local_geometry, native_shell);
            const ExtrusionEntity before(false);
            if (native_shell) {
                const ExtrusionEntityCollection copy(source);
                CHECK(copy.get_id() == source_id);
                CHECK(copy.has_polyline() == local_geometry);
            } else {
                const ExtrusionEntity copy(source);
                CHECK(copy.get_id() == source_id);
                CHECK(copy.has_polyline() == local_geometry);
            }
            // The source stays alive so its own destruction cannot affect these relative ID markers.
            const ExtrusionEntity after(false);
            CHECK(after.get_id() == before.get_id() + 1 + (native_shell && local_geometry ? 1 : 0));
            CHECK(source.get_id() == source_id);
            CHECK(source.has_polyline() == local_geometry);
            CHECK(source.child_count() == (local_geometry ? 0 : 1));
            const ExtrusionEntity &geometry = local_geometry ? static_cast<const ExtrusionEntity &>(source) : source.child(0);
            CHECK(geometry.polyline_ref().to_polyline().points == expected);
        }
    }
}

// Reject short inputs before the polygon invariant without consuming borrowed points.
TEST_CASE("Native loop builders ignore polygons with fewer than three vertices", "[plugins][native-fill]")
{
    using namespace Slic3r;
    const ExtrusionAttributes attributes{ExtrusionRole::SolidInfill, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    for (const bool consume : {false, true}) {
        CAPTURE(consume);
        // Populate public points directly so constructor checks do not preempt the input filter.
        Polygons loops(3);
        loops[1].points = {Point(0, 0)};
        loops[2].points = {Point(0, 0), Point(1000000, 0)};
        const std::vector<Points> before{loops[0].points, loops[1].points, loops[2].points};
        ExtrusionEntityCollection output;
        if (consume)
            extrusion_entities_append_loops(output, std::move(loops), ExtrusionAttributes(attributes), false);
        else
            extrusion_entities_append_loops(output, loops, ExtrusionAttributes(attributes), false);
        CHECK(output.child_count() == 0);
        if (consume) {
            CHECK(loops.empty());
        } else {
            REQUIRE(loops.size() == before.size());
            for (size_t index = 0; index < loops.size(); ++index)
                CHECK(loops[index].points == before[index]);
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
    root.set_can_rotate(false);
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

TEST_CASE("Ordered leaves wrap a fixed-order collection that still permits rotation",
          "[plugins][extrusion][ordered-leaf][rotation][seam-activation]")
{
    using namespace Slic3r;
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}, {0, 0}})));
    REQUIRE(root.can_rotate());
    root.add_property(ExtrusionPropertySpeed(35.f, 700.f));
    const ExtrusionEntity *original = &root.child(0);
    ExtrusionEntity *event = root.emplace_ordered_leaf(OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
    REQUIRE(event);
    REQUIRE(root.child_count() == 2);
    CHECK(&root.child(0) == event);
    REQUIRE(root.child(1).child_count() == 1);
    CHECK(&root.child(1).child(0) == original);
    CHECK(root.child(1).can_rotate());
    CHECK_FALSE(root.can_rotate());
    CHECK_FALSE(event->can_rotate());
    CHECK(root.get_property<ExtrusionPropertySpeed>() != nullptr);
    CHECK(root.child(1).get_property<ExtrusionPropertySpeed>() == nullptr);
}

TEST_CASE("Core C and SDK agree on geometric loops and independent seam permission",
          "[plugins][extrusion][seam-activation]")
{
    using namespace Slic3r;
    ExtrusionPath path(ExtrusionAttributes{ExtrusionRole::Perimeter});
    path.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}}));
    ExtrusionMultiPath fixed(path);
    REQUIRE_FALSE(fixed.is_loop());
    REQUIRE(fixed.ExtrusionEntity::is_loop());
    CHECK(mutable_view(fixed).readonly().is_loop());
    CHECK_FALSE(mutable_view(fixed).readonly().is_seam_unit());
    CHECK(mutable_view(path).readonly().is_seam_unit());
    ExtrusionEntityUPtr copy = fixed.clone_canonical();
    CHECK(mutable_view(*copy).readonly().is_loop());
    CHECK_FALSE(mutable_view(*copy).readonly().is_seam_unit());
    fixed.set_can_rotate(true);
    CHECK(mutable_view(fixed).readonly().is_seam_unit());
    ExtrusionLoop near;
    path.set_polyline(ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {1, 0}}));
    near.append_child(std::make_unique<ExtrusionPath>(path));
    REQUIRE(near.is_loop());
    CHECK_FALSE(mutable_view(near).readonly().is_loop());
    CHECK_FALSE(mutable_view(near).readonly().is_seam_unit());
}

TEST_CASE("Native custom seam scores keep original legal indices and fall back without a callback",
          "[plugins][extrusion][seam-activation]")
{
    using namespace Slic3r;
    using namespace Slic3r::Printing;
    struct Analysis final : LoopEntryAnalysis {
        Points points;
        std::vector<size_t> &scored;
        std::vector<size_t> &selected;
        Analysis(Points values, std::vector<size_t> &scores, std::vector<size_t> &selections)
            : points(std::move(values)), scored(scores), selected(selections) {}
        size_t candidate_count() const override { return points.size(); }
        Point candidate_point(size_t index) const override { return points.at(index); }
        double score_candidate(size_t index, const Point &) const override {
            scored.push_back(index);
            return index == 0 ? -1000. : 42.;
        }
        void rotate_loop_to_candidate(ExtrusionEntity &, size_t index) const override {
            selected.push_back(index);
            CHECK(index == 1);
            // The only admissible candidate is the existing entry.
        }
    };
    struct Policy final : LoopEntryPolicy {
        Points points;
        mutable std::vector<size_t> scored;
        mutable std::vector<size_t> selected;
        std::unique_ptr<LoopEntryAnalysis> analyze_loop(const ExtrusionEntity &, const LoopEntryContext &) const override {
            return std::make_unique<Analysis>(points, scored, selected);
        }
    };
    for (const bool has_legal : {false, true}) {
        CAPTURE(has_legal);
        ExtrusionEntity outer(ExtrusionEntity::Children(), true, false, false);
        auto loop = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        loop->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {10000, 0}})));
        auto atomic = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        atomic->append_child(std::make_unique<ExtrusionEntity>(false,
            ArcPolyline(Points{{10000, 0}, {10000, 10000}, {0, 10000}})));
        loop->append_child(std::move(atomic));
        loop->append_child(std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 10000}, {0, 0}})));
        const ExtrusionEntity *before = loop.get();
        outer.append_child(std::move(loop));
        Policy policy;
        policy.points = Points{{10000, 10000}}; // Preferred -1000 score, but illegal inside the atomic composite.
        if (has_legal)
            policy.points.emplace_back(0, 0);
        order_extrusion_tree(outer, Point(10000, 10000), policy, LoopEntryContext{});
        REQUIRE(outer.child_count() == 1);
        CHECK(&outer.child(0) == before);
        CHECK(outer.ExtrusionEntity::first_point() == Point(0, 0));
        CHECK(outer.child(0).child_count() == 3);
        CHECK_FALSE(outer.child(0).can_rotate());
        if (has_legal) {
            CHECK(policy.scored == std::vector<size_t>{1});
            CHECK(policy.selected == std::vector<size_t>{1});
        } else {
            CHECK(policy.scored.empty());
            CHECK(policy.selected.empty());
        }
    }
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

TEST_CASE("Native overhang root processing preserves dispatch copies and numeric fragments",
          "[plugins][extrusion][native-overhang-root]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    const std::vector<Linef> rectangle{
        Linef(Vec2d(-2., -2.), Vec2d(2., -2.)), Linef(Vec2d(2., -2.), Vec2d(2., 2.)),
        Linef(Vec2d(2., 2.), Vec2d(-2., 2.)), Linef(Vec2d(-2., 2.), Vec2d(-2., -2.))};
    const AABBTreeLines::LinesDistancer<Linef> previous(rectangle);
    const AABBTreeLines::LinesDistancer<CurledLine> no_curled;
    const double nozzle = 0.4;

    SECTION("empty root projects permissions but not direct root properties")
    {
        ExtrusionEntityCollection source(false, true);
        source.set_can_rotate(false);
        source.add_property(ExtrusionPropertySpeed(19.f));
        const ExtrusionEntityCollection before(source);
        const ExtrusionEntity &borrowed_source = source;
        const ExtrusionEntityCollection result =
            ExtrusionProcessor::calculate_and_split_overhanging_extrusions(borrowed_source, previous, no_curled, nozzle);
        CHECK(result.child_count() == 0);
        CHECK(result.can_sort() == source.can_sort());
        CHECK(result.can_reverse() == source.can_reverse());
        CHECK(result.can_rotate() == source.can_rotate());
        CHECK(result.get_id() != source.get_id());
        CHECK(Access::property_count(result) == 0);
        check_ordinary_copy(source, before);
    }
    SECTION("mixed native children retain their order types and exact copies including Z bypass")
    {
        ExtrusionEntityCollection source(false, false);
        source.set_can_rotate(false);
        source.add_property(ExtrusionPropertySpeed(23.f));
        source.append_child(std::make_unique<ExtrusionPath>(
            overhang_fixture_path(Points{{0, 0}, {scale_i(1.), 0}}, false, false)));
        source.append_child(std::make_unique<ExtrusionLoop>(overhang_fixture_path(
            Points{{0, 0}, {scale_i(1.), 0}, {scale_i(1.), scale_i(1.)}, {0, 0}}, false, false)));
        const ExtrusionPaths joined{
            overhang_fixture_path(Points{{0, 0}, {scale_i(1.), 0}}, false, false),
            overhang_fixture_path(Points{{scale_i(1.), 0}, {scale_i(2.), 0}}, false, false)};
        source.append_child(std::make_unique<ExtrusionMultiPath>(joined));
        std::unique_ptr<ExtrusionEntityCollection> nested = std::make_unique<ExtrusionEntityCollection>(true, false);
        nested->set_can_rotate(false);
        nested->add_property(ExtrusionPropertySpeed(29.f));
        nested->append_child(std::make_unique<ExtrusionPath>(
            overhang_fixture_path(Points{{0, 0}, {0, scale_i(1.)}}, false, false)));
        source.append_child(std::move(nested));
        source.append_child(std::make_unique<ExtrusionPath>(
            overhang_fixture_path(Points{{scale_i(-3.), 0}, {scale_i(3.), 0}}, true, true)));
        REQUIRE(source.child_count() == 5);
        const std::array<const ExtrusionEntity *, 5> addresses{
            &source.child(0), &source.child(1), &source.child(2), &source.child(3), &source.child(4)};
        const ExtrusionEntityCollection before(source);
        const ExtrusionEntity &borrowed_source = source;
        const ExtrusionEntityCollection result =
            ExtrusionProcessor::calculate_and_split_overhanging_extrusions(borrowed_source, previous, no_curled, nozzle);
        REQUIRE(result.child_count() == 5);
        CHECK(result.get_id() != source.get_id());
        CHECK(result.can_sort() == source.can_sort());
        CHECK(result.can_reverse() == source.can_reverse());
        CHECK(result.can_rotate() == source.can_rotate());
        CHECK(Access::property_count(result) == 0);
        for (const size_t index : {size_t(0), size_t(1), size_t(2), size_t(4)}) {
            CHECK(typeid(result.child(index)) == typeid(source.child(index)));
            check_ordinary_copy(source.child(index), result.child(index));
        }
        REQUIRE(typeid(result.child(3)) == typeid(ExtrusionEntityCollection));
        REQUIRE(result.child(3).child_count() == 1);
        CHECK(result.child(3).get_id() != source.child(3).get_id());
        CHECK(result.child(3).can_sort() == source.child(3).can_sort());
        CHECK(result.child(3).can_reverse() == source.child(3).can_reverse());
        CHECK(result.child(3).can_rotate() == source.child(3).can_rotate());
        CHECK(Access::property_count(result.child(3)) == 0);
        check_ordinary_copy(source.child(3).child(0), result.child(3).child(0));
        check_ordinary_copy(source, before);
        for (size_t index = 0; index < addresses.size(); ++index)
            CHECK(&source.child(index) == addresses[index]);
    }
    SECTION("real planar fragments are independent and survive their input owners")
    {
        const AABBTreeLines::LinesDistancer<CurledLine> curled(std::vector<CurledLine>{
            CurledLine(Point(scale_i(2.25), scale_i(-1.)), Point(scale_i(2.25), scale_i(1.)), 0.3f)});
        for (const bool with_curled : {false, true}) {
            CAPTURE(with_curled);
            std::unique_ptr<ExtrusionEntityCollection> result;
            std::array<uint64_t, 2> source_ids;
            std::string state;
            {
                ExtrusionEntityCollection source(false, false);
                source.set_can_rotate(false);
                source.append_child(std::make_unique<ExtrusionPath>(overhang_fixture_path(
                    Points{{scale_i(-3.), 0}, {0, 0}, {scale_i(3.), 0}}, true, false)));
                const ExtrusionEntity *address = &source.child(0);
                source_ids = {source.get_id(), address->get_id()};
                const ExtrusionEntityCollection before(source);
                const ExtrusionEntity &borrowed_source = source;
                result = std::make_unique<ExtrusionEntityCollection>(
                    ExtrusionProcessor::calculate_and_split_overhanging_extrusions(
                        borrowed_source, previous, with_curled ? curled : no_curled, nozzle));
                REQUIRE(result->child_count() > 0);
                CHECK(result->get_id() != source.get_id());
                const ExtrusionPath *first = dynamic_cast<const ExtrusionPath *>(&result->child(0));
                const ExtrusionPath *last = dynamic_cast<const ExtrusionPath *>(&result->child(result->child_count() - 1));
                REQUIRE(first != nullptr);
                REQUIRE(last != nullptr);
                CHECK(first->first_point() == Point(scale_i(-3.), 0));
                CHECK(last->last_point() == Point(scale_i(3.), 0));
                bool positive_proximity = false;
                for (const ExtrusionEntityUPtr &child : result->children()) {
                    const ExtrusionPropertyOverhang *overhang = child->get_property<ExtrusionPropertyOverhang>();
                    REQUIRE(overhang != nullptr);
                    CHECK(child.get() != address);
                    CHECK(child->get_id() != address->get_id());
                    positive_proximity = positive_proximity || overhang->proximity_to_curled_lines > 0.f;
                }
                CHECK(positive_proximity == with_curled);
                check_ordinary_copy(source, before);
                CHECK(&source.child(0) == address);
                state = overhang_fragment_state(*result, source_ids);
            }
            REQUIRE(result != nullptr);
            CHECK(overhang_fragment_state(*result, source_ids) == state);
            const std::string expected = with_curled ?
                "Collection,2,0,0,0,16;"
                "Path,3,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.39999a0000000p+0,0x1.a666660000000p-1,0x0.0000000000000p+0,0,0,1,1,2,0,0:-3000000,0,0/-2599999,0,0/;"
                "Path,4,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.a666660000000p-1,0x1.b333300000000p-2,0x0.0000000000000p+0,0,0,1,1,2,0,0:-2599999,0,0/-2199999,0,0/;"
                "Path,5,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.b333300000000p-2,0x1.cccccc0000000p-3,0x1.5d879a0000000p-13,0,0,1,1,2,0,0:-2199999,0,0/-2000000,0,0/;"
                "Path,6,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.cccccc0000000p-3,0x0.0000000000000p+0,0x1.8937980000000p-10,0,0,1,1,2,0,0:-2000000,0,0/-1599999,0,0/;"
                "Path,7,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.5ebd640000000p-8,0,0,1,1,2,0,0:-1599999,0,0/-1199999,0,0/;"
                "Path,8,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.7b42820000000p-7,0,0,1,1,2,0,0:-1199999,0,0/-799999,0,0/;"
                "Path,9,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.4a693c0000000p-6,0,0,1,1,2,0,0:-799999,0,0/-399999,0,0/;"
                "Path,10,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.fe073e0000000p-6,0,0,1,1,2,0,0:-399999,0,0/0,0,0/;"
                "Path,11,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.6c3d980000000p-5,0,0,1,1,2,0,0:0,0,0/400000,0,0/;"
                "Path,12,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.ece2a60000000p-5,0,0,1,1,2,0,0:400000,0,0/800000,0,0/;"
                "Path,13,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.40795e0000000p-4,0,0,1,1,2,0,0:800000,0,0/1200000,0,0/;"
                "Path,14,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x1.9436ec0000000p-4,0,0,1,1,2,0,0:1200000,0,0/1600000,0,0/;"
                "Path,15,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.cccccc0000000p-3,0x1.cccccc0000000p-3,0x1.f1a9fe0000000p-4,0,0,1,1,2,0,0:1600000,0,0/2000000,0,0/;"
                "Path,16,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.4000000000000p-1,0x1.4000000000000p-1,0x1.2c694a0000000p-3,0,0,1,1,2,0,0:2000000,0,0/2400000,0,0/;"
                "Path,17,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.0666660000000p+0,0x1.0666660000000p+0,0x1.0545900000000p-3,0,0,1,1,2,0,0:2400000,0,0/2800000,0,0/;"
                "Path,18,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.39999a0000000p+0,0x1.39999a0000000p+0,0x1.c1b9c40000000p-4,0,0,1,1,2,0,0:2800000,0,0/3000000,0,0/;"
                :
                "Collection,2,0,0,0,9;"
                "Path,3,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.39999a0000000p+0,0x1.a666660000000p-1,0x0.0000000000000p+0,0,0,1,1,2,0,0:-3000000,0,0/-2599999,0,0/;"
                "Path,4,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.a666660000000p-1,0x1.b333300000000p-2,0x0.0000000000000p+0,0,0,1,1,2,0,0:-2599999,0,0/-2199999,0,0/;"
                "Path,5,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.b333300000000p-2,0x1.cccccc0000000p-3,0x0.0000000000000p+0,0,0,1,1,2,0,0:-2199999,0,0/-2000000,0,0/;"
                "Path,6,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.cccccc0000000p-3,0x0.0000000000000p+0,0x0.0000000000000p+0,0,0,1,1,2,0,0:-2000000,0,0/-1599999,0,0/;"
                "Path,7,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x0.0000000000000p+0,0x0.0000000000000p+0,0x0.0000000000000p+0,0,0,1,1,9,0,0:-1599999,0,0/-1199999,0,0/-799999,0,0/-399999,0,0/0,0,0/400000,0,0/800000,0,0/1200000,0,0/1600000,0,0/;"
                "Path,8,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.cccccc0000000p-3,0x1.cccccc0000000p-3,0x0.0000000000000p+0,0,0,1,1,2,0,0:1600000,0,0/2000000,0,0/;"
                "Path,9,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.4000000000000p-1,0x1.4000000000000p-1,0x0.0000000000000p+0,0,0,1,1,2,0,0:2000000,0,0/2400000,0,0/;"
                "Path,10,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.0666660000000p+0,0x1.0666660000000p+0,0x0.0000000000000p+0,0,0,1,1,2,0,0:2400000,0,0/2800000,0,0/;"
                "Path,11,0,0,1,0x1.eb851eb851eb8p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,1,1,0x1.39999a0000000p+0,0x1.39999a0000000p+0,0x0.0000000000000p+0,0,0,1,1,2,0,0:2800000,0,0/3000000,0,0/;";
            CHECK(state == expected);
        }
    }
    SECTION("generic polyline and generic group children retain rejection")
    {
        for (const bool group : {false, true}) {
            CAPTURE(group);
            ExtrusionEntityCollection source;
            std::unique_ptr<ExtrusionEntity> child;
            if (group) {
                child = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, false, false);
                child->append_child(std::make_unique<ExtrusionPath>(
                    overhang_fixture_path(Points{{0, 0}, {scale_i(1.), 0}}, false, false)));
            } else {
                child = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{0, 0}, {scale_i(1.), 0}}));
            }
            source.append_child(std::move(child));
            const ExtrusionEntity *address = &source.child(0);
            const ExtrusionEntityCollection before(source);
            const ExtrusionEntity &borrowed_source = source;
            REQUIRE_THROWS_AS(ExtrusionProcessor::calculate_and_split_overhanging_extrusions(
                borrowed_source, previous, no_curled, nozzle), InvalidArgument);
            check_ordinary_copy(source, before);
            CHECK(&source.child(0) == address);
        }
    }
}

TEST_CASE("Native flatten destinations preserve wrappers prefixes caches and local donation",
          "[plugins][extrusion][native-flatten-destination]")
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    SECTION("explicit false fixed true fixed and true sortable destination layouts")
    {
        for (const int mode : {0, 1, 2}) {
            CAPTURE(mode);
            std::unique_ptr<ExtrusionEntityCollection> output;
            std::unique_ptr<ExtrusionEntityCollection> survivor_reference;
            {
                ExtrusionEntityCollection source = flatten_fixture(mode == 2);
                REQUIRE(source.is_collection());
                REQUIRE(source.child_count() == 4);
                REQUIRE(source.child(1).is_collection());
                REQUIRE(source.child(1).child_count() == 2);
                CHECK(source.can_sort() == (mode == 2));
                CHECK_FALSE(source.child(1).can_sort());
                CHECK_FALSE(source.child(2).is_collection());
                CHECK_FALSE(source.child(3).is_collection());
                const std::array<const ExtrusionEntity *, 6> addresses{
                    &source.child(0), &source.child(1), &source.child(2), &source.child(3),
                    &source.child(1).child(0), &source.child(1).child(1)};
                const ExtrusionEntityCollection before(source);
                output = std::make_unique<ExtrusionEntityCollection>(false, false);
                output->set_can_rotate(false);
                output->add_property(ExtrusionPropertySpeed(83.f));
                output->append_child(std::make_unique<ExtrusionPath>(
                    flatten_fixture_path(Points{{-20000, 0}, {-19000, 0}})));
                const ExtrusionEntity *sentinel = &output->child(0);
                const uint64_t output_id = output->get_id();
                const ExtrusionEntityCollection output_before(*output);
                REQUIRE(output->entities().size() == 1);

                ExtrusionEntity &borrowed_output = *output;
                source.flatten(mode != 0, borrowed_output);

                CHECK(output->get_id() == output_id);
                CHECK_FALSE(output->can_sort());
                CHECK_FALSE(output->can_reverse());
                CHECK_FALSE(output->can_rotate());
                REQUIRE(Access::property_count(*output) == 1);
                CHECK(Access::same_property_payload(output_before, extrusion_property_type_speed, *output));
                const size_t sentinel_index = mode == 0 ? 5 : (mode == 1 ? 0 : 4);
                REQUIRE(output->child_count() == (mode == 0 ? 6 : (mode == 1 ? 2 : 5)));
                CHECK(&output->child(sentinel_index) == sentinel);
                check_ordinary_copy(output_before.child(0), output->child(sentinel_index));
                const std::array<const ExtrusionEntity *, 5> originals{
                    &source.child(0), &source.child(1).child(0), &source.child(1).child(1),
                    &source.child(2), &source.child(3)};
                std::array<const ExtrusionEntity *, 5> copies;
                if (mode == 0) {
                    copies = {&output->child(0), &output->child(1), &output->child(2),
                              &output->child(3), &output->child(4)};
                } else {
                    const ExtrusionEntity *expanded = output.get();
                    if (mode == 1) {
                        const ExtrusionEntity &outer = output->child(1);
                        REQUIRE(typeid(outer) == typeid(ExtrusionEntityCollection));
                        REQUIRE(outer.child_count() == 1);
                        CHECK(outer.can_sort());
                        CHECK(outer.can_reverse());
                        CHECK(outer.can_rotate());
                        CHECK(Access::property_count(outer) == 0);
                        CHECK(outer.get_id() != source.get_id());
                        CHECK(outer.get_id() != output_id);
                        expanded = &outer.child(0);
                        check_rebuilt_flatten_group(source, *expanded);
                        REQUIRE(expanded->child_count() == 4);
                        CHECK(expanded->get_id() != outer.get_id());
                    }
                    const ExtrusionEntity &nested = expanded->child(1);
                    check_rebuilt_flatten_group(source.child(1), nested);
                    REQUIRE(nested.child_count() == 2);
                    copies = {&expanded->child(0), &nested.child(0), &nested.child(1),
                              &expanded->child(2), &expanded->child(3)};
                }
                for (size_t index = 0; index < originals.size(); ++index) {
                    CHECK(typeid(*copies[index]) == typeid(*originals[index]));
                    check_ordinary_copy(*originals[index], *copies[index]);
                }
                REQUIRE(output->entities().size() == output->child_count());
                for (size_t index = 0; index < output->child_count(); ++index)
                    CHECK(output->entities()[index] == &output->child(index));
                check_ordinary_copy(source, before);
                for (size_t index = 0; index < 4; ++index)
                    CHECK(&source.child(index) == addresses[index]);
                CHECK(&source.child(1).child(0) == addresses[4]);
                CHECK(&source.child(1).child(1) == addresses[5]);
                survivor_reference = std::make_unique<ExtrusionEntityCollection>(*output);
            }
            REQUIRE(output != nullptr);
            REQUIRE(survivor_reference != nullptr);
            check_ordinary_copy(*survivor_reference, *output);
        }
    }
    SECTION("local geometry is donated before prefixing or appending the whole wrapper")
    {
        for (const bool preserve : {false, true}) {
            CAPTURE(preserve);
            ExtrusionEntityCollection source(false, false);
            source.set_can_rotate(false);
            source.add_property(ExtrusionPropertySpeed(91.f));
            source.append_child(std::make_unique<ExtrusionPath>(
                flatten_fixture_path(Points{{0, 0}, {1000, 0}})));
            source.append_child(std::make_unique<ExtrusionPath>(
                flatten_fixture_path(Points{{10000, 0}, {11000, 0}})));
            REQUIRE(source.is_collection());
            REQUIRE(source.child_count() == 2);
            const ExtrusionEntityCollection before(source);
            const ExtrusionEntity *first_address = &source.child(0);
            const ExtrusionEntity *second_address = &source.child(1);
            ArcPolyline geometry(Points{{-9000, 0}, {-8000, 0}});
            geometry.set_z_offset(0, 100);
            geometry.set_z_offset(1, 900);
            ExtrusionEntity local_content(false, std::move(geometry));
            local_content.set_can_rotate(false);
            local_content.add_property(ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.12, 0.45f, 0.2f)));
            local_content.add_property(ExtrusionPropertySpeed(61.f));
            ExtrusionEntityCollection output;
            static_cast<ExtrusionEntity &>(output) = std::move(local_content);
            REQUIRE(output.is_leaf());
            REQUIRE(output.has_polyline());
            CHECK_FALSE(output.can_sort());
            CHECK_FALSE(output.can_reverse());
            const ExtrusionEntityCollection output_before(output);
            const uint64_t output_id = output.get_id();

            ExtrusionEntity &borrowed_output = output;
            source.flatten(preserve, borrowed_output);

            CHECK(output.get_id() == output_id);
            CHECK_FALSE(output.is_leaf());
            CHECK_FALSE(output.has_polyline());
            CHECK(output.can_sort());
            CHECK(output.can_reverse());
            CHECK_FALSE(output.can_rotate());
            REQUIRE(output.child_count() == (preserve ? 2 : 3));
            const ExtrusionEntity &donated = output.child(preserve ? 0 : 2);
            REQUIRE(typeid(donated) == typeid(ExtrusionEntity));
            CHECK(donated.get_id() != output_id);
            CHECK_FALSE(donated.can_sort());
            CHECK_FALSE(donated.can_reverse());
            CHECK_FALSE(donated.can_rotate());
            REQUIRE(donated.has_polyline());
            REQUIRE(donated.polyline_ref().size() == 2);
            CHECK_FALSE(donated.polyline_ref().has_arc());
            REQUIRE(donated.polyline_ref().has_z_offset());
            CHECK(donated.polyline_ref().get_point(0) == Point(-9000, 0));
            CHECK(donated.polyline_ref().get_point(1) == Point(-8000, 0));
            CHECK(donated.polyline_ref().z_offset(0) == 100);
            CHECK(donated.polyline_ref().z_offset(1) == 900);
            REQUIRE(Access::property_count(output) == 2);
            REQUIRE(Access::property_count(donated) == 2);
            for (const extrusion_property_type type : {extrusion_property_type_attributes, extrusion_property_type_speed}) {
                CHECK(Access::same_property_payload(output_before, type, output));
                CHECK(Access::same_property_payload(output_before, type, donated));
                CHECK(Access::property_data(output, type) != Access::property_data(donated, type));
            }
            const ExtrusionEntity *copied_root = &output;
            if (preserve) {
                const ExtrusionEntity &outer = output.child(1);
                REQUIRE(typeid(outer) == typeid(ExtrusionEntityCollection));
                REQUIRE(outer.child_count() == 1);
                CHECK(outer.can_sort());
                CHECK(outer.can_reverse());
                CHECK(outer.can_rotate());
                CHECK(Access::property_count(outer) == 0);
                copied_root = &outer.child(0);
                check_rebuilt_flatten_group(source, *copied_root);
                REQUIRE(copied_root->child_count() == 2);
            }
            for (size_t index = 0; index < 2; ++index) {
                CHECK(typeid(copied_root->child(index)) == typeid(ExtrusionPath));
                check_ordinary_copy(source.child(index), copied_root->child(index));
            }
            REQUIRE(output.entities().size() == output.child_count());
            for (size_t index = 0; index < output.child_count(); ++index)
                CHECK(output.entities()[index] == &output.child(index));
            check_ordinary_copy(source, before);
            CHECK(&source.child(0) == first_address);
            CHECK(&source.child(1) == second_address);
        }
    }
}

namespace {

void check_ordinary_copy(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &copy)
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    CHECK(&source != &copy);
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
        CHECK(Access::property_data(source, type) != Access::property_data(copy, type));
    }
    if (const ExtrusionPropertyCustomGcode *text = source.get_property<ExtrusionPropertyCustomGcode>()) {
        const ExtrusionPropertyCustomGcode *copied = copy.get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(copied != nullptr);
        CHECK(copy.custom_gcode_string(*copied) == source.custom_gcode_string(*text));
        uint32_t original_size = 0;
        uint32_t copied_size = 0;
        const void *original = Access::stored_data(source, text->text_id, &original_size);
        const void *buffer = Access::stored_data(copy, copied->text_id, &copied_size);
        REQUIRE(original != nullptr);
        REQUIRE(buffer != nullptr);
        REQUIRE(original_size == copied_size);
        CHECK(buffer != original);
        CHECK(std::memcmp(original, buffer, original_size) == 0);
    }
    REQUIRE(copy.child_count() == source.child_count());
    for (size_t index = 0; index < source.child_count(); ++index) {
        CHECK(typeid(copy.child(index)) == typeid(source.child(index)));
        check_ordinary_copy(source.child(index), copy.child(index));
    }
}

Slic3r::ExtrusionPath overhang_fixture_path(const Slic3r::Points &points, bool dynamic, bool with_z)
{
    using namespace Slic3r;
    ExtrusionAttributes attributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.12, 0.45f, 0.2f));
    attributes.no_seam = 1;
    ExtrusionPath path(ArcPolyline(points), attributes, ExtrusionPropertyUPtr{}, false);
    path.set_can_rotate(true);
    if (dynamic)
        path.add_property(ExtrusionPropertyOverhang(17.f, 19.f, 0.f, false, false, true, true));
    if (with_z) {
        path.polyline().set_z_offset(0, 100);
        path.polyline().set_z_offset(points.size() - 1, 900);
    }
    return path;
}

std::string overhang_fragment_state(const Slic3r::ExtrusionEntityCollection &result,
                                    const std::array<uint64_t, 2> &source_ids)
{
    using namespace Slic3r;
    using Access = ApiInternal::ExtrusionPropertyAccess;
    std::map<uint64_t, size_t> identities{{source_ids[0], 0}, {source_ids[1], 1}};
    const auto identity = [&identities](uint64_t id) {
        const std::map<uint64_t, size_t>::const_iterator found = identities.find(id);
        if (found != identities.end())
            return found->second;
        const size_t next = identities.size();
        identities.emplace(id, next);
        return next;
    };
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    REQUIRE(Access::property_count(result) == 0);
    out << "Collection," << identity(result.get_id()) << ',' << result.can_sort() << ',' << result.can_reverse()
        << ',' << result.can_rotate() << ',' << result.child_count() << ';';
    for (const ExtrusionEntityUPtr &child : result.children()) {
        const ExtrusionPath *path = dynamic_cast<const ExtrusionPath *>(child.get());
        REQUIRE(path != nullptr);
        REQUIRE(path->has_polyline());
        REQUIRE(Access::property_count(*path) == 2);
        const ExtrusionAttributes *attributes = path->get_property<ExtrusionAttributes>();
        const ExtrusionPropertyOverhang *overhang = path->get_property<ExtrusionPropertyOverhang>();
        REQUIRE(attributes != nullptr);
        REQUIRE(overhang != nullptr);
        const ArcPolyline &polyline = path->polyline_ref();
        out << "Path," << identity(path->get_id()) << ',' << path->can_sort() << ',' << path->can_reverse()
            << ',' << path->can_rotate() << ',' << attributes->mm3_per_mm << ',' << attributes->width
            << ',' << attributes->height << ',' << attributes->role << ',' << unsigned(attributes->no_seam)
            << ',' << overhang->start_distance_from_prev_layer << ',' << overhang->end_distance_from_prev_layer
            << ',' << overhang->proximity_to_curled_lines << ',' << unsigned(overhang->has_full_overhangs_flow)
            << ',' << unsigned(overhang->has_full_overhangs_speed) << ',' << unsigned(overhang->has_dynamic_overhangs_flow)
            << ',' << unsigned(overhang->has_dynamic_overhangs_speed) << ',' << polyline.size()
            << ',' << polyline.has_z_offset() << ',' << polyline.has_arc() << ':';
        for (size_t index = 0; index < polyline.size(); ++index) {
            out << polyline.get_point(index).x() << ',' << polyline.get_point(index).y() << ','
                << (polyline.has_z_offset() ? polyline.z_offset(index) : 0);
            if (polyline.has_arc())
                out << ',' << polyline.get_arc(index).radius << ',' << unsigned(polyline.get_arc(index).orientation);
            out << '/';
        }
        out << ';';
    }
    return out.str();
}

Slic3r::ExtrusionPath flatten_fixture_path(const Slic3r::Points &points)
{
    using namespace Slic3r;
    ExtrusionAttributes attributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.12, 0.45f, 0.2f));
    attributes.no_seam = 1;
    ExtrusionPath path(ArcPolyline(points), attributes, ExtrusionPropertyUPtr{}, false);
    path.set_can_rotate(false);
    path.add_property(ExtrusionPropertySpeed(37.f));
    return path;
}

Slic3r::ExtrusionEntityCollection flatten_fixture(bool sortable)
{
    using namespace Slic3r;
    ExtrusionEntityCollection source(sortable, false);
    source.set_can_rotate(false);
    source.add_property(ExtrusionPropertySpeed(17.f));
    source.append_child(std::make_unique<ExtrusionPath>(
        flatten_fixture_path(Points{{0, 0}, {1000, 0}})));
    std::unique_ptr<ExtrusionEntityCollection> nested = std::make_unique<ExtrusionEntityCollection>(false, false);
    nested->set_can_rotate(false);
    nested->add_property(ExtrusionPropertySpeed(29.f));
    nested->append_child(std::make_unique<ExtrusionPath>(
        flatten_fixture_path(Points{{10000, 0}, {11000, 0}})));
    nested->append_child(std::make_unique<ExtrusionPath>(
        flatten_fixture_path(Points{{20000, 0}, {21000, 0}})));
    source.append_child(std::move(nested));
    std::unique_ptr<ExtrusionLoop> loop = std::make_unique<ExtrusionLoop>(flatten_fixture_path(
        Points{{40000, 0}, {41000, 0}, {41000, 1000}, {40000, 0}}));
    loop->add_property(ExtrusionPropertySpeed(47.f));
    source.append_child(std::move(loop));
    const ExtrusionPaths joined{
        flatten_fixture_path(Points{{50000, 0}, {51000, 0}}),
        flatten_fixture_path(Points{{51000, 0}, {52000, 0}})};
    std::unique_ptr<ExtrusionMultiPath> multi = std::make_unique<ExtrusionMultiPath>(joined);
    multi->add_property(ExtrusionPropertySpeed(53.f));
    source.append_child(std::move(multi));
    return source;
}

void check_rebuilt_flatten_group(const Slic3r::ExtrusionEntity &source, const Slic3r::ExtrusionEntity &group)
{
    using namespace Slic3r;
    REQUIRE(source.is_collection());
    REQUIRE(source.child_count() > 1);
    REQUIRE(source.has_properties());
    REQUIRE(typeid(group) == typeid(ExtrusionEntityCollection));
    CHECK(&group != &source);
    CHECK(group.get_id() != source.get_id());
    CHECK(group.can_sort() == source.can_sort());
    CHECK(group.can_reverse() == source.can_reverse());
    CHECK(group.can_rotate() == source.can_rotate());
    CHECK(ApiInternal::ExtrusionPropertyAccess::property_count(group) == 0);
}

} // namespace
