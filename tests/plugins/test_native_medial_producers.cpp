#include <catch2/catch.hpp>

#include <boost/uuid/detail/sha1.hpp>

#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/ExtrusionGeometryUnits.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/ShortestPath.hpp"
#include "perimeter_test_helpers.hpp"
#include "test_native_medial_goldens.hpp"

/* Characterize generic MedialAxis output before source publication. Small owned
   fixtures cover constant and variable widths, exact and epsilon-close contours,
   both degenerate-return branches, no input and two independent centerlines.
   Named snapshots retain every stored point, direct property, permission and
   alpha-renamed identity. These deterministic inputs impose no Clipper island
   order. Type checks independently require directly constructed base nodes.

   Queries and chaining compare live output with a canonical copy, using separate
   recorded endpoint and route expectations. Direct producer calls do not enable
   the historic perimeter or support pipeline; those require separate regressions.
   Chaining probes mix fixed contours, reversible opens and unsortable groups.
   Borrowed references must retain exact source identities and leave stored trees
   unchanged, including when the input view is marked reversed.
   Native perimeter helpers are also exercised directly on prepared sliced layers:
   contour orientation and thin-wall flow preserve source handles, while public
   processing distinguishes copied perimeter groups from adopted gap-fill output.

       TEST_CASE
           |-- medial_inputs() -> thin_variable_width() [adopt every output]
           |-- medial_probes() -> variable_width() [distinguish degenerate branches]
           `-- record_medial()
               |-- medial_snapshot() -> write_medial_tree() [stored content]
               |-- medial_queries() / medial_chain() [readers leave trees unchanged]
               `-- require_medial_generic() [all owned nodes use the base type]
*/

namespace {

using namespace Slic3r;

// Create the open, variable, closed, near-closed, two flukes, empty and multiple-centerline inputs.
ThickPolylines medial_inputs(size_t shape);
// Serialize stored state without calling virtual endpoint or loop projections.
void write_medial_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids);
// Return the complete stored tree text, including relative identities and direct scopes.
std::string medial_snapshot(const ExtrusionEntity &entity);
// Hash one complete stored-state string with the fixture's stable word order.
std::string medial_digest(const std::string &tree);
// Require directly produced base nodes at every owned level, including all geometric leaves.
void require_medial_generic(const ExtrusionEntity &entity);
// Read the actual virtual endpoints and loop predicates used by native perimeter readers.
std::string medial_queries(const ExtrusionEntity &root);
// Chain producer outputs and seam-near controls with the perimeter reader's start and reversal settings.
std::string medial_chain(ExtrusionEntity &root);
// Describe the real intermediate paths so both fluke branches can be distinguished after discretization.
std::string medial_probes(const ThickPolylines &inputs, ExtrusionRole role, const Flow &flow,
                          coord_t resolution, coord_t tolerance, bool can_reverse);
// Compare stored trees and native/generic reader projections with independent fixture records.
void record_medial(const std::string &name, ExtrusionEntity &root);
// Build one fixed ordinary contour for mixed-orientation and nested chaining probes.
ExtrusionEntityUPtr ordinary_perimeter(coord_t shift);
// Keep an ordinary contour before or after a sortable pair under a fixed outer group.
ExtrusionEntityUPtr ordinary_perimeter_group(bool children_before);
// Borrow every node by identity to detect changes in ownership or handles.
void medial_node_handles(const ExtrusionEntity &root, std::map<uint64_t, const ExtrusionEntity *> &handles);
// Compare a complete before/after state with its independent native reader reference.
void record_perimeter_reader(const std::string &name, const ExtrusionEntity &root);
// Encode absolute reversal flags separately from the stored-state references.
std::string medial_chain_text(const std::vector<std::pair<size_t, bool>> &chain);

ThickPolylines medial_inputs(size_t shape)
{
    ThickPolyline constant;
    constant.points = {Point(0, 0), Point(scale_i(4.), scale_i(3.))};
    constant.points_width.assign(constant.points.size(), scale_i(0.45));
    constant.endpoints = {true, true};
    ThickPolyline variable;
    variable.points = {Point(0, 0), Point(scale_i(3.), 0), Point(scale_i(6.), scale_i(1.)), Point(scale_i(9.), scale_i(1.))};
    variable.points_width = {scale_i(0.45), scale_i(0.45), scale_i(0.7), scale_i(0.31)};
    variable.endpoints = {true, true};
    if (shape == 0)
        return {constant};
    if (shape == 1)
        return {variable};
    if (shape == 6)
        return {};
    if (shape == 7) {
        for (Point &point : variable.points)
            point.y() += scale_i(12.);
        return {constant, variable};
    }
    ThickPolyline loop;
    loop.points = {Point(0, 0), Point(scale_i(5.), 0), Point(scale_i(5.), scale_i(5.)),
                   Point(0, scale_i(5.)), Point(shape == 2 ? 0 : 1, 0)};
    loop.points_width.assign(loop.points.size(), scale_i(0.45));
    loop.endpoints = {false, false};
    if (shape == 4 || shape == 5) {
        loop.points = {Point(0, 0), Point(scale_i(5.), 0), Point(1, 0)};
        loop.points_width.assign(loop.points.size(), scale_i(0.45));
        if (shape == 5)
            loop.points_width.back() = scale_i(0.75);
    }
    return {loop};
}
void write_medial_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids)
{
    const size_t next_id = ids.size();
    const size_t id = ids.emplace(entity.get_id(), next_id).first->second;
    out << '{' << id << ':' << entity.is_leaf() << entity.can_sort() << entity.can_reverse()
        << entity.can_rotate() << entity.ExtrusionEntity::is_loop() << ':';
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    const ExtrusionPropertyLoopRole *loop = entity.get_property<ExtrusionPropertyLoopRole>();
    CHECK(ApiInternal::ExtrusionPropertyAccess::property_count(entity) == size_t(attributes != nullptr) + size_t(loop != nullptr));
    if (attributes != nullptr)
        out << 'a' << int32_t(attributes->role) << ',' << attributes->mm3_per_mm << ','
            << attributes->width << ',' << attributes->height << ',' << int(attributes->no_seam);
    if (loop != nullptr)
        out << 'l' << loop->perimeter_flags << ',' << loop->perimeter_idx << ',' << loop->reserved;
    if (const ArcPolyline *line = entity.polyline_or_null()) {
        CHECK_FALSE(line->has_arc());
        CHECK_FALSE(line->has_z_offset());
        REQUIRE(line->size() >= 2);
        REQUIRE(attributes != nullptr);
        for (size_t index = 0; index < line->size(); ++index)
            out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y() << ')';
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_medial_tree(*child, out, ids);
    }
    out << '}';
}

std::string medial_snapshot(const ExtrusionEntity &entity)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    std::map<uint64_t, size_t> ids;
    write_medial_tree(entity, out, ids);
    out << entity.total_volume();
    return out.str();
}

std::string medial_digest(const std::string &tree)
{
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(tree.data(), tree.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream hash;
    for (const unsigned int word : words)
        hash << std::hex << std::setfill('0') << std::setw(8) << word;
    return hash.str();
}

void require_medial_generic(const ExtrusionEntity &entity)
{
    REQUIRE(typeid(entity) == typeid(ExtrusionEntity));
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        require_medial_generic(*child);
    }
}

std::string medial_queries(const ExtrusionEntity &root)
{
    std::ostringstream out;
    for (const ExtrusionEntityUPtr &child : root.children()) {
        REQUIRE_FALSE(child->empty());
        out << child->is_loop() << ':' << child->first_point().x() << ',' << child->first_point().y()
            << '>' << child->last_point().x() << ',' << child->last_point().y() << ';';
    }
    return out.str();
}

std::string medial_chain(ExtrusionEntity &root)
{
    const ExtrusionAttributes attributes{ExtrusionRole::ThinWall, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    ExtrusionEntity left(false, ArcPolyline(Points{{-2, 0}, {-20000000, 0}}));
    ExtrusionEntity right(false, ArcPolyline(Points{{3, 0}, {20000000, 0}}));
    left.add_property(attributes);
    right.add_property(attributes);
    ExtrusionEntitiesPtr entities;
    for (ExtrusionEntityUPtr &child : root.children())
        entities.push_back(child.get());
    entities.push_back(&left);
    entities.push_back(&right);
    const Point start(0, 0);
    const std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(entities, &start, false);
    const std::vector<const ExtrusionEntity *> const_entities(entities.begin(), entities.end());
    CHECK(chain_extrusion_entities(const_entities, &start, false) == chain);
    REQUIRE(chain.size() == entities.size());
    std::vector<bool> used(entities.size(), false);
    std::ostringstream out;
    for (const std::pair<size_t, bool> &item : chain) {
        REQUIRE(item.first < entities.size());
        CHECK_FALSE(used[item.first]);
        used[item.first] = true;
        CHECK((entities[item.first]->can_reverse() || !item.second));
        out << item.first << ':' << item.second << ';';
    }
    return out.str();
}

std::string medial_probes(const ThickPolylines &inputs, ExtrusionRole role, const Flow &flow,
                          coord_t resolution, coord_t tolerance, bool can_reverse)
{
    if (tolerance <= 0)
        tolerance = flow.scaled_width() / 20;
    std::ostringstream out;
    for (const ThickPolyline &input : inputs) {
        const ExtrusionEntity paths = Geometry::variable_width(input, role, flow, resolution, tolerance, can_reverse);
        require_medial_generic(paths);
        REQUIRE(paths.child_count() != 0);
        size_t segments = 0;
        for (const ExtrusionEntityUPtr &path : paths.children()) {
            REQUIRE(path->polyline_or_null() != nullptr);
            segments += path->polyline_ref().size() - 1;
        }
        const bool near_closed = paths.ExtrusionEntity::first_point().coincides_with_epsilon(paths.ExtrusionEntity::last_point());
        out << paths.child_count() << ',' << segments << ',' << paths.child(0).polyline_ref().size() << ',' << near_closed << ';';
    }
    return out.str();
}

void record_medial(const std::string &name, ExtrusionEntity &root)
{
    INFO(name);
    const Slic3r::Test::NativeMedialGolden *golden = nullptr;
    for (const Slic3r::Test::NativeMedialGolden &candidate : Slic3r::Test::native_medial_goldens)
        if (name == candidate.name) {
            REQUIRE(golden == nullptr);
            golden = &candidate;
        }
    REQUIRE(golden != nullptr);
    require_medial_generic(root);
    const std::string tree = medial_snapshot(root);
    ExtrusionEntityUPtr generic = root.clone_canonical();
    REQUIRE(medial_snapshot(*generic) == tree);
    const std::string native_queries = medial_queries(root);
    const std::string generic_queries = medial_queries(*generic);
    const std::string native_chain = medial_chain(root);
    const std::string generic_chain = medial_chain(*generic);
    CHECK(medial_snapshot(root) == tree);
    CHECK(medial_snapshot(*generic) == tree);
    CHECK(medial_digest(tree) == golden->digest);
    CHECK(native_queries == Slic3r::Test::native_medial_queries[golden->generic_query]);
    CHECK(generic_queries == Slic3r::Test::native_medial_queries[golden->generic_query]);
    CHECK(native_chain == Slic3r::Test::native_medial_chains[golden->generic_chain]);
    CHECK(generic_chain == Slic3r::Test::native_medial_chains[golden->generic_chain]);
}

ExtrusionEntityUPtr ordinary_perimeter(coord_t shift)
{
    const ExtrusionAttributes attributes{ExtrusionRole::ExternalPerimeter, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    ExtrusionPaths paths;
    paths.emplace_back(ArcPolyline(Points{{shift, 0}, {shift + 4000000, 0}, {shift + 4000000, 4000000},
                                         {shift, 4000000}, {shift, 0}}), attributes, nullptr, false);
    return std::make_unique<ExtrusionLoop>(std::move(paths));
}

ExtrusionEntityUPtr ordinary_perimeter_group(bool children_before)
{
    ExtrusionEntityUPtr group = std::make_unique<ExtrusionEntityCollection>(false, false);
    ExtrusionEntityUPtr children = std::make_unique<ExtrusionEntityCollection>();
    children->append_child(ordinary_perimeter(0));
    children->append_child(ordinary_perimeter(10000000));
    if (children_before)
        group->append_child(std::move(children));
    group->append_child(ordinary_perimeter(20000000));
    if (!children_before)
        group->append_child(std::move(children));
    return group;
}

void medial_node_handles(const ExtrusionEntity &root, std::map<uint64_t, const ExtrusionEntity *> &handles)
{
    REQUIRE(handles.emplace(root.get_id(), &root).second);
    for (const ExtrusionEntityUPtr &child : root.children()) {
        REQUIRE(child != nullptr);
        medial_node_handles(*child, handles);
    }
}

void record_perimeter_reader(const std::string &name, const ExtrusionEntity &root)
{
    INFO(name);
    const Slic3r::Test::NativePerimeterReaderGolden *golden = nullptr;
    for (const Slic3r::Test::NativePerimeterReaderGolden &candidate : Slic3r::Test::native_perimeter_reader_goldens)
        if (name == candidate.name) {
            REQUIRE(golden == nullptr);
            golden = &candidate;
        }
    REQUIRE(golden != nullptr);
    const std::string tree = medial_snapshot(root);
    CHECK(medial_digest(tree) == golden->digest);
}

std::string medial_chain_text(const std::vector<std::pair<size_t, bool>> &chain)
{
    std::ostringstream out;
    for (const std::pair<size_t, bool> &item : chain)
        out << item.first << ':' << item.second << ';';
    return out.str();
}

} // namespace

TEST_CASE("Native variable width producers separate stored output from legacy projections", "[plugins][native-medial]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    const ExtrusionRole roles[] = {ExtrusionRole::ThinWall, ExtrusionRole::GapFill, ExtrusionRole::SolidInfill};
    for (size_t shape = 0; shape < 8; ++shape)
        for (size_t role_index = 0; role_index < 3; ++role_index)
            for (const bool can_reverse : {false, true})
                for (size_t method = 0; method < 4; ++method) {
                    CAPTURE(shape, role_index, can_reverse, method);
                    const ThickPolylines inputs = medial_inputs(shape);
                    const coord_t resolution = shape == 5 ? scale_i(20.) : flow.scaled_width() / 8;
                    const coord_t tolerance = method == 1 ? 0 : (method == 2 ? -1 : scale_i(0.02));
                    ExtrusionEntitiesPtr raw = method == 0 ?
                        Geometry::thin_variable_width(inputs, roles[role_index], flow, resolution, can_reverse) :
                        Geometry::thin_variable_width(inputs, roles[role_index], flow, resolution, tolerance, can_reverse);
                    ExtrusionEntity root(ExtrusionEntity::Children{}, true, true, false);
                    root.children().reserve(raw.size());
                    for (ExtrusionEntity *entity : raw)
                        root.append_child(ExtrusionEntityUPtr(entity));
                    raw.clear();
                    REQUIRE(root.child_count() == inputs.size());
                    const std::string probes = medial_probes(inputs, roles[role_index], flow, resolution,
                        method == 0 ? 0 : tolerance, can_reverse);
                    const std::string name = std::to_string(shape) + '-' + std::to_string(role_index) + '-'
                        + std::to_string(can_reverse) + '-' + std::to_string(method);
                    if (shape == 4)
                        CHECK(probes == "1,2,3,1;");
                    if (shape == 5)
                        CHECK(probes == "2,2,2,1;");
                    record_medial(name, root);
                }
}

TEST_CASE("Native thin wall orientation preserves open paths and contour handles", "[plugins][native-perimeter-readers]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    for (const bool generic : {false, true})
      for (size_t shape = 0; shape < 8; ++shape)
        for (const bool mirror : {false, true}) {
            CAPTURE(generic, shape, mirror);
            ThickPolylines inputs = medial_inputs(shape);
            if (mirror)
                for (ThickPolyline &line : inputs)
                    for (Point &point : line.points)
                        point.y() = -point.y();
            const coord_t resolution = shape == 5 ? scale_i(20.) : flow.scaled_width() / 8;
            ExtrusionEntitiesPtr raw = Geometry::thin_variable_width(inputs, ExtrusionRole::ThinWall, flow,
                                                                     resolution, scale_i(0.02), false);
            ExtrusionEntityCollection root;
            root.append(std::move(raw));
            ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
            ExtrusionEntity &tested = generic ? *canonical : root;
            std::map<uint64_t, const ExtrusionEntity *> before;
            medial_node_handles(tested, before);
            const std::string original = medial_snapshot(tested);
            const std::string name = "orientation-" + std::to_string(shape) + '-' + std::to_string(mirror);
            record_perimeter_reader(name + "-before", tested);
            for (const ExtrusionEntityUPtr &child : tested.children()) {
                CHECK(PerimeterGenerator::detail::thin_wall_is_contour(*child) == (shape == 2 || shape == 3));
                PerimeterGenerator::detail::orient_thin_wall(*child);
            }
            std::map<uint64_t, const ExtrusionEntity *> after;
            medial_node_handles(tested, after);
            CHECK(after == before);
            if ((shape != 2 && shape != 3) || !mirror)
                CHECK(medial_snapshot(tested) == original);
            else
                CHECK(medial_snapshot(tested) != original);
            record_perimeter_reader(name + "-after", tested);
        }
}

TEST_CASE("Native sole perimeter group removal transfers only one wrapper", "[plugins][native-perimeter-readers]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    for (const bool generic : {false, true})
      for (const bool initial_rotation : {false, true})
        for (size_t shape = 0; shape < 8; ++shape) {
            CAPTURE(generic, initial_rotation, shape);
            const coord_t resolution = shape == 5 ? scale_i(20.) : flow.scaled_width() / 8;
            ExtrusionEntitiesPtr raw = Geometry::thin_variable_width(medial_inputs(shape), ExtrusionRole::ThinWall,
                                                                     flow, resolution, scale_i(0.02), false);
            ExtrusionEntityCollection root;
            root.set_can_rotate(initial_rotation);
            root.append(std::move(raw));
            ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
            ExtrusionEntity &tested = generic ? *canonical : root;
            std::map<uint64_t, const ExtrusionEntity *> expected;
            medial_node_handles(tested, expected);
            const std::string original = medial_snapshot(tested);
            const bool remove_wrapper = shape == 0 || shape == 1;
            if (remove_wrapper) {
                REQUIRE(tested.child(0).can_rotate());
                expected.erase(tested.child(0).get_id());
            }
            const bool expected_rotation = remove_wrapper ? tested.child(0).can_rotate() : initial_rotation;
            const size_t expected_count = remove_wrapper ? tested.child(0).child_count() : tested.child_count();
            const std::string name = "unwrap-thin-" + std::to_string(shape) + '-' + std::to_string(initial_rotation);
            record_perimeter_reader(name + "-before", tested);
            PerimeterGenerator::detail::unwrap_single_perimeter_group(tested);
            REQUIRE(tested.child_count() == expected_count);
            CHECK(tested.can_sort() == !remove_wrapper);
            CHECK(tested.can_reverse() == !remove_wrapper);
            CHECK(tested.can_rotate() == expected_rotation);
            std::map<uint64_t, const ExtrusionEntity *> actual;
            medial_node_handles(tested, actual);
            CHECK(actual == expected);
            if (!remove_wrapper)
                CHECK(medial_snapshot(tested) == original);
            record_perimeter_reader(name + "-after", tested);
        }

    for (const bool generic : {false, true})
      for (const bool initial_rotation : {false, true})
        for (const bool children_before : {false, true})
            for (size_t count = 0; count < 3; ++count) {
                CAPTURE(generic, initial_rotation, children_before, count);
                ExtrusionEntityCollection root;
                root.set_can_rotate(initial_rotation);
                for (size_t index = 0; index < count; ++index)
                    root.append_child(ordinary_perimeter_group(children_before));
                ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
                ExtrusionEntity &tested = generic ? *canonical : root;
                std::map<uint64_t, const ExtrusionEntity *> expected;
                medial_node_handles(tested, expected);
                const std::string original = medial_snapshot(tested);
                const bool remove_wrapper = count == 1;
                std::vector<const ExtrusionEntity *> promoted;
                if (remove_wrapper) {
                    REQUIRE(tested.child(0).can_rotate());
                    expected.erase(tested.child(0).get_id());
                    for (const ExtrusionEntityUPtr &child : tested.child(0).children())
                        promoted.push_back(child.get());
                }
                const bool expected_rotation = remove_wrapper ? tested.child(0).can_rotate() : initial_rotation;
                const std::string name = "unwrap-ordinary-" + std::to_string(children_before) + '-'
                    + std::to_string(count) + '-' + std::to_string(initial_rotation);
                record_perimeter_reader(name + "-before", tested);
                PerimeterGenerator::detail::unwrap_single_perimeter_group(tested);
                REQUIRE(tested.child_count() == (remove_wrapper ? 2 : count));
                CHECK(tested.can_sort() == !remove_wrapper);
                CHECK(tested.can_reverse() == !remove_wrapper);
                CHECK(tested.can_rotate() == expected_rotation);
                if (remove_wrapper) {
                    REQUIRE(tested.child_count() == promoted.size());
                    for (size_t index = 0; index < promoted.size(); ++index)
                        CHECK(&tested.child(index) == promoted[index]);
                    if (!generic) {
                        REQUIRE(root.entities().size() == promoted.size());
                        for (size_t index = 0; index < promoted.size(); ++index)
                            CHECK(root.entities()[index] == promoted[index]);
                    }
                    const size_t nested = children_before ? 0 : 1;
                    CHECK(tested.child(nested).can_sort());
                    CHECK(tested.child(nested).child_count() == 2);
                } else {
                    CHECK(medial_snapshot(tested) == original);
                }
                std::map<uint64_t, const ExtrusionEntity *> actual;
                medial_node_handles(tested, actual);
                CHECK(actual == expected);
                record_perimeter_reader(name + "-after", tested);
            }
}

TEST_CASE("Native fixed contour chaining preserves permission in the reversed input view", "[plugins][native-chain-reference]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    const Point start(0, 0);
    for (const size_t shape : {size_t(2), size_t(3)}) {
        ExtrusionEntityCollection root;
        root.append(Geometry::thin_variable_width(medial_inputs(shape), ExtrusionRole::ThinWall, flow,
                                                 flow.scaled_width() / 8, scale_i(0.02), false));
        REQUIRE(root.child_count() == 1);
        REQUIRE_FALSE(root.child(0).can_reverse());
        ExtrusionEntityUPtr generic = root.clone_canonical();
        const std::string before = medial_snapshot(root);
        REQUIRE(medial_snapshot(*generic) == before);
        std::vector<ExtrusionEntity *> native_nodes{&root.child(0)};
        std::vector<ExtrusionEntity *> generic_nodes{&generic->child(0)};
        std::vector<const ExtrusionEntity *> native_const{&root.child(0)};
        std::vector<const ExtrusionEntity *> generic_const{&generic->child(0)};
        for (const bool reversed : {false, true})
            for (const bool const_input : {false, true}) {
                CAPTURE(shape, reversed, const_input);
                const std::vector<std::pair<size_t, bool>> native_chain = const_input ?
                    chain_extrusion_entities(native_const, &start, reversed) :
                    chain_extrusion_entities(native_nodes, &start, reversed);
                const std::vector<std::pair<size_t, bool>> generic_chain = const_input ?
                    chain_extrusion_entities(generic_const, &start, reversed) :
                    chain_extrusion_entities(generic_nodes, &start, reversed);
                REQUIRE(native_chain.size() == 1);
                CHECK(native_chain.front().first == 0);
                CHECK_FALSE(native_chain.front().second);
                REQUIRE(generic_chain.size() == 1);
                CHECK(generic_chain == native_chain);
                CHECK(medial_chain_text(native_chain) == "0:0;");
                CHECK_FALSE(generic_chain.front().second);
                CHECK(medial_snapshot(root) == before);
                CHECK(medial_snapshot(*generic) == before);
            }
    }
}

TEST_CASE("Chaining keeps fixed endpoints in mixed and fixed reversed input views", "[plugins][native-chain-reference]")
{
    const ExtrusionAttributes attributes{ExtrusionRole::ThinWall, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    const Point start(0, 0);
    for (const bool generic : {false, true})
        for (const bool second_reversible : {false, true}) {
            CAPTURE(generic, second_reversible);
            ExtrusionEntityCollection root;
            root.append_child(std::make_unique<ExtrusionPath>(
                ArcPolyline(Points{{0, 0}, {scale_i(100.), 0}}), attributes, nullptr, false));
            root.append_child(std::make_unique<ExtrusionPath>(
                ArcPolyline(Points{{scale_i(10.), 0}, {scale_i(20.), 0}}), attributes, nullptr, second_reversible));
            ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
            ExtrusionEntity &tested = generic ? *canonical : root;
            std::vector<ExtrusionEntity *> nodes{&tested.child(0), &tested.child(1)};
            const std::vector<const ExtrusionEntity *> const_nodes(nodes.begin(), nodes.end());
            const std::string original = medial_snapshot(tested);
            std::map<uint64_t, const ExtrusionEntity *> before;
            medial_node_handles(tested, before);
            const std::vector<std::pair<size_t, bool>> expected{{0, false}, {1, second_reversible}};
            for (const bool reversed : {false, true}) {
                CAPTURE(reversed);
                const std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(nodes, &start, reversed);
                CHECK(chain == expected);
                CHECK(chain_extrusion_entities(const_nodes, &start, reversed) == expected);
                for (const std::pair<size_t, bool> &item : chain) {
                    REQUIRE(item.first < nodes.size());
                    CHECK((nodes[item.first]->can_reverse() || !item.second));
                }
                CHECK(medial_snapshot(tested) == original);
                std::map<uint64_t, const ExtrusionEntity *> after;
                medial_node_handles(tested, after);
                CHECK(after == before);
            }
        }

    for (const bool generic : {false, true}) {
        CAPTURE(generic);
        ExtrusionEntityCollection root;
        root.append_child(ordinary_perimeter(0));
        root.append_child(ordinary_perimeter(20000000));
        ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
        ExtrusionEntity &tested = generic ? *canonical : root;
        const Point near_second(20000000, 0);
        std::vector<ExtrusionEntity *> nodes{&tested.child(0), &tested.child(1)};
        const std::vector<const ExtrusionEntity *> const_nodes(nodes.begin(), nodes.end());
        const std::vector<std::pair<size_t, bool>> expected{{1, false}, {0, false}};
        const std::string original = medial_snapshot(tested);
        for (const bool reversed : {false, true}) {
            CAPTURE(reversed);
            CHECK(chain_extrusion_entities(nodes, &near_second, reversed) == expected);
            CHECK(chain_extrusion_entities(const_nodes, &near_second, reversed) == expected);
            CHECK(medial_snapshot(tested) == original);
        }
    }
}

TEST_CASE("Root chaining references preserve direct order and source ownership", "[plugins][native-chain-reference]")
{
    const ExtrusionAttributes attributes{ExtrusionRole::ThinWall, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    ExtrusionEntityCollection root;
    Point start(0, 0);
    const Point *start_near = &start;
    bool sortable = true;
    std::vector<std::pair<size_t, bool>> forward;
    std::vector<std::pair<size_t, bool>> reversed_view;

    SECTION("sortable opens respect fixed endpoints in both input views")
    {
        root.append_child(std::make_unique<ExtrusionPath>(
            ArcPolyline(Points{{0, 0}, {scale_i(100.), 0}}), attributes, nullptr, false));
        root.append_child(std::make_unique<ExtrusionPath>(
            ArcPolyline(Points{{scale_i(10.), 0}, {scale_i(20.), 0}}), attributes, nullptr, true));
        REQUIRE_FALSE(root.child(0).can_reverse());
        REQUIRE(root.child(1).can_reverse());
        forward = {{0, false}, {1, true}};
        reversed_view = forward;
    }
    SECTION("sortable loops start near the second without reversal")
    {
        root.append_child(ordinary_perimeter(0));
        root.append_child(ordinary_perimeter(20000000));
        start = Point(20000000, 0);
        forward = {{1, false}, {0, false}};
        reversed_view = forward;
    }
    SECTION("fixed order ignores the nearby last child and reverses only the permitted open")
    {
        sortable = false;
        root.set_can_sort_reverse(false, false);
        root.append_child(std::make_unique<ExtrusionPath>(
            ArcPolyline(Points{{0, 0}, {scale_i(100.), 0}}), attributes, nullptr, false));
        root.append_child(std::make_unique<ExtrusionPath>(
            ArcPolyline(Points{{scale_i(10.), 0}, {scale_i(20.), 0}}), attributes, nullptr, true));
        root.append_child(ordinary_perimeter(20000000));
        root.child(2).set_can_sort_reverse(false, false);
        REQUIRE_FALSE(root.child(0).can_reverse());
        REQUIRE(root.child(1).can_reverse());
        REQUIRE(root.child(2).is_loop());
        REQUIRE_FALSE(root.child(2).can_reverse());
        start = Point(20000000, 0);
        forward = {{0, false}, {1, false}, {2, false}};
        reversed_view = {{0, false}, {1, true}, {2, false}};
    }
    SECTION("empty Children remain empty in both sorting modes")
    {
        sortable = GENERATE(false, true);
        root.set_can_sort_reverse(sortable, false);
        start_near = nullptr;
    }
    SECTION("fixed direct group does not promote its sortable inner pair")
    {
        sortable = false;
        root.set_can_sort_reverse(false, false);
        root.append_child(ordinary_perimeter_group(false));
        REQUIRE(root.child_count() == 1);
        REQUIRE_FALSE(root.child(0).can_reverse());
        REQUIRE(root.child(0).child_count() == 2);
        REQUIRE(root.child(0).child(1).can_sort());
        REQUIRE(root.child(0).child(1).child_count() == 2);
        start = Point(20000000, 0);
        forward = {{0, false}};
        reversed_view = forward;
    }

    REQUIRE_FALSE(root.is_leaf());
    REQUIRE(root.can_sort() == sortable);
    const std::string original = medial_snapshot(root);
    std::map<uint64_t, const ExtrusionEntity *> original_handles;
    medial_node_handles(root, original_handles);
    std::vector<const ExtrusionEntity *> children;
    for (const ExtrusionEntityUPtr &child : root.children())
        children.push_back(child.get());

    const ExtrusionEntity &borrowed = root;
    for (const bool reversed : {false, true}) {
        CAPTURE(sortable, reversed);
        const std::vector<std::pair<size_t, bool>> &expected = reversed ? reversed_view : forward;
        const ExtrusionEntityReferences actual = chain_extrusion_references(borrowed, start_near, reversed);
        REQUIRE(actual.size() == expected.size());
        for (size_t index = 0; index < expected.size(); ++index) {
            REQUIRE(expected[index].first < children.size());
            CHECK(&actual[index].extrusion_entity() == children[expected[index].first]);
            CHECK(actual[index].flipped() == expected[index].second);
        }
        if (sortable) {
            const ExtrusionEntityReferences direct = chain_extrusion_references(children, start_near, reversed);
            REQUIRE(direct.size() == actual.size());
            for (size_t index = 0; index < direct.size(); ++index) {
                CHECK(&actual[index].extrusion_entity() == &direct[index].extrusion_entity());
                CHECK(actual[index].flipped() == direct[index].flipped());
            }
        }
        CHECK(medial_snapshot(root) == original);
        std::map<uint64_t, const ExtrusionEntity *> handles;
        medial_node_handles(root, handles);
        CHECK(handles == original_handles);
        REQUIRE(root.child_count() == children.size());
        for (size_t index = 0; index < children.size(); ++index)
            CHECK(&root.child(index) == children[index]);
    }
}

TEST_CASE("PG thin wall contour policy retains full and cumulative native flow regimes", "[plugins][native-perimeter-readers]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    const Point anchor(0, 0);
    for (const bool generic : {false, true})
        for (size_t shape = 0; shape < 6; ++shape) {
            CAPTURE(generic, shape);
            ExtrusionEntityCollection root;
            const coord_t resolution = shape == 5 ? scale_i(20.) : flow.scaled_width() / 8;
            root.append(Geometry::thin_variable_width(medial_inputs(shape), ExtrusionRole::ThinWall, flow,
                                                     resolution, scale_i(0.02), false));
            ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
            ExtrusionEntity &tested = generic ? *canonical : root;
            REQUIRE(tested.child_count() == 1);
            const bool full_pass = PerimeterGenerator::detail::thin_wall_is_contour(tested.child(0));
            CHECK(full_pass == (shape == 2 || shape == 3));
            std::map<uint64_t, const ExtrusionEntity *> nodes;
            medial_node_handles(tested, nodes);
            std::map<uint64_t, ExtrusionAttributes> expected;
            for (const std::pair<const uint64_t, const ExtrusionEntity *> &entry : nodes)
                if (entry.second->has_polyline()) {
                    REQUIRE(entry.second->get_property<ExtrusionAttributes>() != nullptr);
                    expected.emplace(entry.first, *entry.second->get_property<ExtrusionAttributes>());
                }
            REQUIRE_FALSE(expected.empty());
            ExtrusionPaths first_copy;
            std::string first_snapshot;
            for (size_t phase = 0; phase < (full_pass ? 1 : 2); ++phase) {
                const float factor = full_pass ? 1.f : (phase == 0 ? 0.1f : 9.f);
                ExtrusionPaths scratch;
                const Point *first = &anchor;
                PerimeterGenerator::detail::append_thin_wall_flow(tested, scratch, first, factor, !full_pass, 1000000.);
                CHECK(first == nullptr);
                REQUIRE_FALSE(scratch.empty());
                for (std::pair<const uint64_t, ExtrusionAttributes> &entry : expected) {
                    entry.second.mm3_per_mm *= factor;
                    entry.second.width *= factor;
                    const ExtrusionAttributes *actual = nodes.at(entry.first)->get_property<ExtrusionAttributes>();
                    REQUIRE(actual != nullptr);
                    CHECK(actual->mm3_per_mm == entry.second.mm3_per_mm);
                    CHECK(actual->width == entry.second.width);
                    CHECK(actual->height == entry.second.height);
                    CHECK(bool(actual->no_seam) == !full_pass);
                }
                for (const ExtrusionPath &path : scratch) {
                    REQUIRE(path.role() == ExtrusionRole::ThinWall);
                    CHECK(path.attributes().mm3_per_mm == expected.at(path.get_id()).mm3_per_mm);
                    CHECK(path.width() == expected.at(path.get_id()).width);
                    CHECK(bool(path.attributes().no_seam) == !full_pass);
                }
                const std::string source_before_return = medial_snapshot(tested);
                if (full_pass) {
                    PerimeterGenerator::detail::ensure_thin_wall_return(scratch, anchor, false, 1000000.);
                    CHECK(scratch.back().last_point() == anchor);
                    CHECK(medial_snapshot(tested) == source_before_return);
                    if (shape == 3)
                        CHECK(tested.child(0).ExtrusionEntity::last_point() == Point(1, 0));
                } else if (phase == 0) {
                    for (ExtrusionPath &path : scratch)
                        path.reverse();
                    std::reverse(scratch.begin(), scratch.end());
                    first_copy = scratch;
                    first_snapshot = medial_snapshot(ExtrusionEntityCollection(first_copy));
                } else {
                    CHECK(medial_snapshot(ExtrusionEntityCollection(first_copy)) == first_snapshot);
                }
            }
            std::map<uint64_t, const ExtrusionEntity *> after;
            medial_node_handles(tested, after);
            CHECK(after == nodes);
        }
}

TEST_CASE("PG thin wall orientation reverses a zero area contour without adding closure", "[plugins][native-perimeter-readers]")
{
    const Flow flow = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, 1.f, false);
    ThickPolyline line;
    line.points = {Point(0, 0), Point(scale_i(3.), 0), Point(scale_i(6.), 0), Point(scale_i(2.), 0), Point(0, 0)};
    line.points_width.assign(line.points.size(), scale_i(0.45));
    line.endpoints = {false, false};
    const Points expected{Point(0, 0), Point(scale_i(2.), 0), Point(scale_i(6.), 0), Point(scale_i(3.), 0), Point(0, 0)};
    for (const bool generic : {false, true}) {
        CAPTURE(generic);
        ExtrusionEntityCollection root;
        root.append(Geometry::thin_variable_width({line}, ExtrusionRole::ThinWall, flow,
                                                 flow.scaled_width() / 8, scale_i(0.02), false));
        ExtrusionEntityUPtr canonical = generic ? root.clone_canonical() : nullptr;
        ExtrusionEntity &tested = generic ? *canonical : root;
        REQUIRE(tested.child_count() == 1);
        ExtrusionEntity &contour = tested.child(0);
        REQUIRE(contour.child_count() == 1);
        REQUIRE(contour.child(0).polyline_or_null() != nullptr);
        REQUIRE(contour.child(0).polyline_ref().to_polyline().points == line.points);
        REQUIRE(PerimeterGenerator::detail::thin_wall_is_contour(contour));
        CHECK(extrusion_geometry_area(contour) == 0.);
        std::map<uint64_t, const ExtrusionEntity *> before;
        medial_node_handles(tested, before);
        PerimeterGenerator::detail::orient_thin_wall(contour);
        CHECK(contour.child(0).polyline_ref().to_polyline().points == expected);
        std::map<uint64_t, const ExtrusionEntity *> after;
        medial_node_handles(tested, after);
        CHECK(after == before);
    }
}

namespace {

// Summarize positive native leaves by role and loop ancestry without retaining borrowed tree handles.
struct NativePGObservations
{
    size_t loops = 0;
    size_t perimeters = 0;
    size_t gap_leaves = 0;
    size_t thin_inside = 0;
    size_t thin_outside = 0;
    coord_t inside_max_x = std::numeric_limits<coord_t>::lowest();
    coord_t outside_max_x = std::numeric_limits<coord_t>::lowest();
};

// Count positive flow leaves and actual Loop ancestors, emitting exact join/type checks without modifying borrowed nodes.
void observe_native_pg_output(const ExtrusionEntity &root, bool inside_loop, NativePGObservations &observed);

void observe_native_pg_output(const ExtrusionEntity &root, bool inside_loop, NativePGObservations &observed)
{
    if (const ExtrusionLoop *loop = dynamic_cast<const ExtrusionLoop *>(&root)) {
        ++observed.loops;
        inside_loop = true;
        REQUIRE(loop->child_count() > 0);
        for (size_t index = 0; index < loop->child_count(); ++index) {
            const ExtrusionEntity &path = loop->child(index);
            const ExtrusionEntity &next = loop->child((index + 1) % loop->child_count());
            REQUIRE(typeid(path) == typeid(ExtrusionPath));
            REQUIRE(path.polyline_or_null() != nullptr);
            REQUIRE(next.polyline_or_null() != nullptr);
            REQUIRE(path.polyline_ref().size() >= 2);
            REQUIRE(next.polyline_ref().size() >= 2);
            CHECK(path.polyline_ref().back() == next.polyline_ref().front());
        }
    }
    if (const ArcPolyline *line = root.polyline_or_null()) {
        REQUIRE(line->size() >= 2);
        const ExtrusionAttributes *attributes = root.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        if (line->length() > 0 && attributes->mm3_per_mm > 0.) {
            REQUIRE(root.total_volume() > 0.);
            if (attributes->extrusion_role().is_perimeter())
                ++observed.perimeters;
            if (attributes->role == ExtrusionRole::GapFill)
                ++observed.gap_leaves;
            if (attributes->role == ExtrusionRole::ThinWall) {
                if (inside_loop)
                    ++observed.thin_inside;
                else
                    ++observed.thin_outside;
                for (size_t index = 0; index < line->size(); ++index) {
                    coord_t &maximum = inside_loop ? observed.inside_max_x : observed.outside_max_x;
                    maximum = std::max(maximum, line->get_point(index).x());
                }
            }
        }
    }
    for (const ExtrusionEntityUPtr &child : root.children()) {
        REQUIRE(child != nullptr);
        observe_native_pg_output(*child, inside_loop, observed);
    }
}

} // namespace

TEST_CASE("Native public perimeter processing distinguishes group copies and raw adoption",
          "[plugins][extrusion][native-perimeter-destination]")
{
    using namespace Slic3r;
    using namespace Slic3r::Test::PerimeterPluginTests;
    int scenario = -1;
    ExPolygon area;
    SECTION("classic has positive perimeter and gap fill")
    {
        scenario = 0;
        area = rectangle_expolygon(0., 0., 12., 1.1);
    }
    SECTION("arachne appends a nonempty perimeter group")
    {
        scenario = 1;
        area = rectangle_with_hole_expolygon();
    }
    SECTION("thin wall is anchored inside an ordinary loop")
    {
        scenario = 2;
        area = ExPolygon(Polygon({Point(0, 0), Point(scale_i(10.), 0),
            Point(scale_i(10.), scale_i(4.875)), Point(scale_i(17.), scale_i(4.875)),
            Point(scale_i(17.), scale_i(5.125)), Point(scale_i(10.), scale_i(5.125)),
            Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))}));
    }
    SECTION("thin wall beyond a narrow neck remains outside ordinary loops")
    {
        scenario = 3;
        area = ExPolygon(Polygon({Point(0, 0), Point(scale_i(10.), 0),
            Point(scale_i(10.), scale_i(4.975)), Point(scale_i(13.), scale_i(4.975)),
            Point(scale_i(13.), scale_i(4.875)), Point(scale_i(20.), scale_i(4.875)),
            Point(scale_i(20.), scale_i(5.125)), Point(scale_i(13.), scale_i(5.125)),
            Point(scale_i(13.), scale_i(5.025)), Point(scale_i(10.), scale_i(5.025)),
            Point(scale_i(10.), scale_i(10.)), Point(0, scale_i(10.))}));
    }
    REQUIRE(scenario >= 0);
    REQUIRE(area.contour.area() > 0.);
    const bool arachne = scenario == 1;
    const bool thin = scenario >= 2;
    const DynamicPrintConfig config = perimeter_config({
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"nozzle_diameter", "0.4"},
        {"perimeter_extrusion_width", "0.45"}, {"external_perimeter_extrusion_width", "0.45"},
        {"perimeter_generator", arachne ? "arachne" : "classic"}, {"perimeters", arachne ? "2" : "1"},
        {"spiral_vase", "0"}, {"fuzzy_skin", "none"}, {"overhangs", "0"},
        {"overhangs_extrusion_spacing", "0"}, {"extra_perimeters_on_overhangs", "0"},
        {"milling_post_process", "0"}, {"milling_extra_size", "0"}, {"perimeter_loop", "0"},
        {"thin_perimeters", "100%"}, {"thin_walls", thin ? "1" : "0"},
        {"thin_walls_merge", thin ? "1" : "0"}, {"thin_walls_min_width", "33%"}, {"thin_walls_overlap", "0.2"},
        {"gap_fill_enabled", scenario == 0 ? "1" : "0"}, {"gap_fill_last", "1"}, {"gap_fill_min_area", "0"}});
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, config);
    replace_all_layer_islands(prepared, area);
    REQUIRE(prepared.print.objects().size() == 1);
    PrintObject &object = prepared.print.object(0);
    REQUIRE(object.layer_count() > 2);
    Layer &layer = object.layer(1);
    REQUIRE(layer.id() == 1);
    REQUIRE(layer.scaled_height() == scale_to_layer_coord(0.2));
    REQUIRE(layer.region_count() == 1);
    REQUIRE(layer.islands().size() == 1);
    const LayerRegion &region = layer.region(0);
    const PrintRegionConfig &region_config = region.region().config();
    CHECK(region_config.perimeter_generator.value ==
          (arachne ? PerimeterGeneratorType::Arachne : PerimeterGeneratorType::Classic));
    REQUIRE(region_config.option("perimeters_hole") != nullptr);
    REQUIRE_FALSE(region_config.option("perimeters_hole")->is_enabled());
    REQUIRE(region_config.option("thin_walls") != nullptr);
    REQUIRE(object.config().option("thin_walls_merge") != nullptr);
    CHECK(region_config.option("thin_walls")->get_bool() == thin);
    CHECK(object.config().option("thin_walls_merge")->get_bool() == thin);
    CHECK_FALSE(region_config.perimeter_loop.value);
    CHECK_FALSE(prepared.print.config().spiral_vase.value);
    CHECK(region_config.fuzzy_skin.value == FuzzySkinType::None);
    CHECK_FALSE(region_config.overhangs.get_bool());
    const Flow perimeter_flow = region.flow(frPerimeter);
    const Flow external_flow = region.flow(frExternalPerimeter);
    CHECK(perimeter_flow.width() == 0.45f);
    CHECK(external_flow.width() == 0.45f);
    PerimeterGenerator::Parameters params(&layer, perimeter_flow, external_flow, perimeter_flow,
        region.flow(frSolidInfill), region_config, object.config(), prepared.print.config(), false,
        region_config.perimeter_generator.value == PerimeterGeneratorType::Arachne, false);
    params.region_setting.segregate_regions(area, layer.island(0).regions());
    params.segregate_extra_perimeters(area, layer.island(0).regions());
    PerimeterGenerator::PerimeterGenerator generator(params);
    ExtrusionEntityCollection perimeters;
    ExtrusionEntityCollection gaps;
    REQUIRE(perimeters.entities().empty());
    REQUIRE(gaps.entities().empty());
    const uint64_t perimeter_id = perimeters.get_id();
    const uint64_t gap_id = gaps.get_id();
    ExPolygons fill;
    ExPolygons no_overlap;
    const ExPolygons slices{area};

    generator.process(area, layer.lower_layer ? &layer.lower_layer->lslices() : nullptr, slices,
        layer.upper_layer ? &layer.upper_layer->lslices() : nullptr, &perimeters, &gaps, fill, no_overlap);

    CHECK(perimeters.get_id() == perimeter_id);
    CHECK(gaps.get_id() == gap_id);
    NativePGObservations perimeter_observed;
    NativePGObservations gap_observed;
    observe_native_pg_output(perimeters, false, perimeter_observed);
    observe_native_pg_output(gaps, false, gap_observed);
    const std::string perimeter_state = medial_snapshot(perimeters);
    const std::string gap_state = medial_snapshot(gaps);
    const char *const expected_perimeters[] = {
        "{0:01110:{1:01110:{2:00011:l35,-1,0{3:10011:a129,0x1.4d7aca0000000p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-"
        "3,0(11775001,875001)(224999,875001)(224999,224999)(11775001,224999)(11775001,875001)}}}}0x1.fc8e7efa3ae25p+0",
        "{0:01110:{1:00010:{2:00011:l35,-1,0{3:10111:a1,0x1.4d7aa40000000p-4,0x1.cccc9e0000000p-2,0x1.99999a0000000p-3,"
        "0(9359338,-5902440)(9359338,9359338)(-9359337,9359337)(-9359337,-9359337)(9359338,-9359338)(9359338,-5902440)}"
        "}{4:00011:{5:10111:a129,0x1.4d7aa40000000p-4,0x1.cccc9e0000000p-2,0x1.99999a0000000p-3,0(9775001,-5902440)(977"
        "5001,9775001)(-9775000,9775000)(-9775000,-9775000)(9775001,-9775001)(9775001,-5902440)}}{6:00011:l43,-1,0{7:10"
        "111:a1,0x1.4d7aa40000000p-4,0x1.cccc9e0000000p-2,0x1.99999a0000000p-3,0(-3640662,-3017168)(-3640662,3017168)(-"
        "3589375,3264806)(-3458045,3458045)(-3264806,3589375)(-3017168,3640662)(3017168,3640662)(3264806,3589375)(34580"
        "45,3458045)(3589375,3264806)(3640662,3017168)(3640662,-3017168)(3589375,-3264806)(3458045,-3458045)(3264806,-3"
        "589375)(3017168,-3640662)(-3017168,-3640662)(-3264806,-3589375)(-3458045,-3458045)(-3589375,-3264806)(-3640662"
        ",-3017168)}}{8:00011:l9,-1,0{9:10111:a129,0x1.4d7aa40000000p-4,0x1.cccc9e0000000p-2,0x1.99999a0000000p-3,0(-32"
        "24999,-3017168)(-3224999,3017168)(-3164127,3164127)(-3017168,3224999)(3017168,3224999)(3164127,3164127)(322499"
        "9,3017168)(3224999,-3017168)(3164127,-3164127)(3017168,-3224999)(-3017168,-3224999)(-3164127,-3164127)(-322499"
        "9,-3017168)}}}}0x1.0d042ad657c34p+4",
        "{0:01110:{1:01110:{2:00011:l35,-1,0{3:10010:a129,0x1.4d7aca0000000p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-"
        "3,0(9775001,9775001)(224999,9775001)(224999,224999)(9775001,224999)(9775001,4999997)}{4:10010:a32,0x0.00000000"
        "00000p+0,0x1.d01db40000000p-8,0x1.99999a0000000p-3,1(9775001,4999997)(9800000,4999997)}{5:10010:a2177,0x1.4919"
        "7e52465f8p-8,0x1.0510b60000000p-4,0x1.99999a0000000p-3,1(9800000,4999997)(9835708,4999997)}{6:10010:a2177,0x1."
        "316174192529cp-7,0x1.5d15a60000000p-4,0x1.99999a0000000p-3,1(9835708,4999997)(9871416,4999997)}{7:10010:a2177,"
        "0x1.3161bc19253bcp-6,0x1.0df9740000000p-3,0x1.99999a0000000p-3,1(9871416,4999997)(9907124,4999997)}{8:10010:a2"
        "177,0x1.ca12bd3f517c0p-6,0x1.6d68160000000p-3,0x1.99999a0000000p-3,1(9907124,4999997)(9942832,4999997)}{9:1001"
        "0:a2177,0x1.3161e0192544cp-5,0x1.ccd6b80000000p-3,0x1.99999a0000000p-3,1(9942832,4999997)(9978540,4999998)(100"
        "00139,4999999)(10031391,4999999)}{10:10010:a2177,0x1.90322dfda6250p-5,0x1.21ad8c0000000p-2,0x1.99999a0000000p-"
        "3,1(10031391,4999999)(10062644,4999999)}{11:10010:a2177,0x1.ef027daef3d28p-5,0x1.5cefbe0000000p-2,0x1.99999a00"
        "00000p-3,1(10062644,4999999)(10093896,4999999)}{12:10010:a2177,0x1.26e9687ced8d4p-4,0x1.9831f00000000p-2,0x1.9"
        "9999a0000000p-3,1(10093896,4999999)(10125149,5000000)(16821460,5000000)(16839306,5000000)}{13:10010:a2177,0x1."
        "f562cf4a25808p-5,0x1.60ebf00000000p-2,0x1.99999a0000000p-3,1(16839306,5000000)(16857153,5000000)}{14:10010:a21"
        "77,0x1.9cf2d13409810p-5,0x1.29a5f20000000p-2,0x1.99999a0000000p-3,1(16857153,5000000)(16875000,5000000)(169062"
        "50,4999999)}{15:10010:a2177,0x1.134c8144d3204p-5,0x1.a73c000000000p-3,0x1.99999a0000000p-3,1(16906250,4999999)"
        "(16937500,4999999)}{16:10010:a2177,0x1.134c5f119fe48p-6,0x1.f658340000000p-4,0x1.99999a0000000p-3,1(16937500,4"
        "999999)(16968750,4999999)}{17:10010:a2177,0x1.49197e52465f8p-8,0x1.0510b60000000p-4,0x1.99999a0000000p-3,1(169"
        "68750,4999999)(17000000,4999999)}{17:10010:a2177,0x1.24887049221c0p-11,0x1.d01db40000000p-8,0x1.99999a0000000p"
        "-3,1(17000000,4999999)(16968750,4999999)}{16:10010:a2177,0x1.e96b53ad8e080p-10,0x1.be874a0000000p-7,0x1.99999a"
        "0000000p-3,1(16968750,4999999)(16937500,4999999)}{15:10010:a2177,0x1.e96b907a5ae40p-9,0x1.7835560000000p-6,0x1"
        ".99999a0000000p-3,1(16937500,4999999)(16906250,4999999)}{14:10010:a2177,0x1.6f10b9f55dc80p-8,0x1.0893820000000"
        "p-5,0x1.99999a0000000p-3,1(16906250,4999999)(16875000,5000000)(16857153,5000000)}{13:10010:a2177,0x1.bdad2a090"
        "4e40p-8,0x1.39b5480000000p-5,0x1.99999a0000000p-3,1(16857153,5000000)(16839306,5000000)}{12:10010:a2177,0x1.06"
        "24cea7ef9a0p-7,0x1.6ad70e0000000p-5,0x1.99999a0000000p-3,1(16839306,5000000)(16821460,5000000)(10125149,500000"
        "0)(10093896,4999999)}{11:10010:a2177,0x1.b80236d466f40p-8,0x1.362a700000000p-5,0x1.99999a0000000p-3,1(10093896"
        ",4999999)(10062644,4999999)}{10:10010:a2177,0x1.63bad38c21e80p-8,0x1.017dd20000000p-5,0x1.99999a0000000p-3,1(1"
        "0062644,4999999)(10031391,4999999)}{9:10010:a2177,0x1.0f7371dd76760p-8,0x1.99a26a0000000p-6,0x1.99999a0000000p"
        "-3,1(10031391,4999999)(10000139,4999999)(9978540,4999998)(9942832,4999997)}{8:10010:a2177,0x1.972d19ff64e00p-9"
        ",0x1.44ce4c0000000p-6,0x1.99999a0000000p-3,1(9942832,4999997)(9907124,4999997)}{7:10010:a2177,0x1.0f7351dd766e"
        "0p-9,0x1.dff45e0000000p-7,0x1.99999a0000000p-3,1(9907124,4999997)(9871416,4999997)}{6:10010:a2177,0x1.0f7311dd"
        "765e0p-10,0x1.364c220000000p-7,0x1.99999a0000000p-3,1(9871416,4999997)(9835708,4999997)}{5:10010:a2177,0x1.248"
        "87049221c0p-11,0x1.d01db40000000p-8,0x1.99999a0000000p-3,1(9835708,4999997)(9800000,4999997)}{18:10010:a32,0x0"
        ".0000000000000p+0,0x1.2212900000000p-4,0x1.99999a0000000p-3,1(9800000,4999997)(9775001,4999997)}{19:10010:a129"
        ",0x1.4d7aca0000000p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-3,0(9775001,4999997)(9775001,224999)(9775001,977"
        "5001)}}}}0x1.1c5a4f74de9c9p+2",
        "{0:01110:{1:01110:{2:00011:l35,-1,0{3:10011:a129,0x1.4d7aca0000000p-4,0x1.cccccc0000000p-2,0x1.99999a0000000p-"
        "3,0(9775001,9775001)(224999,9775001)(224999,224999)(9775001,224999)(9775001,9775001)}}{4:00010:{5:10010:a2177,"
        "0x1.6daa8c0000000p-8,0x1.2212900000000p-4,0x1.99999a0000000p-3,0(20000000,5000000)(19958333,5000000)}{6:10010:"
        "a2177,0x1.cad4c40000000p-6,0x1.76ab9c0000000p-3,0x1.99999a0000000p-3,0(19958333,5000000)(19916666,5000000)}{7:"
        "10010:a2177,0x1.cad4e80000000p-5,0x1.4ab8620000000p-2,0x1.99999a0000000p-3,0(19916666,5000000)(19875000,500000"
        "0)}{8:10010:a2177,0x1.168c3a0000000p-4,0x1.88229a0000000p-2,0x1.99999a0000000p-3,0(19875000,5000000)(19821460,"
        "5000000)}{9:10010:a2177,0x1.47ae020000000p-4,0x1.c58cd20000000p-2,0x1.99999a0000000p-3,0(19821460,5000000)(131"
        "25000,5000000)(13083333,5000000)}{10:10010:a2177,0x1.f151c20000000p-5,0x1.62c66a0000000p-2,0x1.99999a0000000p-"
        "3,0(13083333,5000000)(13041666,5000000)}{11:10010:a2177,0x1.53477e0000000p-5,0x1.0000000000000p-2,0x1.99999a00"
        "00000p-3,0(13041666,5000000)(13000000,5000000)(12978530,5000000)(12919016,5000000)}{12:10010:a2177,0x1.5347560"
        "000000p-6,0x1.2bf3380000000p-3,0x1.99999a0000000p-3,0(12919016,5000000)(12859503,5000000)}{13:10010:a2177,0x1."
        "6daa8c0000000p-8,0x1.2212900000000p-4,0x1.99999a0000000p-3,0(12859503,5000000)(12799990,5000000)}}}}0x1.d53537"
        "26111b1p+1",
    };
    const char *const expected_gaps[] = {
        "{0:01110:{1:10010:a34816,0x1.71f8ce0000000p-5,0x1.132ed20000000p-2,0x1.99999a0000000p-3,0(432801,550000)(49140"
        "1,550000)(550001,550001)(6000001,550001)(11450001,550001)(11508600,550001)(11567200,550001)}}0x1.0176a28659d21"
        "p-1",
        "{0:01110:}0x0.0000000000000p+0",
        "{0:01110:}0x0.0000000000000p+0",
        "{0:01110:}0x0.0000000000000p+0",
    };
    CHECK(perimeter_state == expected_perimeters[scenario]);
    CHECK(gap_state == expected_gaps[scenario]);
    std::cout << "NATIVE_PG_BRANCH " << scenario << " loops=" << perimeter_observed.loops
        << " perimeters=" << perimeter_observed.perimeters << " gaps=" << gap_observed.gap_leaves
        << " inside=" << perimeter_observed.thin_inside << " outside=" << perimeter_observed.thin_outside << '\n';
    CHECK(perimeters.child_count() > 0);
    CHECK(perimeter_observed.perimeters > 0);
    CHECK(perimeters.total_volume() > 0.);
    if (scenario == 0) {
        CHECK(gap_observed.gap_leaves > 0);
        CHECK(gaps.total_volume() > 0.);
    } else if (thin) {
        CHECK(perimeter_observed.loops > 0);
        if (scenario == 2) {
            CHECK(perimeter_observed.thin_inside > 0);
            CHECK(perimeter_observed.inside_max_x > scale_i(11.));
        } else {
            CHECK(perimeter_observed.thin_outside > 0);
            CHECK(perimeter_observed.outside_max_x > scale_i(13.));
        }
    }
    REQUIRE(perimeters.entities().size() == perimeters.child_count());
    REQUIRE(gaps.entities().size() == gaps.child_count());
    for (size_t index = 0; index < perimeters.child_count(); ++index)
        CHECK(perimeters.entities()[index] == &perimeters.child(index));
    for (size_t index = 0; index < gaps.child_count(); ++index)
        CHECK(gaps.entities()[index] == &gaps.child(index));
}
