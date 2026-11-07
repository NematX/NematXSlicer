#include <catch2/catch.hpp>

#include <boost/uuid/detail/sha1.hpp>

#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/ShortestPath.hpp"
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
