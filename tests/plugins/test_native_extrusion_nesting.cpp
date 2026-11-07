#include <catch2/catch.hpp>

#include <boost/uuid/detail/sha1.hpp>

#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Brim.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Surface.hpp"

/* Check native brim construction before auxiliary-layer or SDK publication.
   Hand-built nesting trees make parent-before-descendant order explicit; the
   clipping fixtures cover open paths, removed parents and single-child moves.
   Digests retain geometry, property values, permissions and repeated-ID
   relationships, not absolute IDs or dynamic types. Separate checks require
   generic owned nodes and complete marked parent/leaf attributes. Concentric
   ears retain their distinct leaf flow and unmarked parent scopes.

       TEST_CASE -> brim_loops() / brim_frontiers(): prepare clipping inputs
           |-- extrude_brim_from_tree() or make_brim_ears(): build owned output
           |-- tree_digest() -> write_tree(): compare stored content
           `-- require_generic_tree() / require_brim_scopes(): check representation
*/

namespace Slic3r {
// Exercise the native tree producer before auxiliary-layer or SDK publication.
void extrude_brim_from_tree(const Print &print, std::vector<std::vector<BrimLoop>> &loops,
                            const Polygons &frontiers, const Flow &flow,
                            ExtrusionEntity &out, bool reversed);
}

namespace {

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

// Serialize stored state in order, retaining repeated-ID relationships but not absolute IDs.
void write_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids);
// Hash the complete deterministic tree text for a frozen producer regression oracle.
std::string tree_digest(const ExtrusionEntity &entity);
// Construct clipping and nesting cases independently of the brim offset generator.
std::vector<std::vector<BrimLoop>> brim_loops(size_t shape);
// Return a full frontier or two disjoint regions that remove the outer ring.
Polygons brim_frontiers(size_t shape);
// Require direct generic construction at every node of an owned producer subtree.
void require_generic_tree(const ExtrusionEntity &entity);
// Check parent/leaf property scopes without changing the generated tree.
void require_brim_scopes(const ExtrusionEntity &entity, bool marked);

const char *const brim_digests[2][5] = {
    {"97ba032669547e727eeb6f465246924bf2efbce7", "14fedba973bc04d9ce3cb3c13ff16275ab05c7d7",
     "1d6dab095fab35ad5bf2a1c09802a924e37562f6", "c714e9a2e1220729cd73c4b37fe85d85e41f4074",
     "b2cd58455784a88974c3810bd1a192238cbc3c21"},
    {"4a649154ad0e82bddec444c43b60cc868a53a75d", "38c280a9bba9c2999eb9a5d2cc69567c6b236b9b",
     "810bfa2e93f08d675d5c20b7f17f216bd0aa15ef", "4afc023c2a6b58eacd04b083f73900bf69c49eb7",
     "0be76d8b27c8bab917d2de58ce60cfdc136530dc"}
};


void write_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids)
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
            << attributes->width << ',' << attributes->height << ',' << attributes->no_seam;
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
        write_tree(*child, out, ids);
    }
    out << '}';
}

std::string tree_digest(const ExtrusionEntity &entity)
{
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::hexfloat;
    std::map<uint64_t, size_t> ids;
    write_tree(entity, text, ids);
    text << entity.total_volume();
    const std::string bytes = text.str();
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(bytes.data(), bytes.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream result;
    for (const unsigned int word : words)
        result << std::hex << std::setfill('0') << std::setw(8) << word;
    return result.str();
}

std::vector<std::vector<BrimLoop>> brim_loops(size_t shape)
{
    std::vector<std::vector<BrimLoop>> loops(1);
    if (shape == 0) {
        loops.front().emplace_back(rectangle_expolygon(0., 0., 20., 20.).contour);
    } else if (shape == 1) {
        loops.front().emplace_back(Polyline{Point(scale_i(-2.), scale_i(5.)), Point(scale_i(22.), scale_i(5.))});
    } else {
        loops.front().emplace_back(rectangle_expolygon(0., 0., 20., 20.).contour);
        BrimLoop &parent = loops.front().back();
        parent.children.emplace_back(rectangle_expolygon(2., 2., 8., 8.).contour);
        if (shape != 3)
            parent.children.emplace_back(rectangle_expolygon(12., 12., 18., 18.).contour);
        if (shape == 2)
            parent.children.front().children.emplace_back(rectangle_expolygon(3., 3., 7., 7.).contour);
    }
    return loops;
}

Polygons brim_frontiers(size_t shape)
{
    if (shape >= 3)
        return {rectangle_expolygon(1., 1., 9., 9.).contour,
                rectangle_expolygon(11., 11., 19., 19.).contour};
    if (shape == 1)
        return {rectangle_expolygon(0., 0., 20., 20.).contour};
    return {rectangle_expolygon(-1., -1., 21., 21.).contour};
}

void require_generic_tree(const ExtrusionEntity &entity)
{
    REQUIRE(typeid(entity) == typeid(ExtrusionEntity));
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        require_generic_tree(*child);
    }
}

void require_brim_scopes(const ExtrusionEntity &entity, bool marked)
{
    const ExtrusionPropertyLoopRole *loop = entity.get_property<ExtrusionPropertyLoopRole>();
    if (entity.child_count() == 1 && entity.child(0).has_polyline()) {
        REQUIRE(entity.is_loop());
        CHECK_FALSE(entity.can_reverse());
        CHECK(entity.can_rotate());
        const ExtrusionAttributes *leaf = entity.child(0).get_property<ExtrusionAttributes>();
        REQUIRE(leaf != nullptr);
        const ExtrusionAttributes *parent = entity.get_property<ExtrusionAttributes>();
        if (marked) {
            REQUIRE(loop != nullptr);
            CHECK(loop->perimeter_role() == elrSkirt);
            REQUIRE(parent != nullptr);
            CHECK(parent != leaf);
            CHECK(parent->role == leaf->role);
            CHECK(parent->width == leaf->width);
            CHECK(parent->height == leaf->height);
            CHECK(parent->mm3_per_mm == leaf->mm3_per_mm);
            CHECK(parent->no_seam == leaf->no_seam);
        } else {
            CHECK(loop == nullptr);
            CHECK(parent == nullptr);
        }
    }
    if (entity.has_polyline()) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        CHECK(attributes->extrusion_role() == ExtrusionRole::Skirt);
        CHECK(entity.can_reverse() == !marked);
        CHECK(entity.can_rotate());
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        require_brim_scopes(*child, marked);
}

} // namespace

TEST_CASE("Native brim trees retain clipping nesting and complete properties",
          "[plugins][native-nesting][native-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool clockwise : {false, true}) {
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({{"perimeter_direction", clockwise ? "cw_cw" : "ccw_ccw"}}));
        const Flow flow = Flow::new_from_width(0.45f, 0.4f, 0.2f, 1.f, false);
        for (size_t shape = 0; shape < 5; ++shape)
            for (const bool reversed : {false, true}) {
                CAPTURE(shape, clockwise, reversed);
                std::vector<std::vector<BrimLoop>> loops = brim_loops(shape);
                ExtrusionEntityCollection output;
                ExtrusionEntity &borrowed_output = output;
                extrude_brim_from_tree(prepared.print, loops, brim_frontiers(shape), flow, borrowed_output, reversed);
                REQUIRE_FALSE(output.empty());
                CHECK(output.total_volume() > 0.);
                CHECK(tree_digest(output) == brim_digests[clockwise][shape]);
                for (const ExtrusionEntityUPtr &child : output.children()) {
                    require_generic_tree(*child);
                    require_brim_scopes(*child, true);
                }
            }
    }
}

TEST_CASE("Native concentric brim ears retain their distinct leaf flow",
          "[plugins][native-nesting][native-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool clockwise : {false, true}) {
        CAPTURE(clockwise);
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({{"brim_width", "3"},
            {"perimeter_direction", clockwise ? "cw_cw" : "ccw_ccw"}}));
        const Flow flow = Flow::new_from_width(0.45f, 0.4f, 0.2f, 1.f, false);
        BrimGenerationParameters parameters;
        parameters.ear_pattern = BrimEarPattern::Concentric;
        ExPolygons unbrimmable;
        ExtrusionEntityCollection output;
        ExtrusionEntity &borrowed_output = output;
        make_brim_ears(prepared.print, flow, {&prepared.print.objects().front()}, parameters, unbrimmable, borrowed_output);
        REQUIRE_FALSE(output.empty());
        REQUIRE_FALSE(unbrimmable.empty());
        CHECK(output.total_volume() > 0.);
        CHECK(tree_digest(output) == "00257a929fb406436db93a9568be559b0273a691");
        for (const ExtrusionEntityUPtr &child : output.children()) {
            require_generic_tree(*child);
            require_brim_scopes(*child, false);
        }
    }
}

TEST_CASE("Native brim discards clipped empty descendant groups",
          "[plugins][native-nesting][native-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({}));
    const Flow flow = Flow::new_from_width(0.45f, 0.4f, 0.2f, 1.f, false);
    for (const size_t shape : {size_t(0), size_t(2), size_t(3)}) {
        CAPTURE(shape);
        std::vector<std::vector<BrimLoop>> loops = brim_loops(shape);
        ExtrusionEntityCollection output;
        ExtrusionEntity &borrowed_output = output;
        extrude_brim_from_tree(prepared.print, loops, {}, flow, borrowed_output, false);
        CHECK(output.child_count() == 0);
        CHECK(output.empty());
        CHECK(output.total_volume() == 0.);
    }
}

TEST_CASE("Native brim transfers the only surviving child out of temporary groups",
          "[plugins][native-nesting][native-brim]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool clockwise : {false, true}) {
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({{"perimeter_direction", clockwise ? "cw_cw" : "ccw_ccw"}}));
        const Flow flow = Flow::new_from_width(0.45f, 0.4f, 0.2f, 1.f, false);
        for (const bool keep_parent : {false, true}) {
            CAPTURE(clockwise, keep_parent);
            std::vector<std::vector<BrimLoop>> loops = brim_loops(4);
            REQUIRE(loops.front().front().children.size() == 2);
            const Polygons frontiers{rectangle_expolygon(keep_parent ? -1. : 1.,
                keep_parent ? -1. : 1., keep_parent ? 10. : 9., keep_parent ? 21. : 9.).contour};
            Polylines expected_parent = intersection_pl(Polygons{loops.front().front().polygon()}, frontiers);
            Polylines expected_child = intersection_pl(Polygons{loops.front().front().children.front().polygon()}, frontiers);
            REQUIRE(expected_parent.empty() == !keep_parent);
            REQUIRE(expected_child.size() == 1);
            REQUIRE(expected_child.front().front() == expected_child.front().back());
            if (clockwise) {
                for (Polyline &line : expected_parent)
                    line.reverse();
                expected_child.front().reverse();
            }

            ExtrusionEntityCollection output;
            ExtrusionEntity &borrowed_output = output;
            extrude_brim_from_tree(prepared.print, loops, frontiers, flow, borrowed_output, false);
            REQUIRE(output.child_count() == 1);
            const BrimLoop &source = loops.front().front();
            REQUIRE_FALSE(source.children.front().lines.empty());
            REQUIRE(source.children.back().lines.empty());
            CHECK(source.lines.empty() == !keep_parent);
            const ExtrusionEntity &group = output.child(0);
            require_generic_tree(group);
            require_brim_scopes(group, true);
            if (keep_parent) {
                CHECK_FALSE(group.can_sort());
                CHECK_FALSE(group.can_reverse());
                REQUIRE(group.child_count() == expected_parent.size() + 1);
                for (size_t index = 0; index < expected_parent.size(); ++index) {
                    const ExtrusionEntity &path = group.child(index);
                    REQUIRE(path.has_polyline());
                    CHECK_FALSE(path.is_loop());
                    CHECK(path.polyline_ref().to_polyline().points == expected_parent[index].points);
                }
            }
            const ExtrusionEntity &ring = keep_parent ? group.child(expected_parent.size()) : group;
            REQUIRE(ring.child_count() == 1);
            REQUIRE(ring.child(0).has_polyline());
            CHECK(ring.polyline_or_null() == nullptr);
            CHECK(ring.child(0).polyline_ref().to_polyline().points == expected_child.front().points);
            CHECK(ring.get_id() != ring.child(0).get_id());
        }
    }
}
