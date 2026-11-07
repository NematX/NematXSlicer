
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
#include "test_native_fill_goldens.hpp"
#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/Fill/FillSmooth.hpp"
#include "libslic3r/Surface.hpp"

/* Check generic path and ring builders directly, before source publication.
   Hand-built inputs cover valid and invalid paths, exact endpoint closure,
   degenerate rings, copy/consume ownership and overload-specific winding. A
   retained prefix proves that append operations do not replace prior children.
   Snapshots include geometry, direct attributes, permissions and alpha-renamed
   IDs; generic descendants are required independently of their legacy shell.
   Only named builder references are exercised here. Other native-fill references
   in the shared table are not evidence for producers until their cases run.

       TEST_CASE
           |-- extrusion_entities_append_paths/loops/loops_and_paths()
           `-- record_fill()
               |-- fill_snapshot() -> write_fill_tree() [complete stored state]
               |-- fill_digest() [named immutable reference]
               `-- require_generic_fill() [owned descendant representation]
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

// Freeze a complete tree without tying its oracle to absolute IDs or dynamic types.
void write_fill_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids);
// Return the full stored tree text, including direct scopes and explicit Z samples.
std::string fill_snapshot(const ExtrusionEntity &entity);
// Hash the complete tree text for the pre-migration oracle.
std::string fill_digest(const std::string &text);
// Check generic nodes and the fixed child count at this fixture's recorded boundary.
void require_generic_fill(const ExtrusionEntity &entity, const std::string &path,
                          const Slic3r::Test::NativeFillGolden &golden, bool &found_delegate);
// Compare complete stored output and require direct generic construction of owned subtrees.
void record_fill(const std::string &name, const ExtrusionEntity &entity);


void write_fill_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids)
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
        REQUIRE(line->size() >= 2);
        REQUIRE(attributes != nullptr);
        out << 'z' << line->has_z_offset();
        for (size_t index = 0; index < line->size(); ++index) {
            out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y();
            if (line->has_z_offset())
                out << ',' << line->z_offset(index);
            out << ')';
        }
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_fill_tree(*child, out, ids);
    }
    out << '}';
}
std::string fill_snapshot(const ExtrusionEntity &entity)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    std::map<uint64_t, size_t> ids;
    write_fill_tree(entity, out, ids);
    out << entity.total_volume();
    return out.str();
}

std::string fill_digest(const std::string &text)
{
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(text.data(), text.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream result;
    for (const unsigned int word : words)
        result << std::hex << std::setfill('0') << std::setw(8) << word;
    return result.str();
}

void require_generic_fill(const ExtrusionEntity &entity, const std::string &path,
                          const Slic3r::Test::NativeFillGolden &golden, bool &found_delegate)
{
    INFO(path);
    if (!path.empty())
        REQUIRE(typeid(entity) == typeid(ExtrusionEntity));
    if (golden.delegate_count != 0 && path == golden.delegate_parent) {
        REQUIRE_FALSE(found_delegate);
        REQUIRE(entity.child_count() == golden.delegate_count);
        found_delegate = true;
    }
    for (size_t index = 0; index < entity.child_count(); ++index)
        require_generic_fill(entity.child(index), path.empty() ? std::to_string(index) : path + '.' + std::to_string(index), golden, found_delegate);
}

void record_fill(const std::string &name, const ExtrusionEntity &entity)
{
    INFO(name);
    const Slic3r::Test::NativeFillGolden *golden = nullptr;
    for (const Slic3r::Test::NativeFillGolden &candidate : Slic3r::Test::native_fill_goldens)
        if (name == candidate.name) {
            REQUIRE(golden == nullptr);
            golden = &candidate;
        }
    REQUIRE(golden != nullptr);
    const std::string tree = fill_snapshot(entity);
    CHECK(fill_digest(tree) == golden->digest);
    bool found_delegate = false;
    require_generic_fill(entity, "", *golden, found_delegate);
    CHECK(found_delegate == (golden->delegate_count != 0));
}


} // namespace

TEST_CASE("Native geometry builders preserve overload specific inputs and closure", "[plugins][native-fill]")
{
    const ExtrusionAttributes attributes{ExtrusionRole::SolidInfill, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    for (size_t method = 0; method < 5; ++method)
        for (const bool can_reverse : {false, true}) {
            CAPTURE(method, can_reverse);
            ExtrusionEntity output(ExtrusionEntity::Children{}, true, true, false);
            ExtrusionEntityUPtr prefix = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{{-5000000, 0}, {-4000000, 0}}));
            prefix->add_property(attributes);
            const ExtrusionEntity *prefix_handle = prefix.get();
            output.append_child(std::move(prefix));
            Polygon clockwise = rectangle_expolygon(0., 0., 3., 2.).contour;
            clockwise.make_clockwise();
            Polylines lines{Polyline{Point(0, 0), Point(1000000, 0)}, clockwise.split_at_first_point(),
                Polyline{Point(0, 0)}, Polyline{Point(7000000, 0), Point(7000000, 0)}};
            const Polylines before_lines = lines;
            Polygons loops{clockwise, Polygon{Point(0, 0), Point(1000000, 0)},
                Polygon{Point(0, 0), Point(1000000, 0), Point(2000000, 0)}};
            const Polygons before_loops = loops;
            if (method == 0)
                extrusion_entities_append_paths(output, lines, attributes.extrusion_role(), attributes.mm3_per_mm,
                                                 attributes.width, attributes.height, can_reverse);
            else if (method == 1)
                extrusion_entities_append_paths(output, std::move(lines), ExtrusionAttributes(attributes), can_reverse);
            else if (method == 2)
                extrusion_entities_append_loops(output, loops, ExtrusionAttributes(attributes), can_reverse);
            else if (method == 3)
                extrusion_entities_append_loops(output, std::move(loops), ExtrusionAttributes(attributes), can_reverse);
            else
                extrusion_entities_append_loops_and_paths(output, std::move(lines), ExtrusionAttributes(attributes), can_reverse);
            CHECK(&output.child(0) == prefix_handle);
            if (method == 0) {
                REQUIRE(lines.size() == before_lines.size());
                for (size_t index = 0; index < lines.size(); ++index)
                    CHECK(lines[index].points == before_lines[index].points);
            } else if (method == 2) {
                REQUIRE(loops.size() == before_loops.size());
                for (size_t index = 0; index < loops.size(); ++index)
                    CHECK(loops[index].points == before_loops[index].points);
            } else if (method == 3) {
                CHECK(loops.empty());
            } else {
                CHECK(lines.empty());
            }
            CHECK(output.child_count() == (method == 2 || method == 3 ? 3 : 4));
            record_fill("builder-" + std::to_string(method) + '-' + std::to_string(can_reverse), output);
        }
}
