#include <catch2/catch.hpp>

#include <boost/uuid/detail/sha1.hpp>

#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <string>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include "test_native_thin_wall_flow_goldens.hpp"

/* Exercise the native perimeter merger's thin-wall flow operations directly.
   Owned Path, Loop and fixed-group inputs distinguish cumulative source changes
   from independent scratch copies. Connection checks cover exact joins, snaps
   below the squared threshold and explicit travel above it. Full snapshots and
   numeric checks retain properties, permissions, identity relationships and
   heap-owned text after source destruction; no perimeter pipeline is run.

       TEST_CASE -> thin_flow_source(): own the source geometry and resources
           |-- append_thin_wall_flow(): mutate source flow and append scratch copies
           |-- ensure_thin_wall_return(): reconnect the scratch endpoint
           `-- record_thin_flow() / require_thin_resources(): verify complete state
*/

namespace {

using namespace Slic3r;

// Build a native Path, closed Loop or fixed open thin-wall collection with owned leaf properties.
ExtrusionEntityUPtr thin_flow_source(size_t form);
// Serialize stored geometry, direct properties, permissions and relative identity relationships.
void write_thin_flow_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids);
// Read one source tree independently of its virtual endpoint or loop classification.
std::string thin_source_snapshot(const ExtrusionEntity &source);
// Serialize two scratch stages and an optional source using one relative identity map.
std::string thin_flow_snapshot(const ExtrusionEntity *source, const ExtrusionPaths &paths, const ExtrusionPaths &second);
// Compare a full stored state with the native reference for this exact stage.
void record_thin_flow(const std::string &name, const ExtrusionEntity &source,
                      const ExtrusionPaths &paths, const ExtrusionPaths &second = {});
// Check every segment-bearing leaf against exact cumulative floating-point operations.
void require_thin_flow(const ExtrusionEntity &entity, double volume, float width, bool no_seam);
// Verify copied resources remain readable after their source tree has been destroyed.
void require_thin_resources(const ExtrusionPaths &paths);

const std::string thin_text = std::string(180, 'x') + " owned thin-wall flow";

ExtrusionEntityUPtr thin_flow_source(size_t form)
{
    const ExtrusionAttributes attributes{ExtrusionRole::ThinWall, ExtrusionFlow{0.12, 0.45f, 0.2f}};
    ExtrusionPaths paths;
    paths.emplace_back(ArcPolyline(Points{{0, 0}, {2000000, 0}}), attributes, nullptr, false);
    if (form != 0)
        paths.emplace_back(form == 1 ? ArcPolyline(Points{{2000000, 0}, {2000000, 1000000}, {0, 0}}) :
                                      ArcPolyline(Points{{2000000, 0}, {3000000, 0}}), attributes, nullptr, true);
    for (ExtrusionPath &path : paths) {
        path.add_property(ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcode::Code::COMMENT, thin_text));
        path.get_or_add_property<ExtrusionPropertySpeed>().speed_mm_per_s = 17.f;
    }
    if (form == 0)
        return std::make_unique<ExtrusionPath>(std::move(paths.front()));
    if (form == 1)
        return std::make_unique<ExtrusionLoop>(std::move(paths));
    ExtrusionEntityUPtr root = std::make_unique<ExtrusionEntityCollection>(false, false);
    for (ExtrusionPath &path : paths)
        root->append_child(std::make_unique<ExtrusionPath>(std::move(path)));
    return root;
}

void write_thin_flow_tree(const ExtrusionEntity &entity, std::ostringstream &out, std::map<uint64_t, size_t> &ids)
{
    const size_t next_id = ids.size();
    const size_t id = ids.emplace(entity.get_id(), next_id).first->second;
    out << '{' << id << ':' << entity.is_leaf() << entity.can_sort() << entity.can_reverse()
        << entity.can_rotate() << entity.ExtrusionEntity::is_loop() << ':'
        << ApiInternal::ExtrusionPropertyAccess::property_count(entity) << ':';
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        out << 'a' << int32_t(attributes->role) << ',' << attributes->mm3_per_mm << ','
            << attributes->width << ',' << attributes->height << ',' << int(attributes->no_seam);
    if (const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>())
        out << 's' << speed->speed_mm_per_s;
    if (const ExtrusionPropertyCustomGcode *text = entity.get_property<ExtrusionPropertyCustomGcode>())
        out << 't' << text->kind << ',' << text->script_type << ',' << std::quoted(entity.custom_gcode_string(*text));
    if (const ExtrusionPropertyLoopRole *loop = entity.get_property<ExtrusionPropertyLoopRole>())
        out << 'l' << loop->perimeter_flags << ',' << loop->perimeter_idx << ',' << loop->reserved;
    if (const ArcPolyline *line = entity.polyline_or_null()) {
        CHECK_FALSE(line->has_arc());
        CHECK_FALSE(line->has_z_offset());
        REQUIRE(line->size() >= 2);
        for (size_t index = 0; index < line->size(); ++index)
            out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y() << ')';
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_thin_flow_tree(*child, out, ids);
    }
    out << '}';
}

std::string thin_source_snapshot(const ExtrusionEntity &source)
{
    return thin_flow_snapshot(&source, {}, {});
}

std::string thin_flow_snapshot(const ExtrusionEntity *source, const ExtrusionPaths &paths, const ExtrusionPaths &second)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    std::map<uint64_t, size_t> ids;
    if (source != nullptr)
        write_thin_flow_tree(*source, out, ids);
    out << '[';
    for (const ExtrusionPath &path : paths)
        write_thin_flow_tree(path, out, ids);
    out << "][";
    for (const ExtrusionPath &path : second)
        write_thin_flow_tree(path, out, ids);
    out << ']';
    return out.str();
}

void record_thin_flow(const std::string &name, const ExtrusionEntity &source,
                      const ExtrusionPaths &paths, const ExtrusionPaths &second)
{
    const std::string text = thin_flow_snapshot(&source, paths, second);
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(text.data(), text.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream hash;
    for (const unsigned int word : words)
        hash << std::hex << std::setfill('0') << std::setw(8) << word;
    INFO(name);
    const Slic3r::Test::NativeThinFlowGolden *golden = nullptr;
    for (const Slic3r::Test::NativeThinFlowGolden &candidate : Slic3r::Test::native_thin_flow_goldens)
        if (name == candidate.name) {
            REQUIRE(golden == nullptr);
            golden = &candidate;
        }
    REQUIRE(golden != nullptr);
    CHECK(hash.str() == golden->digest);
}

void require_thin_flow(const ExtrusionEntity &entity, double volume, float width, bool no_seam)
{
    if (entity.has_polyline()) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        CHECK(attributes->mm3_per_mm == volume);
        CHECK(attributes->width == width);
        CHECK(attributes->height == 0.2f);
        CHECK(bool(attributes->no_seam) == no_seam);
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        require_thin_flow(*child, volume, width, no_seam);
}

void require_thin_resources(const ExtrusionPaths &paths)
{
    for (const ExtrusionPath &path : paths) {
        if (path.role() == ExtrusionRole::Travel)
            continue;
        const ExtrusionPropertyCustomGcode *text = path.get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(text != nullptr);
        CHECK(path.custom_gcode_string(*text) == thin_text);
        const ExtrusionPropertySpeed *speed = path.get_property<ExtrusionPropertySpeed>();
        REQUIRE(speed != nullptr);
        CHECK(speed->speed_mm_per_s == 17.f);
    }
}

} // namespace

TEST_CASE("Native thin wall flow mutates cumulatively and copies owned scratch stages", "[plugins][native-thin-flow]")
{
    const coordf_t threshold_sqr = 1000. * 1000.;
    for (const bool generic : {false, true})
        for (size_t form = 0; form < 3; ++form)
        for (size_t connection = 0; connection < 5; ++connection)
            for (const bool no_seam : {false, true}) {
                CAPTURE(generic, form, connection, no_seam);
                ExtrusionEntityUPtr source = thin_flow_source(form);
                if (generic)
                    source = source->clone_canonical();
                const uint64_t source_id = source->get_id();
                const ExtrusionEntity *first_leaf = source->has_polyline() ? source.get() : &source->child(0);
                const ArcPolyline *first_line = first_leaf->polyline_or_null();
                const ExtrusionAttributes *first_attributes = first_leaf->get_property<ExtrusionAttributes>();
                const coord_t offsets[] = {0, 0, -999, -1000, -1001};
                const Point origin(offsets[connection], 0);
                const Point *first = connection == 0 ? nullptr : &origin;
                ExtrusionPaths paths;
                PerimeterGenerator::detail::append_thin_wall_flow(*source, paths, first, 0.1f, no_seam, threshold_sqr);
                CHECK(first == nullptr);
                CHECK(source->get_id() == source_id);
                CHECK((source->has_polyline() ? source.get() : &source->child(0)) == first_leaf);
                CHECK(first_leaf->polyline_or_null() == first_line);
                CHECK(first_leaf->get_property<ExtrusionAttributes>() == first_attributes);
                require_thin_flow(*source, 0.12 * 0.1f, float(0.45f * 0.1f), no_seam);
                REQUIRE(paths.size() == (form == 0 ? 1 : 2) + (connection >= 3 ? 1 : 0));
                CHECK(paths.front().role() == (connection >= 3 ? ExtrusionRole::Travel : ExtrusionRole::ThinWall));
                CHECK(source->ExtrusionEntity::first_point() == (connection == 2 ? origin : Point(0, 0)));
                const std::string name = std::to_string(form) + '-' + std::to_string(connection) + '-' + std::to_string(no_seam);
                record_thin_flow(name + "-first", *source, paths);

                const std::string source_before_return = thin_source_snapshot(*source);
                const size_t before_count = paths.size();
                const coord_t returns[] = {0, 999, 1000, 1001, -999};
                const Point return_to = paths.back().last_point() + Point(returns[connection], 0);
                PerimeterGenerator::detail::ensure_thin_wall_return(paths, return_to, no_seam, threshold_sqr);
                CHECK(thin_source_snapshot(*source) == source_before_return);
                REQUIRE(paths.size() == before_count + (connection == 2 || connection == 3 ? 1 : 0));
                CHECK(paths.back().last_point() == return_to);
                record_thin_flow(name + "-return", *source, paths);

                const std::string first_copy = thin_flow_snapshot(nullptr, paths, {});
                ExtrusionPaths second;
                first = &origin;
                PerimeterGenerator::detail::append_thin_wall_flow(*source, second, first, 9.f, no_seam, threshold_sqr);
                CHECK(first == nullptr);
                double expected_volume = 0.12;
                expected_volume *= 0.1f;
                expected_volume *= 9.f;
                float expected_width = 0.45f;
                expected_width *= 0.1f;
                expected_width *= 9.f;
                require_thin_flow(*source, expected_volume, expected_width, no_seam);
                CHECK((source->has_polyline() ? source.get() : &source->child(0)) == first_leaf);
                CHECK(first_leaf->polyline_or_null() == first_line);
                CHECK(first_leaf->get_property<ExtrusionAttributes>() == first_attributes);
                CHECK(thin_flow_snapshot(nullptr, paths, {}) == first_copy);
                record_thin_flow(name + "-second", *source, paths, second);
                source.reset();
                CHECK(thin_flow_snapshot(nullptr, paths, {}) == first_copy);
                require_thin_resources(paths);
                require_thin_resources(second);
            }
}

TEST_CASE("Native thin wall full flow retains the closed pass and return policy", "[plugins][native-thin-flow]")
{
    for (const bool generic : {false, true})
        for (size_t form = 0; form < 3; ++form) {
        CAPTURE(generic, form);
        ExtrusionEntityUPtr source = thin_flow_source(form);
        if (generic)
            source = source->clone_canonical();
        const Point origin(0, 0);
        const Point *first = &origin;
        ExtrusionPaths paths;
        PerimeterGenerator::detail::append_thin_wall_flow(*source, paths, first, 1.f, false, 1000000.);
        require_thin_flow(*source, 0.12, 0.45f, false);
        PerimeterGenerator::detail::ensure_thin_wall_return(paths, origin, false, 1000000.);
        CHECK(paths.back().last_point() == origin);
        record_thin_flow("unity-" + std::to_string(form), *source, paths);
    }
}
