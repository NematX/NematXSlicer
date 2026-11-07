
#include <boost/uuid/detail/sha1.hpp>
#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iterator>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "test_native_extrusion_publication_goldens.hpp"

/* Characterize native and base-qualified role projections on hand-built trees.
   Scratch roots retain fixed property and geometry payloads, while snapshot()
   warms and checks the native child-pointer view. A single alpha map replaces
   allocation-dependent IDs before a deterministic trace is compared with its
   named immutable digest. These are not Clipper island-order goldens or evidence
   for callback routing and real generator outputs.
   The complete-node swap probe separately checks transferred resource buffers,
   their next IDs, live child handles and warmed pointer views. It is distinct
   from content-only publication swaps, which retain the outer identities.

       TEST_CASE
           |-- make_leaf() -> mark_scope() [seed exact owned metadata]
           |-- snapshot()
           |   `-- write_tree() [geometry, permissions, properties and alpha IDs]
           `-- record() [require the named reference and snapshot count]
*/

namespace {

using namespace Slic3r;

struct Profile { bool sort; bool reverse; bool rotate; };
const std::array<Profile, 3> profiles{{{true, true, true}, {false, false, true}, {false, true, false}}};
enum class Operation { Clear, Append, Swap, Replace };
struct Scenario { const char *name; Operation operation; bool destination_full; bool source_full; };
const std::array<Scenario, 12> scenarios{{
    {"clear-empty", Operation::Clear, false, false}, {"clear-full", Operation::Clear, true, false},
    {"append-00", Operation::Append, false, false}, {"append-01", Operation::Append, false, true},
    {"append-10", Operation::Append, true, false}, {"append-11", Operation::Append, true, true},
    {"swap-00", Operation::Swap, false, false}, {"swap-01", Operation::Swap, false, true},
    {"swap-10", Operation::Swap, true, false}, {"swap-11", Operation::Swap, true, true},
    {"clear-append-empty", Operation::Replace, true, false},
    {"clear-append-full", Operation::Replace, true, true}
}};
struct Trace {
    std::map<uint64_t, size_t> ids;
    std::ostringstream out;
    size_t snapshots = 0;
};

// Mark one owning scope with exact attributes, process values and long owned text.
void mark_scope(ExtrusionEntity &entity, size_t marker);
// Build a leaf with local geometry; roots are never changed through the legacy set_polyline fallback.
ExtrusionEntityUPtr make_leaf(size_t marker, Point start, Point end);
// Build a distinct native Children root, optionally with a two-leaf ordered scope.
std::unique_ptr<ExtrusionEntityCollection> make_root(size_t marker, bool full, const Profile &profile, bool continuous);
// Serialize all seeded direct properties, content and identities with one alpha map for the entire trace.
void write_tree(const ExtrusionEntity &entity, Trace &trace);
// Reacquire the native pointer view and record a complete root state and two separate role projections.
void snapshot(const char *label, const ExtrusionEntityCollection &root, Trace &trace);
// Return the current direct child addresses only while the children remain owned and alive.
ExtrusionEntitiesPtr child_addresses(const ExtrusionEntity &root);
// Read one scope's owned text, requiring the property before dereferencing it.
std::string scope_text(const ExtrusionEntity &entity);
// Expose effective permissions on a fresh disconnected pair without replacing root properties or flags.
void repopulate(ExtrusionEntityCollection &root);
// Check the complete trace against its named immutable snapshot count and SHA-1; a missing reference fails.
void record(const std::string &name, const Trace &trace, size_t expected_snapshots);

} // namespace

TEST_CASE("Publication swaps transfer owned property buffers and future resource IDs",
          "[plugins][extrusion][native-publication]")
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    ExtrusionEntityCollection destination;
    std::unique_ptr<ExtrusionEntityCollection> source = std::make_unique<ExtrusionEntityCollection>();
    destination.append_child(make_leaf(1, Point(0, 0), Point(1000, 0)));
    source->append_child(make_leaf(2, Point(2000, 0), Point(3000, 0)));
    const ExtrusionEntity *destination_child = destination.entities().front();
    const ExtrusionEntity *source_child = source->entities().front();
    const uint64_t destination_node_id = destination.get_id();
    const uint64_t source_node_id = source->get_id();
    destination.set_can_sort_reverse(false, false);
    destination.set_can_rotate(false);
    const std::string destination_text(180, 'd');
    const std::string source_text(220, 's');
    destination.add_property(ExtrusionPropertyCustomGcodeText("replaced"));
    const extrusion_data_id destination_id = destination.add_property(ExtrusionPropertyCustomGcodeText(destination_text)).text_id;
    const extrusion_data_id source_id = source->add_property(ExtrusionPropertyCustomGcodeText(source_text)).text_id;
    REQUIRE(destination_id != source_id);
    uint32_t destination_size = 0;
    uint32_t source_size = 0;
    const void *destination_buffer = Access::stored_data(destination, destination_id, &destination_size);
    const void *source_buffer = Access::stored_data(*source, source_id, &source_size);
    REQUIRE(destination_buffer != nullptr);
    REQUIRE(source_buffer != nullptr);
    CHECK(destination_size == destination_text.size() + 1);
    CHECK(source_size == source_text.size() + 1);

    destination.swap(*source);

    CHECK(destination.get_id() == source_node_id);
    CHECK(source->get_id() == destination_node_id);
    CHECK(destination.can_sort());
    CHECK(destination.can_reverse());
    CHECK(destination.can_rotate());
    CHECK_FALSE(source->can_sort());
    CHECK_FALSE(source->can_reverse());
    CHECK_FALSE(source->can_rotate());
    REQUIRE(destination.entities().size() == 1);
    REQUIRE(source->entities().size() == 1);
    CHECK(destination.entities().front() == source_child);
    CHECK(source->entities().front() == destination_child);

    const ExtrusionPropertyCustomGcode *destination_property = destination.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyCustomGcode *source_property = source->get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(destination_property != nullptr);
    REQUIRE(source_property != nullptr);
    CHECK(destination_property->text_id == source_id);
    CHECK(source_property->text_id == destination_id);
    CHECK(destination.custom_gcode_string(*destination_property) == source_text);
    CHECK(source->custom_gcode_string(*source_property) == destination_text);
    CHECK(Access::stored_data(destination, source_id, &source_size) == source_buffer);
    CHECK(Access::stored_data(*source, destination_id, &destination_size) == destination_buffer);
    CHECK(source_size == source_text.size() + 1);
    CHECK(destination_size == destination_text.size() + 1);

    const std::string next_source_text(240, 'n');
    CHECK(source->add_property(ExtrusionPropertyCustomGcodeText(next_source_text)).text_id == destination_id + 1);
    CHECK(scope_text(*source) == next_source_text);
    source.reset();
    CHECK(scope_text(destination) == source_text);
    const std::string next_destination_text(260, 'm');
    CHECK(destination.add_property(ExtrusionPropertyCustomGcodeText(next_destination_text)).text_id == source_id + 1);
    CHECK(scope_text(destination) == next_destination_text);
}

TEST_CASE("Native publication role queries retain separate Collection and base projections",
          "[plugins][extrusion][native-publication]")
{
    const std::array<const char *, 9> names{{"role-empty", "role-empty-attributed", "role-homogeneous",
        "role-mixed", "role-root-opposite", "role-root-concordant", "role-none-first", "role-none-last",
        "role-attributed-nesting"}};
    for (size_t index = 0; index < names.size(); ++index) {
        CAPTURE(names[index]);
        ExtrusionEntityCollection root;
        const ExtrusionAttributes sparse{ExtrusionRole::InternalInfill};
        const ExtrusionAttributes solid{ExtrusionRole::SolidInfill};
        if (index == 1 || index == 4)
            root.add_property(solid);
        if (index == 5)
            root.add_property(sparse);
        if (index >= 2 && index <= 5) {
            std::unique_ptr<ExtrusionEntityCollection> group = std::make_unique<ExtrusionEntityCollection>(false, false);
            group->append_child(make_leaf(400, Point(0, 0), Point(1000, 0)));
            group->child(0).add_property(sparse);
            if (index == 2 || index == 3) {
                group->append_child(make_leaf(401, Point(1000, 0), Point(2000, 0)));
                group->child(1).add_property(index == 2 ? sparse : solid);
            }
            root.append_child(std::move(group));
        } else if (index == 6 || index == 7) {
            for (size_t child = 0; child < 2; ++child) {
                ExtrusionEntityUPtr leaf = make_leaf(410 + child, Point(coord_t(child * 1000), 0),
                    Point(coord_t((child + 1) * 1000), 0));
                leaf->add_property((child == 0) == (index == 6) ? ExtrusionAttributes{ExtrusionRole::None} : sparse);
                root.append_child(std::move(leaf));
            }
        } else if (index == 8) {
            std::unique_ptr<ExtrusionEntityCollection> native = std::make_unique<ExtrusionEntityCollection>();
            native->add_property(solid);
            native->append_child(make_leaf(420, Point(0, 0), Point(1000, 0)));
            native->child(0).add_property(sparse);
            root.append_child(std::move(native));
            ExtrusionEntityUPtr generic = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, true, true, false);
            generic->add_property(solid);
            generic->append_child(make_leaf(421, Point(1000, 0), Point(2000, 0)));
            generic->child(0).add_property(sparse);
            root.append_child(std::move(generic));
        }
        Trace trace;
        trace.out.imbue(std::locale::classic());
        trace.out << std::hexfloat;
        snapshot("role-projections", root, trace);
        record(names[index], trace, 1);
    }
}

namespace {

void mark_scope(ExtrusionEntity &entity, size_t marker)
{
    entity.add_property(ExtrusionAttributes{ExtrusionRole::Perimeter,
        ExtrusionFlow(0.0625 * (marker % 8 + 1), 0.5f, 0.25f)});
    entity.add_property(ExtrusionPropertySpeed(float(marker), float(marker * 2), 0.125f));
    entity.add_property(ExtrusionPropertyCustomGcodeText(std::string(180, char('a' + marker % 20)) + '-' + std::to_string(marker)));
}

ExtrusionEntityUPtr make_leaf(size_t marker, Point start, Point end)
{
    ExtrusionEntityUPtr leaf = std::make_unique<ExtrusionEntity>(false, ArcPolyline(Points{start, end}));
    mark_scope(*leaf, marker);
    return leaf;
}

std::unique_ptr<ExtrusionEntityCollection> make_root(size_t marker, bool full, const Profile &profile, bool continuous)
{
    std::unique_ptr<ExtrusionEntityCollection> root = std::make_unique<ExtrusionEntityCollection>(profile.sort, profile.reverse);
    root->set_can_rotate(profile.rotate);
    mark_scope(*root, marker);
    if (full) {
        ExtrusionEntityUPtr group = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, true, false);
        group->set_can_rotate(false);
        mark_scope(*group, marker + 1);
        group->append_child(make_leaf(marker + 2, Point(0, 0), Point(1000, 0)));
        group->append_child(make_leaf(marker + 3, Point(continuous ? 1000 : 2000, 0), Point(3000, 0)));
        root->append_child(std::move(group));
    }
    return root;
}

void write_tree(const ExtrusionEntity &entity, Trace &trace)
{
    using Access = ApiInternal::ExtrusionPropertyAccess;
    const size_t next = trace.ids.size();
    trace.out << '{' << trace.ids.emplace(entity.get_id(), next).first->second << ':'
              << entity.is_leaf() << ',' << entity.is_nop() << ',' << entity.has_polyline() << ','
              << entity.child_count() << ',' << entity.can_sort() << ',' << entity.can_reverse() << ','
              << entity.can_rotate() << ',' << entity.is_continuous() << ';';
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>();
    const ExtrusionPropertyCustomGcode *text = entity.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(Access::property_count(entity) == size_t(attributes != nullptr) + size_t(speed != nullptr) + size_t(text != nullptr));
    for (size_t index = 0; index < Access::property_count(entity); ++index)
        trace.out << 'p' << Access::property_type_at(entity, index) << ';';
    if (attributes != nullptr)
        trace.out << 'a' << int(attributes->role) << ',' << attributes->mm3_per_mm << ',' << attributes->width
                  << ',' << attributes->height << ',' << int(attributes->no_seam) << ';';
    if (speed != nullptr)
        trace.out << 's' << speed->speed_mm_per_s << ',' << speed->accel_mm_per_s2 << ',' << speed->pressure_adv << ';';
    if (text != nullptr) {
        trace.out << 't' << text->kind << ',' << text->script_type << ',' << text->text_id << ',' << text->config_id
                  << ',' << int(text->processing_extruder_id) << ',' << std::quoted(entity.custom_gcode_string(*text)) << ';';
        uint32_t size = 0;
        const void *buffer = Access::stored_data(entity, text->text_id, &size);
        REQUIRE(buffer != nullptr);
        CHECK(size == entity.custom_gcode_string(*text).size() + 1);
        trace.out << 'r' << size << ';';
    }
    if (const ArcPolyline *line = entity.polyline_or_null()) {
        CHECK_FALSE(line->has_arc());
        CHECK_FALSE(line->has_z_offset());
        trace.out << 'g' << line->size();
        for (size_t index = 0; index < line->size(); ++index)
            trace.out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y() << ')';
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_tree(*child, trace);
    }
    trace.out << '}';
}

void snapshot(const char *label, const ExtrusionEntityCollection &root, Trace &trace)
{
    ++trace.snapshots;
    trace.out << std::quoted(label) << ':';
    const ExtrusionEntitiesPtr &view = root.entities();
    REQUIRE(view.size() == root.child_count());
    for (size_t index = 0; index < view.size(); ++index)
        CHECK(view[index] == &root.child(index));
    write_tree(root, trace);
    // Only the root call is base-qualified; descendant role calls remain virtual in both projections.
    trace.out << "native-role:" << int(root.role()()) << ":base-role:" << int(root.ExtrusionEntity::role()()) << ':';
    const std::array<ExtrusionRole, 5> queries{{ExtrusionRole::InternalInfill, ExtrusionRole::SolidInfill,
        ExtrusionRole::SupportMaterial, ExtrusionRole::None,
        ExtrusionRole{ExtrusionRole::InternalInfill | ExtrusionRoleModifier::ERM_Solid | ExtrusionRoleModifier::ERM_External}}};
    for (const ExtrusionRole query : queries)
        trace.out << int(query()) << '=' << root.has_role(query) << ',' << root.ExtrusionEntity::has_role(query) << ';';
}

ExtrusionEntitiesPtr child_addresses(const ExtrusionEntity &root)
{
    ExtrusionEntitiesPtr out;
    for (const ExtrusionEntityUPtr &child : root.children()) {
        REQUIRE(child != nullptr);
        out.push_back(child.get());
    }
    return out;
}

std::string scope_text(const ExtrusionEntity &entity)
{
    const ExtrusionPropertyCustomGcode *text = entity.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(text != nullptr);
    return entity.custom_gcode_string(*text);
}

void repopulate(ExtrusionEntityCollection &root)
{
    root.clear();
    root.append_child(make_leaf(500, Point(0, 0), Point(1000, 0)));
    root.append_child(make_leaf(501, Point(2000, 0), Point(3000, 0)));
}

void record(const std::string &name, const Trace &trace, size_t expected_snapshots)
{
    REQUIRE(trace.snapshots == expected_snapshots);
    const std::string state = trace.out.str();
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(state.data(), state.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream hash;
    for (const unsigned int word : words)
        hash << std::hex << std::setfill('0') << std::setw(8) << word;
    CAPTURE(name);
    const Slic3r::Test::NativeExtrusionPublicationGolden *golden = std::find_if(
        std::begin(Slic3r::Test::native_extrusion_publication_goldens),
        std::end(Slic3r::Test::native_extrusion_publication_goldens),
        [&name](const Slic3r::Test::NativeExtrusionPublicationGolden &entry) { return name == entry.name; });
    REQUIRE(golden != std::end(Slic3r::Test::native_extrusion_publication_goldens));
    CHECK(trace.snapshots == golden->snapshots);
    CHECK(hash.str() == golden->digest);
}

} // namespace
