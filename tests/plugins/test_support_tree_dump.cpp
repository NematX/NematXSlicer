#include "support_grid_test_helpers.hpp"
#include "support_tree_test_helpers.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSupportGenerator.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeVolumes.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <tbb/global_control.h>
#include <tbb/task_arena.h>

/* Baseline dump of the tree family, recorded before it moved to the plugin API. The test records, for a few objects, what the
   family computes: the TreeSettings fields, the TreeVolumes areas of a fixed list of keys, the nodes that the routing returns, the
   same nodes after the slab generation (the organic smoothing moves them), and the slabs that the generator publishes. The
   serialization is canonical (see support_tree_test_helpers.hpp): nothing in it depends on addresses, ring start points or ring
   order, and the numbers are written with the "C" locale.

   Every case is made inside a TBB arena of one thread under a global limit of one thread, slicing included, so that the routing
   buckets, which read the concurrency of the arena, give the same result on every machine. Each dump is made twice from scratch and
   the two must be equal before anything is written.

   The dump test runs only when an environment variable is set, as it is slow:
   - SLIC3R_TREE_DUMP=<file>      write the dump to the file.
   - SLIC3R_TREE_DUMP_REF=<file>  compare the dump with the reference file; differences below ring_tolerance_mm2 with the same ring
                                  count are listed as drifts, every other difference fails the test.
   Both may be set together, but not with the same file: the reference is read and compared before anything is written, and it is
   never overwritten. The lists of drifts and changes are written to the standard error stream.

       $env:SLIC3R_TREE_DUMP_REF="<root>\build\temp\tree-baseline\baseline.txt"
       plugins_tests "[support-tree-dump]"

   The format cases ([dump-format]) always run: they check the parser, the key check and the canonical form on small inputs.

   TreeVolumes keeps its holefree collisions private, so they are not recorded directly: the FastSafe avoidance, which is built on
   them, covers them (see dump_volumes). */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace ST = Slic3r::Test::SupportTree;
namespace TF = Slic3r::TreeFamily;
namespace Common = slic3r_api::Support::Common;

namespace {
using ST::Dump;
using ST::Record;

// One object to dump: the comparison model (see ST::comparison_mesh), the style, the settings to override, and whether the routing
// and the slabs are recorded. The volumes-only cases are the scenes of the tree volumes tests.
struct DumpCase {
    const char *name;
    int model_id;
    const char *style;
    int interfaces;
    std::vector<std::pair<std::string, std::string>> overrides;
    bool routing;
};

// The cases of the baseline, in recording order.
const std::vector<DumpCase> &dump_cases();
// Builds a fresh print for every case and records it in one thread (slicing included); the dump of all the cases.
Dump make_dump();
// Records one case on a sliced print: settings, volumes, then routing and slabs when the case asks for them.
void dump_one(Dump &dump, const DumpCase &current, const Print &print);
// Records every field of TreeSettings and the maximum moves of the layers as text.
void dump_settings(Dump &dump, const std::string &prefix, const TF::TreeSettings &settings);
// Records the TreeVolumes areas of a fixed list of keys as canonical rings.
void dump_volumes(Dump &dump, const std::string &prefix, const Print &print, const PrintObject &object, const TF::TreeSettings &settings);
// Records the node count, the tip count and every node of each routed tree; base is the key of the record.
void dump_routing(Dump &dump, const std::string &base, const std::vector<TF::RoutedTree> &trees);
// Records the slabs of each role, in a total order, with the contact and overhang areas of the top contacts.
void dump_slabs(Dump &dump, const std::string &prefix, const Common::GeneratedSupport &support);
// Text lines "name=value" of a record; the overloads format each type without any locale dependence.
std::string field(const char *name, double value);
std::string field(const char *name, long long value);
std::string field(const char *name, size_t value);
std::string field(const char *name, int value);
std::string field(const char *name, bool value);
// The content of a file; fails the test when the file cannot be opened.
std::string read_file(const std::string &path);
// Writes a titled list of lines to the standard error stream.
void report(const char *title, const std::vector<std::string> &lines);
// Parses the text of a dump that must be valid; the format cases use it.
Dump parse_valid(const std::string &text);
// A square ring from (x, y) of the given side, counterclockwise, in canonical form.
ST::Ring square(long long x, long long y, long long side);

const std::vector<DumpCase> &dump_cases()
{
    static const std::vector<DumpCase> cases{
        {"slab-tree", 0, "tree", 3, {}, true},
        {"tbridge-tree", 1, "tree", 3, {}, true},
        {"shelf-tree", 2, "tree", 3, {}, true},
        {"shelf-organic", 2, "organic", 3, {}, true},
        {"volumes-scene", 3, "tree", 3, {{"support_material_xy_spacing", "3"}}, false},
        {"volumes-scene-buildplate", 3, "tree", 3, {{"support_material_xy_spacing", "3"}, {"support_material_buildplate_only", "1"}}, false},
    };
    return cases;
}

Dump make_dump()
{
    Dump dump;
    for (const DumpCase &current : dump_cases()) {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        DynamicPrintConfig config = SG::config(current.style, false, current.interfaces, false);
        for (const std::pair<std::string, std::string> &override : current.overrides)
            config.set_deserialize_strict({{override.first, override.second}});
        // One thread, whatever the machine: the arena and the global limit both. The print is built, sliced, dumped and destroyed
        // inside, so that no step of the case depends on the number of cores.
        tbb::global_control limit(tbb::global_control::max_allowed_parallelism, 1);
        tbb::task_arena arena(1);
        arena.execute([&] {
            Print print;
            Model model;
            Test::init_print({ST::comparison_mesh(current.model_id)}, print, model, config);
            SG::prefix(print);
            dump_one(dump, current, print);
        });
    }
    return dump;
}

void dump_one(Dump &dump, const DumpCase &current, const Print &print)
{
    const std::string prefix = current.name;
    const PrintObject &object = print.object(0);
    const Common::SupportParams params = SG::params(print, object);
    const TF::TreeSettings settings = Slic3r::Test::SupportTree::tree_settings(print, object, params);
    dump_settings(dump, prefix, settings);
    dump_volumes(dump, prefix, print, object, settings);
    if (!current.routing)
        return;
    // The trees are routed once, recorded, then the slabs are generated from these same trees: the organic smoothing moves their
    // nodes in place, so the second record is the state the planner and the drawing work on.
    ST::Routed routed(print);
    dump_routing(dump, prefix + "/routing", routed.trees);
    dump_slabs(dump, prefix + "/routed_contacts", routed.support);
    const bool organic = std::string(current.style) == "organic";
    TF::generate_tree_slabs(nullptr, print, object, routed.trees, organic, routed.support);
    dump_routing(dump, prefix + "/routing_smoothed", routed.trees);
    dump_slabs(dump, prefix, routed.support);
}

void dump_settings(Dump &dump, const std::string &prefix, const TF::TreeSettings &s)
{
    Record &record = dump.add(prefix + "/settings");
    record.lines = {
        field("layer_height", (long long) s.layer_height), field("resolution", (long long) s.resolution),
        field("min_feature_size", (long long) s.min_feature_size), field("support_line_width", (long long) s.support_line_width),
        field("branch_radius", (long long) s.branch_radius), field("min_radius", (long long) s.min_radius),
        field("tip_layers", s.tip_layers), field("branch_radius_increase_per_layer", s.branch_radius_increase_per_layer),
        field("bp_radius_increase_per_layer", s.bp_radius_increase_per_layer), field("bp_radius", (long long) s.bp_radius),
        field("layer_start_bp_radius", s.layer_start_bp_radius), field("increase_radius_until_radius", (long long) s.increase_radius_until_radius),
        field("increase_radius_until_layer", s.increase_radius_until_layer), field("min_dtt_to_model", s.min_dtt_to_model),
        field("max_to_model_radius_increase", (long long) s.max_to_model_radius_increase),
        field("support_rests_on_model", s.support_rests_on_model), field("xy_distance", (long long) s.xy_distance),
        field("xy_min_distance", (long long) s.xy_min_distance), field("z_distance_top_layers", s.z_distance_top_layers),
        field("z_distance_bottom_layers", s.z_distance_bottom_layers), field("branch_distance", (long long) s.branch_distance),
        field("top_rate", s.top_rate), field("angle", s.angle), field("max_overhang", (long long) s.max_overhang),
        field("min_slab_height", (long long) s.min_slab_height), field("max_slab_height", (long long) s.max_slab_height),
        field("dense_slab_height", (long long) s.dense_slab_height), field("z_step", (long long) s.z_step)};
    std::string moves, slow;
    for (const coord_t move : s.max_move)
        moves += std::to_string((long long) move) + " ";
    for (const coord_t move : s.max_move_slow)
        slow += std::to_string((long long) move) + " ";
    record.lines.push_back("max_move=" + moves);
    record.lines.push_back("max_move_slow=" + slow);
}

// Radii: the tip, the branch and the widened foot. Every layer of the object is asked, after a precalculation up to the top layer,
// so that the lazy path of the getters is part of what is recorded as well.
// The holefree collisions (collisions that also keep out the holes too small for a branch) have no public getter
// (TreeVolumes::getCollisionHolefree is private), so they are not recorded directly. The FastSafe avoidances are propagated from
// them, so they are covered through those records: a change of a holefree collision shows in the FastSafe avoidance.
void dump_volumes(Dump &dump, const std::string &prefix, const Print &print, const PrintObject &object, const TF::TreeSettings &settings)
{
    using Volumes = TF::TreeVolumes;
    const TF::LayerIndex layers = TF::LayerIndex(object.layer_count());
    // The storage owns the areas of the volumes, so it is declared first and destroyed last.
    PluginStorage storage;
    Volumes volumes(SG::object_view(object), SG::print_view(print), settings, reinterpret_cast<storage_handle *>(&storage), [] {});
    volumes.precalculate(layers - 1, [] {});
    const std::set<coord_t> radii{settings.getRadius(0), settings.branch_radius, settings.bp_radius};
    const std::vector<std::pair<Volumes::AvoidanceType, const char *>> types{
        {Volumes::AvoidanceType::Slow, "slow"}, {Volumes::AvoidanceType::FastSafe, "fastsafe"}, {Volumes::AvoidanceType::Fast, "fast"}};
    auto add = [&](const std::string &key, const slic3r_api::StoredPolygonCollection &polygons) {
        dump.add(prefix + "/volumes/" + key).rings = ST::rings_of_view(polygons);
    };
    for (const coord_t radius : radii) {
        const std::string r = "r" + std::to_string((long long) radius);
        for (const bool min_xy : {true, false}) {
            const std::string xy = min_xy ? "/minxy" : "/regxy";
            for (TF::LayerIndex layer = 0; layer < layers; ++layer)
                add("collision/" + r + xy + "/L" + std::to_string(layer), volumes.getCollision(radius, layer, min_xy));
            for (TF::LayerIndex layer = 1; layer < layers; ++layer) {
                const std::string at = "/L" + std::to_string(layer);
                for (const std::pair<Volumes::AvoidanceType, const char *> &type : types)
                    for (const bool to_model : {false, true})
                        add(std::string("avoidance/") + type.second + (to_model ? "/model/" : "/plate/") + r + xy + at,
                            volumes.getAvoidance(radius, layer, type.first, to_model, min_xy));
                add("wall/" + r + xy + at, volumes.getWallRestriction(radius, layer, min_xy));
            }
        }
    }
    // The placeable areas have no XY distance variant: radius 0 (the top surfaces of the object) and the tip radius.
    for (const coord_t radius : {coord_t(0), settings.getRadius(0)})
        for (TF::LayerIndex layer = 1; layer < layers; ++layer)
            add("placeable/r" + std::to_string((long long) radius) + "/L" + std::to_string(layer), volumes.getPlaceableAreas(radius, layer, [] {}));
}

void dump_routing(Dump &dump, const std::string &base, const std::vector<TF::RoutedTree> &trees)
{
    size_t tips = 0, nodes = 0;
    for (const TF::RoutedTree &tree : trees)
        for (const TF::TreeNode &node : tree.nodes) {
            ++nodes;
            tips += node.carries.empty() ? 1 : 0;
        }
    dump.add(base).lines = {field("trees", trees.size()), field("nodes", nodes), field("tips", tips)};
    for (size_t t = 0; t < trees.size(); ++t) {
        Record &record = dump.add(base + "/tree" + std::to_string(t));
        record.lines.push_back(field("nodes", trees[t].nodes.size()));
        for (size_t n = 0; n < trees[t].nodes.size(); ++n) {
            const TF::TreeNode &node = trees[t].nodes[n];
            std::string carries;
            for (const size_t carried : node.carries)
                carries += std::to_string(carried) + " ";
            record.lines.push_back("node " + std::to_string(n) + " layer=" + std::to_string(node.layer) + " print_z=" + ST::number(node.print_z) +
                " x=" + std::to_string((long long) node.position.x()) + " y=" + std::to_string((long long) node.position.y()) +
                " radius=" + std::to_string((long long) node.radius) + " to_buildplate=" + (node.to_buildplate ? "1" : "0") +
                " to_model_gracious=" + (node.to_model_gracious ? "1" : "0") + " lost=" + (node.lost ? "1" : "0") +
                " verylost=" + (node.verylost ? "1" : "0") + " carries=" + carries);
        }
    }
}

// The order of generation is not recorded: the slabs of a role are sorted by (print Z, bottom Z, decreasing height, type, bridging,
// then the text of the whole slab: lines, rings, contact and overhang rings). The last key makes the order total, so two slabs
// never swap between two runs.
void dump_slabs(Dump &dump, const std::string &prefix, const Common::GeneratedSupport &support)
{
    const std::vector<std::pair<const char *, const Common::GeneratorLayers *>> roles{
        {"top", &support.top}, {"bottom", &support.bottom}, {"base", &support.base}, {"interfaces", &support.interfaces}};
    for (const std::pair<const char *, const Common::GeneratorLayers *> &role : roles) {
        struct Slab {
            std::tuple<long long, long long, long long, int, bool, std::string> order;
            Record record, contact, overhang;
            bool has_contact = false, has_overhang = false;
        };
        std::vector<Slab> slabs;
        for (const Common::GeneratorLayer *layer : *role.second) {
            Slab slab;
            slab.record.rings = ST::rings_of_view(layer->polygons);
            slab.record.lines = {field("type", int(layer->type)), field("bottom_z", (long long) layer->bottom_z),
                field("print_z", (long long) layer->print_z), field("height", (long long) layer->height),
                field("height_block", (long long) layer->height_block), field("bridging", layer->bridging),
                field("object_above", (long long) layer->object_above), field("object_below", (long long) layer->object_below)};
            // The text of the slab as it will be written, for the total order below.
            ST::Dump text;
            text.records.push_back(slab.record);
            if (layer->contact_polygons) {
                slab.has_contact = true;
                slab.contact.key = "contact";
                slab.contact.rings = ST::rings_of_view(*layer->contact_polygons);
                text.records.push_back(slab.contact);
            }
            if (layer->overhang_polygons) {
                slab.has_overhang = true;
                slab.overhang.key = "overhang";
                slab.overhang.rings = ST::rings_of_view(*layer->overhang_polygons);
                text.records.push_back(slab.overhang);
            }
            slab.order = {(long long) layer->print_z, (long long) layer->bottom_z, -(long long) layer->height, int(layer->type), layer->bridging,
                          ST::serialize(text)};
            slabs.push_back(std::move(slab));
        }
        std::stable_sort(slabs.begin(), slabs.end(), [](const Slab &a, const Slab &b) { return a.order < b.order; });
        dump.add(prefix + "/slabs/" + role.first).lines = {field("count", slabs.size())};
        for (size_t i = 0; i < slabs.size(); ++i) {
            const std::string key = prefix + "/slabs/" + role.first + "/" + std::to_string(i);
            Record &record = dump.add(key);
            record = slabs[i].record;
            record.key = key;
            if (slabs[i].has_contact)
                dump.add(key + "/contact").rings = slabs[i].contact.rings;
            if (slabs[i].has_overhang)
                dump.add(key + "/overhang").rings = slabs[i].overhang.rings;
        }
    }
}

std::string field(const char *name, double value) { return std::string(name) + "=" + ST::number(value); }
std::string field(const char *name, long long value) { return std::string(name) + "=" + std::to_string(value); }
std::string field(const char *name, size_t value) { return std::string(name) + "=" + std::to_string(value); }
std::string field(const char *name, int value) { return std::string(name) + "=" + std::to_string(value); }
std::string field(const char *name, bool value) { return std::string(name) + "=" + (value ? "1" : "0"); }

std::string read_file(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void report(const char *title, const std::vector<std::string> &lines)
{
    std::cerr << title << ": " << lines.size() << "\n";
    for (const std::string &line : lines)
        std::cerr << "  " << line << "\n";
}

// The errors of the parser go to the test report.
Dump parse_valid(const std::string &text)
{
    std::vector<std::string> errors;
    Dump dump = ST::parse_dump(text, "text", errors);
    CHECK(errors.empty());
    return dump;
}

ST::Ring square(long long x, long long y, long long side)
{
    return ST::canonical_ring({{x, y}, {x + side, y}, {x + side, y + side}, {x, y + side}});
}
}

// Records the baseline of the tree family, checks that it is repeatable, and compares it with a reference when asked. The reference
// is read and checked first, then compared with the current dump; only then is the dump written, and never over the reference.
TEST_CASE("Tree baseline dump", "[plugins][support-tree-dump][slow]")
{
    const char *out_env = std::getenv("SLIC3R_TREE_DUMP");
    const char *ref_env = std::getenv("SLIC3R_TREE_DUMP_REF");
    const std::string out_path = out_env != nullptr ? out_env : "";
    const std::string ref_path = ref_env != nullptr ? ref_env : "";
    if (out_path.empty() && ref_path.empty()) {
        SUCCEED("SLIC3R_TREE_DUMP and SLIC3R_TREE_DUMP_REF are not set: nothing to dump");
        return;
    }
    std::error_code ignored;
    if (!out_path.empty() && !ref_path.empty() && std::filesystem::equivalent(out_path, ref_path, ignored))
        FAIL("SLIC3R_TREE_DUMP and SLIC3R_TREE_DUMP_REF name the same file: the reference would be overwritten");

    // The reference first, so a missing or damaged one fails before the long run.
    Dump reference;
    if (!ref_path.empty()) {
        std::vector<std::string> errors;
        reference = ST::parse_dump(read_file(ref_path), ref_path, errors);
        for (const std::string &error : ST::duplicate_keys(reference, ref_path))
            errors.push_back(error);
        report("reference errors", errors);
        REQUIRE(errors.empty());
    }

    const Dump first = make_dump();
    const Dump second = make_dump();
    const std::vector<std::string> duplicates = ST::duplicate_keys(first, "current dump");
    report("current dump errors", duplicates);
    REQUIRE(duplicates.empty());
    // Repeatable: the second run gives the very same text. Nothing is written before this holds.
    const std::string text = ST::serialize(first);
    const bool repeatable = text == ST::serialize(second);
    if (!repeatable) {
        const ST::Comparison twice = ST::compare_dumps(first, second);
        report("repeat changes", twice.changes);
        report("repeat drifts", twice.drifts);
    }
    REQUIRE(repeatable);

    bool matches = true;
    if (!ref_path.empty()) {
        const ST::Comparison comparison = ST::compare_dumps(reference, first);
        std::cerr << "records identical to the reference: " << comparison.identical << "\n";
        report("drifts (accepted, to explain)", comparison.drifts);
        report("changes", comparison.changes);
        matches = comparison.changes.empty();
    }
    if (!out_path.empty()) {
        std::ofstream out(out_path, std::ios::binary);
        REQUIRE(out.is_open());
        out << text;
        REQUIRE(out.good());
    }
    CHECK(matches);
}

// The text format round-trips, the numbers keep every digit, and CRLF line ends are read as LF ones.
TEST_CASE("Tree dump text format round-trips", "[plugins][support-tree-dump][dump-format]")
{
    Dump dump;
    dump.add("a/one").lines = {field("value", 2.8), field("small", 0.1 + 0.2), field("count", size_t(3))};
    dump.records.back().rings = {square(0, 0, 1000), square(-5, 7, 20)};
    dump.add("a/empty");
    const std::string text = ST::serialize(dump);
    CHECK(ST::number(0.1 + 0.2) == "0.30000000000000004");
    CHECK(ST::serialize(parse_valid(text)) == text);
    std::string crlf;
    for (const char c : text)
        crlf += c == '\n' ? std::string("\r\n") : std::string(1, c);
    CHECK(ST::serialize(parse_valid(crlf)) == text);
}

// A reference with a truncated or malformed line is refused with the line number, never compared partially.
TEST_CASE("Tree dump parser reports malformed lines", "[plugins][support-tree-dump][dump-format]")
{
    const std::vector<std::pair<std::string, std::string>> bad{
        {"@ k\nr 0\n", "2: ring line without"},
        {"@ k\nr 10.0 3 1 2 3 4\n", "2: ring announces 3 points"},
        {"@ k\nr 10.0 1 1 2 3\n", "2: ring has more coordinates"},
        {"@ k\nr 1.0 1 1 x\n", "2: ring announces 1 points"},
        {"@ k\nr 1.0 0\n", "2: ring line without"},
        {"@ k\nx\n", "2: unknown line"},
        {"@ k\n\n", "2: unknown line"},
        {"= text\n", "1: text line before any record"},
        {"r 1.0 1 1 2\n", "1: ring before any record"},
        {"@ \n", "1: empty record key"},
    };
    for (const std::pair<std::string, std::string> &item : bad) {
        CAPTURE(item.first);
        std::vector<std::string> errors;
        ST::parse_dump(item.first, "ref", errors);
        REQUIRE(errors.size() == 1);
        CHECK(errors.front().find("ref:" + item.second) == 0);
    }
}

// A key that appears twice is reported with both record indices, in a parsed reference as in a current dump.
TEST_CASE("Tree dump keys must be unique", "[plugins][support-tree-dump][dump-format]")
{
    Dump dump;
    dump.add("k").lines = {"x"};
    dump.add("other");
    dump.add("k");
    const std::vector<std::string> errors = ST::duplicate_keys(dump, "current");
    REQUIRE(errors.size() == 1);
    CHECK(errors.front() == "current: duplicated key [k] in records 0 and 2");
    std::vector<std::string> parse_errors;
    CHECK(ST::duplicate_keys(ST::parse_dump(ST::serialize(dump), "ref", parse_errors), "ref").size() == 1);
    CHECK(parse_errors.empty());
}

// The canonical form does not depend on the start point or the order of the rings, and the comparison tells a drift from a change.
TEST_CASE("Tree dump canonical rings and comparison", "[plugins][support-tree-dump][dump-format]")
{
    const ST::Ring rotated = ST::canonical_ring({{1000, 1000}, {0, 1000}, {0, 0}, {1000, 0}});
    CHECK(rotated == square(0, 0, 1000));
    // Equal smallest points: the points that follow decide, so the form is unique.
    CHECK(ST::canonical_ring({{0, 0}, {5, 5}, {0, 0}, {5, -5}}) == ST::canonical_ring({{0, 0}, {5, -5}, {0, 0}, {5, 5}}));

    Dump reference;
    reference.add("k").rings = {square(0, 0, scale_i(2.)), square(scale_i(5.), 0, scale_i(1.))};
    ST::sort_rings(reference.records.back().rings);
    Dump same;
    same.add("k").rings = {square(scale_i(5.), 0, scale_i(1.)), square(0, 0, scale_i(2.))};
    // The rings of a record are sorted when they are recorded; sort_rings is the one used by rings_of().
    ST::sort_rings(same.records.back().rings);
    ST::Comparison comparison = ST::compare_dumps(reference, same);
    CHECK(comparison.identical == 1);
    CHECK(comparison.changes.empty());
    CHECK(comparison.drifts.empty());

    Dump drift;
    drift.add("k").rings = {square(1, 0, scale_i(2.)), square(scale_i(5.), 0, scale_i(1.))};
    ST::sort_rings(drift.records.back().rings);
    comparison = ST::compare_dumps(reference, drift);
    CHECK(comparison.drifts.size() == 1);
    CHECK(comparison.changes.empty());

    Dump moved;
    moved.add("k").rings = {square(scale_i(1.), 0, scale_i(2.)), square(scale_i(5.), 0, scale_i(1.))};
    ST::sort_rings(moved.records.back().rings);
    comparison = ST::compare_dumps(reference, moved);
    CHECK(comparison.changes.size() == 1);

    Dump fewer;
    fewer.add("k").rings = {square(0, 0, scale_i(2.))};
    comparison = ST::compare_dumps(reference, fewer);
    CHECK(comparison.changes.size() == 1);
}
