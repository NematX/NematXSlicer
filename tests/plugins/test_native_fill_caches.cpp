#include <catch2/catch.hpp>
#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillCache.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Thread.hpp"
#include "perimeter_test_helpers.hpp"
#include "test_native_fill_cache_goldens.hpp"

/* Check native fill-cache publication and direct Layer fill consumers without
   running an exporter or the historical perimeter/support pipeline. Explicit
   input surfaces and bounded recipes isolate priority/extruder routing, copied
   versus transferred identities, owned property resources and ironing wrappers.
   Printer settings are sized for every configured nozzle before preparation.

       TEST_CASE -> cache_config() -> prepare_cube_print(): own the sliced context
           |-- make_cache() -> FillCacheDetail::store_fill(): publish priority rows
           |-- install_fill_inputs(): supply explicit native surfaces
           |-- Layer fill/anchoring/ironing calls: build owned output
           `-- record_cache(): compare full state and identity/resource observations
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
using CacheMatrix = std::vector<std::vector<std::pair<const Surface *, std::unique_ptr<ExtrusionEntity>>>>;

struct CacheObservation {
    std::uintptr_t address;
    bool present;
    bool copied;
    std::string local;
};

// Prepare bounded native fill settings without adaptive, lightning or perimeter dependencies.
// Size every extruder option for the configured nozzles before preparing a print.
DynamicPrintConfig cache_config(bool distinct_extruders = false);
// Attach distinct direct speed and heap-owned text markers to a node.
void mark_cache_node(ExtrusionEntity &node, size_t marker);
// Build an optional two-leaf cache with distinct permissions at all scopes.
std::unique_ptr<ExtrusionEntity> make_cache(size_t marker, bool sparse, bool emit);
// Require every node in an owned cache subtree to use the generic stored-content type.
void require_generic_cache(const ExtrusionEntity &entity);
// Serialize local geometry and every fixture property without any identity renaming.
void write_cache_local(const ExtrusionEntity &entity, std::ostream &out);
// Serialize a complete stored tree using a shared identity correspondence.
void write_cache_tree(const ExtrusionEntity &entity, std::ostream &out, std::map<uint64_t, size_t> &ids);
// Return local state for strict copy/transfer comparisons.
std::string cache_local(const ExtrusionEntity &entity);
// Record input addresses and local state while every source node is still alive.
void observe_cache(const ExtrusionEntity &entity, std::map<uint64_t, CacheObservation> &observations,
                   bool present, bool copied);
// Borrow output nodes, permitting only a repeated reference to the same aliased root.
void collect_cache_nodes(const ExtrusionEntity &entity, std::map<uint64_t, const ExtrusionEntity *> &nodes);
// Match each complete stored state to its native reference digest.
void record_cache(const std::string &name, const std::string &state);
// Serialize native region-island buckets after a public Layer operation.
std::string layer_cache_snapshot(const Layer &layer, ExtrusionRole role);
// Install explicit LayerRegion input surfaces rather than relying on perimeter fixture preparation.
void install_fill_inputs(Layer &layer, Surfaces surfaces);
// Supply native region lookups with owned context and, for cached anchoring, a native enum backed by static tables.
// Publish both projections together; global definitions and earlier snapshots stay unchanged.
void prepare_native_fill_context(PreparedPerimeterPrint &prepared, Layer &layer, bool cached_anchoring = false);

} // namespace

// Direct test configurations must size extruder fields as printer presets normally do.
TEST_CASE("Native fill fixtures size settings for every configured extruder", "[plugins][native-fill-cache]")
{
    for (const bool distinct : {false, true}) {
        CAPTURE(distinct);
        const DynamicPrintConfig config = cache_config(distinct);
        const size_t extruder_count = distinct ? 2 : 1;
        for (const std::string &key : PrintConfigDef::instance().extruder_option_keys()) {
            // Printer profile normalization deliberately leaves this preset list unchanged.
            if (key == "default_filament_profile")
                continue;
            CAPTURE(key);
            const ConfigOption *option = config.option(key);
            REQUIRE(option != nullptr);
            REQUIRE(option->is_vector());
            CHECK(option->size() == extruder_count);
        }
    }
}

TEST_CASE("Native fill publication preserves priority copies and owner transfers", "[plugins][native-fill-cache]")
{
    const std::vector<std::vector<std::vector<int>>> recipes{
        {{1}}, {{0}}, {{1, 1}}, {{1, 0, 1}}, {{1}, {1}},
        {{}, {}, {1}}, {{1, 1}, {}, {1}}, {{1, 0, 1}, {1}},
        {{1, 1}, {}, {1, 1}}, {{}, {1, 1, 0}, {}}
    };
    for (const bool distinct : {false, true}) {
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, cache_config(distinct));
        REQUIRE(prepared.print.object(0).layer_count() > 1);
        Layer &layer = prepared.print.object(0).layer(1);
        for (size_t recipe = 0; recipe < recipes.size(); ++recipe)
            for (size_t family = 0; family < 3; ++family) {
                CAPTURE(distinct, recipe, family);
                LayerSliceIsland island(rectangle_expolygon(-10., -10., 10., 10.));
                island.fill_regions(layer);
                REQUIRE(island.regions().count(&layer.region(0)) == 1);
                const LayerRegionSetCPtrs regions{&layer.region(0)};
                size_t count = 0;
                for (const std::vector<int> &priority : recipes[recipe])
                    count += priority.size();
                Surfaces metadata;
                metadata.reserve(count);
                CacheMatrix caches(recipes[recipe].size());
                std::map<uint64_t, CacheObservation> observed;
                bool has_sparse = false, has_solid = false;
                size_t marker = 0;
                for (size_t priority = 0; priority < recipes[recipe].size(); ++priority)
                    for (const int emit : recipes[recipe][priority]) {
                        const bool sparse = family == 0 || (family == 2 && marker % 2 == 0);
                        has_sparse |= sparse;
                        has_solid |= !sparse;
                        metadata.emplace_back(stPosInternal | (sparse ? stDensSparse : stDensSolid),
                                              rectangle_expolygon(-8., -8., 8., 8.));
                        std::unique_ptr<ExtrusionEntity> cache = make_cache(marker++, sparse, emit != 0);
                        const bool copy = caches.size() > 1 && recipes[recipe][priority].size() > 1;
                        observe_cache(*cache, observed, emit != 0, copy);
                        if (caches.size() == 1 && recipes[recipe][priority].size() == 1)
                            observed.at(cache->get_id()).present = false;
                        caches[priority].emplace_back(&metadata.back(), std::move(cache));
                    }
                const Surfaces metadata_before = metadata;
                ExtrusionEntityCollection *sparse = has_sparse ?
                    &island.get_or_add_region_island(regions, 0).mutable_extrusion(LayerRegionIsland::INFILLS) : nullptr;
                ExtrusionEntityCollection *solid = has_solid ?
                    &island.get_or_add_region_island(regions, distinct ? 1 : 0).mutable_extrusion(LayerRegionIsland::INFILLS) : nullptr;
                const bool rotation = recipe % 2 != 0;
                for (ExtrusionEntityCollection *root : {sparse, solid})
                    if (root != nullptr) {
                        root->set_can_sort_reverse(true, true);
                        root->set_can_rotate(rotation);
                        mark_cache_node(*root, 9000);
                        CHECK(root->entities().empty());
                    }
                CHECK(bool(sparse != nullptr && solid != nullptr && sparse == solid) ==
                      bool(has_sparse && has_solid && !distinct));
                std::map<uint64_t, size_t> ids;
                std::ostringstream state;
                state.imbue(std::locale::classic());
                state << std::hexfloat << "before[";
                for (const std::vector<std::pair<const Surface *, std::unique_ptr<ExtrusionEntity>>> &priority : caches) {
                    state << '[';
                    for (const std::pair<const Surface *, std::unique_ptr<ExtrusionEntity>> &entry : priority)
                        write_cache_tree(*entry.second, state, ids);
                    state << ']';
                }
                state << "]roots[";
                for (const ExtrusionEntityCollection *root : {sparse, solid}) {
                    if (root != nullptr) write_cache_tree(*root, state, ids);
                    else state << '-';
                }
                state << "]after[";
                FillCacheDetail::store_fill(caches, island, regions);
                CHECK(caches.empty());
                std::map<uint64_t, const ExtrusionEntity *> output;
                for (const ExtrusionEntityCollection *root : {sparse, solid}) {
                    if (root != nullptr) {
                        CHECK(root->can_sort() == (recipes[recipe].size() == 1));
                        CHECK(root->can_reverse() == (recipes[recipe].size() == 1));
                        CHECK(root->can_rotate() == rotation);
                        REQUIRE(root->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
                        CHECK(root->custom_gcode_string(*root->get_property<ExtrusionPropertyCustomGcode>()) ==
                              std::string(180, 'c') + "-9000");
                        const ExtrusionEntitiesPtr &published = root->entities();
                        REQUIRE(published.size() == root->child_count());
                        for (size_t index = 0; index < published.size(); ++index)
                            CHECK(published[index] == root->children()[index].get());
                        for (const ExtrusionEntityUPtr &child : root->children())
                            require_generic_cache(*child);
                        write_cache_tree(*root, state, ids);
                        collect_cache_nodes(*root, output);
                    } else state << '-';
                }
                state << "]handles[";
                for (const std::pair<const uint64_t, CacheObservation> &entry : observed) {
                    const std::map<uint64_t, const ExtrusionEntity *>::const_iterator found = output.find(entry.first);
                    CHECK((found != output.end()) == entry.second.present);
                    state << ids.at(entry.first) << ':' << (found != output.end());
                    if (found != output.end()) {
                        const bool same = reinterpret_cast<std::uintptr_t>(found->second) == entry.second.address;
                        CHECK(same == !entry.second.copied);
                        CHECK(cache_local(*found->second) == entry.second.local);
                        state << same;
                    }
                    state << ';';
                }
                state << ']';
                REQUIRE(metadata.size() == metadata_before.size());
                for (size_t index = 0; index < metadata.size(); ++index) {
                    CHECK(metadata[index].surface_type == metadata_before[index].surface_type);
                    CHECK(metadata[index].expolygon.contour.points == metadata_before[index].expolygon.contour.points);
                }
                record_cache("storage-" + std::to_string(recipe) + '-' + std::to_string(family) + '-' + std::to_string(distinct), state.str());
            }
    }
}

TEST_CASE("Native Layer filling consumes explicitly prepared surfaces", "[plugins][native-fill-cache]")
{
    for (size_t recipe = 0; recipe < 6; ++recipe) {
        CAPTURE(recipe);
        PreparedPerimeterPrint prepared;
        DynamicPrintConfig config = cache_config();
        config.set_deserialize_strict("infill_dense", recipe == 5 ? "1" : "0");
        prepare_cube_print(prepared, config);
        Layer &layer = prepared.print.object(0).layer(1);
        prepare_native_fill_context(prepared, layer);
        Surfaces surfaces;
        if (recipe == 1 || recipe == 2)
            surfaces.emplace_back(stPosInternal | (recipe == 1 ? stDensSparse : stDensSolid), rectangle_expolygon(-8., -8., 8., 8.));
        if (recipe >= 3) {
            surfaces.emplace_back(stPosInternal | stDensSparse, rectangle_expolygon(-8., -8., -1., 8.));
            surfaces.emplace_back(stPosInternal | (recipe == 4 ? stDensSolid : stDensSparse), rectangle_expolygon(1., -8., 8., 8.));
            if (recipe == 5) {
                surfaces.back().surface_type = stPosInternal | stDensSparse | stModBridge;
                surfaces.back().maxNbSolidLayersOnTop = 1;
                surfaces.back().priority = 2;
            }
        }
        install_fill_inputs(layer, std::move(surfaces));
        for (LayerSliceIsland &island : layer.islands())
            island.get_or_add_region_island(island.regions(), 0).mutable_extrusion(LayerRegionIsland::IRONINGS)
                .append_child(make_cache(123, false, true));
        Slic3r::thread_data().seed_random_generator(42);
        layer.make_fills(nullptr, nullptr, nullptr);
        size_t filled = 0;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands()) {
                CHECK_FALSE(region.has_extrusion(LayerRegionIsland::IRONINGS));
                if (region.has_extrusion(LayerRegionIsland::INFILLS)) {
                    ++filled;
                    CHECK(region.extrusion(LayerRegionIsland::INFILLS).can_sort() == (recipe != 5));
                    CHECK(region.extrusion(LayerRegionIsland::INFILLS).total_volume() > 0.);
                }
            }
        CHECK(filled == (recipe == 0 ? 0 : 1));
        record_cache("public-fill-" + std::to_string(recipe), layer_cache_snapshot(layer, LayerRegionIsland::INFILLS));
    }
}

TEST_CASE("Native anchoring retains owned points from both generation branches", "[plugins][native-fill-cache]")
{
    for (const InfillPattern pattern : {ipRectilinear, ipRectiWithPerimeter}) {
        CAPTURE(int(pattern));
        Polylines lines;
        {
            PreparedPerimeterPrint prepared;
            prepare_cube_print(prepared, cache_config());
            Layer &layer = prepared.print.object(0).layer(1);
            prepare_native_fill_context(prepared, layer, pattern == ipRectiWithPerimeter);
            install_fill_inputs(layer, {Surface(stPosInternal | stDensSparse, rectangle_expolygon(-8., -8., 8., 8.))});
            REQUIRE(region_infill_pattern(layer.region(0).region().config(), "fill_pattern", ipStars) == pattern);
            const std::unique_ptr<Fill> filler(Fill::new_from_type(pattern));
            REQUIRE(filler != nullptr);
            CHECK(bool(filler->can_fill_surface_single) == (pattern == ipRectilinear));
            Slic3r::thread_data().seed_random_generator(42);
            lines = layer.generate_sparse_infill_polylines_for_anchoring(nullptr, nullptr, nullptr);
        }
        REQUIRE_FALSE(lines.empty());
        std::ostringstream state;
        for (const Polyline &line : lines) {
            REQUIRE(line.points.size() >= 2);
            state << '[';
            for (const Point &point : line.points) state << point.x() << ',' << point.y() << ';';
            state << ']';
        }
        record_cache("anchoring-" + std::to_string(int(pattern)), state.str());
    }
}

TEST_CASE("Native ironing wrapper choice uses all areas rather than productive areas", "[plugins][native-fill-cache]")
{
    for (size_t recipe = 0; recipe < 4; ++recipe) {
        CAPTURE(recipe);
        PreparedPerimeterPrint prepared;
        DynamicPrintConfig config = cache_config();
        config.set_deserialize_strict({{"ironing", "1"}, {"ironing_type", "top"},
            {"ironing_angle", "0"}, {"ironing_spacing", "0.2"}, {"top_solid_layers", "1"}});
        prepare_cube_print(prepared, config);
        Layer &layer = prepared.print.object(0).layer(1);
        ExPolygons areas;
        if (recipe != 0) areas.push_back(rectangle_expolygon(-8., -8., -1., 8.));
        if (recipe == 2) areas.push_back(rectangle_expolygon(1., -8., 8., 8.));
        if (recipe == 3) areas.push_back(rectangle_expolygon(4.023, 4.023, 4.033, 4.033));
        ApiInternal::LayerRegionAccess::surfaces_mutable(layer.region(0)).set(areas, stPosTop | stDensSolid);
        layer.region(0).set_fill_surfaces().set(areas, stPosTop | stDensSolid);
        Slic3r::thread_data().seed_random_generator(42);
        layer.make_ironing();
        size_t filled = 0;
        for (const LayerSliceIsland &island : layer.islands())
            for (const LayerRegionIsland &region : island.regions_islands())
                if (region.has_extrusion(LayerRegionIsland::IRONINGS)) {
                    ++filled;
                    const ExtrusionEntity &root = region.extrusion(LayerRegionIsland::IRONINGS);
                    CHECK(root.can_sort() == (recipe >= 2));
                    CHECK(root.can_reverse() == (recipe >= 2));
                    CHECK(root.can_rotate());
                    if (recipe >= 2) {
                        REQUIRE(root.child_count() == (recipe == 2 ? 2 : 1));
                        for (const ExtrusionEntityUPtr &group : root.children()) {
                            CHECK_FALSE(group->can_sort());
                            CHECK_FALSE(group->can_reverse());
                            CHECK(group->can_rotate());
                            CHECK_FALSE(group->empty());
                        }
                    }
                }
        CHECK(filled == (recipe == 0 ? 0 : 1));
        record_cache("ironing-" + std::to_string(recipe), layer_cache_snapshot(layer, LayerRegionIsland::IRONINGS));
    }
}

namespace {

DynamicPrintConfig cache_config(bool distinct_extruders)
{
    DynamicPrintConfig config = perimeter_config({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "0"}, {"fill_density", "25%"}, {"fill_pattern", "rectilinear"},
        {"solid_fill_pattern", "rectilinear"}, {"infill_extruder", "1"},
        {"solid_infill_extruder", distinct_extruders ? "2" : "1"},
        {"nozzle_diameter", distinct_extruders ? "0.4,0.4" : "0.4"}, {"ironing", "0"},
        {"infill_dense", "0"}, {"enforce_full_fill_volume", "0"}, {"arc_fitting", "disabled"}});
    config.set_num_extruders(distinct_extruders ? 2 : 1);
    return config;
}

void mark_cache_node(ExtrusionEntity &node, size_t marker)
{
    node.add_property(ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcode::Code::COMMENT,
                      std::string(180, 'c') + '-' + std::to_string(marker)));
    node.get_or_add_property<ExtrusionPropertySpeed>().speed_mm_per_s = float(10 + marker);
}

std::unique_ptr<ExtrusionEntity> make_cache(size_t marker, bool sparse, bool emit)
{
    std::unique_ptr<ExtrusionEntity> cache = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, marker % 2 == 0, marker % 3 != 0, false);
    cache->set_can_rotate(marker % 2 != 0);
    mark_cache_node(*cache, marker);
    if (emit) {
        ExtrusionEntityUPtr group = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children{}, false, true, false);
        group->set_can_rotate(false);
        mark_cache_node(*group, 100 + marker);
        for (size_t index = 0; index < 2; ++index) {
            const coord_t start = coord_t(marker * 4000000 + index * 1000000);
            ExtrusionEntityUPtr leaf = std::make_unique<ExtrusionEntity>(index != 0,
                ArcPolyline(Points{{start, 0}, {start + 1000000, 500000}}));
            leaf->set_can_rotate(index == 0);
            leaf->add_property(ExtrusionAttributes{sparse ? ExtrusionRole::InternalInfill : ExtrusionRole::SolidInfill,
                ExtrusionFlow{0.12 + marker * 0.01, 0.45f, 0.2f}});
            mark_cache_node(*leaf, 1000 + marker * 2 + index);
            group->append_child(std::move(leaf));
        }
        cache->append_child(std::move(group));
    }
    require_generic_cache(*cache);
    return cache;
}

void require_generic_cache(const ExtrusionEntity &entity)
{
    CHECK(typeid(entity) == typeid(ExtrusionEntity));
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        require_generic_cache(*child);
    }
}

void write_cache_local(const ExtrusionEntity &entity, std::ostream &out)
{
    out << entity.is_leaf() << entity.can_sort() << entity.can_reverse() << entity.can_rotate()
        << entity.ExtrusionEntity::is_loop() << ':';
    const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
    const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>();
    const ExtrusionPropertyCustomGcode *text = entity.get_property<ExtrusionPropertyCustomGcode>();
    const ExtrusionPropertyLoopRole *loop = entity.get_property<ExtrusionPropertyLoopRole>();
    REQUIRE(ApiInternal::ExtrusionPropertyAccess::property_count(entity) ==
        size_t(attributes != nullptr) + size_t(speed != nullptr) + size_t(text != nullptr) + size_t(loop != nullptr));
    if (attributes != nullptr)
        out << 'a' << int32_t(attributes->role) << ',' << attributes->mm3_per_mm << ','
            << attributes->width << ',' << attributes->height << ',' << int(attributes->no_seam);
    if (speed != nullptr) out << 's' << speed->speed_mm_per_s;
    if (text != nullptr) out << 't' << text->kind << ',' << text->script_type << ',' << std::quoted(entity.custom_gcode_string(*text));
    if (loop != nullptr) out << 'l' << loop->perimeter_flags << ',' << loop->perimeter_idx << ',' << loop->reserved;
    if (const ArcPolyline *line = entity.polyline_or_null()) {
        CHECK_FALSE(line->has_arc());
        CHECK_FALSE(line->has_z_offset());
        REQUIRE(line->size() >= 2);
        for (size_t index = 0; index < line->size(); ++index)
            out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y() << ')';
    }
}

void write_cache_tree(const ExtrusionEntity &entity, std::ostream &out, std::map<uint64_t, size_t> &ids)
{
    const size_t next = ids.size();
    out << '{' << ids.emplace(entity.get_id(), next).first->second << ':';
    write_cache_local(entity, out);
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_cache_tree(*child, out, ids);
    }
    out << '}' << entity.total_volume();
}

std::string cache_local(const ExtrusionEntity &entity)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    write_cache_local(entity, out);
    return out.str();
}

void observe_cache(const ExtrusionEntity &entity, std::map<uint64_t, CacheObservation> &observations,
                   bool present, bool copied)
{
    REQUIRE(observations.emplace(entity.get_id(), CacheObservation{
        reinterpret_cast<std::uintptr_t>(&entity), present, copied, cache_local(entity)}).second);
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        observe_cache(*child, observations, present, copied);
    }
}

void collect_cache_nodes(const ExtrusionEntity &entity, std::map<uint64_t, const ExtrusionEntity *> &nodes)
{
    const std::pair<std::map<uint64_t, const ExtrusionEntity *>::iterator, bool> added = nodes.emplace(entity.get_id(), &entity);
    if (!added.second) {
        REQUIRE(added.first->second == &entity);
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        collect_cache_nodes(*child, nodes);
    }
}

void record_cache(const std::string &name, const std::string &state)
{
    boost::uuids::detail::sha1 digest;
    digest.process_bytes(state.data(), state.size());
    unsigned int words[5];
    digest.get_digest(words);
    std::ostringstream hash;
    for (const unsigned int word : words) hash << std::hex << std::setfill('0') << std::setw(8) << word;
    CAPTURE(name);
    const Slic3r::Test::NativeFillCacheGolden *golden = std::find_if(
        std::begin(Slic3r::Test::native_fill_cache_goldens), std::end(Slic3r::Test::native_fill_cache_goldens),
        [&name](const Slic3r::Test::NativeFillCacheGolden &entry) { return name == entry.name; });
    REQUIRE(golden != std::end(Slic3r::Test::native_fill_cache_goldens));
    CHECK(hash.str() == golden->digest);
}

std::string layer_cache_snapshot(const Layer &layer, ExtrusionRole role)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    std::map<uint64_t, size_t> ids;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands()) {
            out << '[' << region.extruder_id() << ':' << region.has_extrusion(role);
            if (region.has_extrusion(role)) {
                const ExtrusionEntity &root = region.extrusion(role);
                for (const ExtrusionEntityUPtr &child : root.children())
                    require_generic_cache(*child);
                write_cache_tree(root, out, ids);
            }
            out << ']';
        }
    return out.str();
}

void install_fill_inputs(Layer &layer, Surfaces surfaces)
{
    REQUIRE(layer.region_count() == 1);
    for (const Surface &surface : surfaces) {
        bool belongs = false;
        for (const LayerSliceIsland &island : layer.islands())
            belongs |= island.is_expolygons_from_region(surface.expolygon);
        REQUIRE(belongs);
    }
    layer.region(0).set_fill_surfaces().surfaces = std::move(surfaces);
}

void prepare_native_fill_context(PreparedPerimeterPrint &prepared, Layer &layer, bool cached_anchoring)
{
    REQUIRE(layer.region_count() == 1);
    const PrintConfig &global = prepared.print.config();
    const ConfigOptionFloatsOrPercents *source = global.opt<ConfigOptionFloatsOrPercents>("seam_gap");
    REQUIRE(source != nullptr);
    const ConfigOptionFloatsOrPercents source_before = *source;
    const ConfigOptionDef *definition = PrintConfigDef::instance().get("seam_gap");
    REQUIRE(definition != nullptr);
    const ConfigOptionDef definition_before = *definition;
    const ConfigOptionDef *pattern_definition = PrintConfigDef::instance().get("fill_pattern");
    REQUIRE(pattern_definition != nullptr);
    REQUIRE(pattern_definition->type == coEnum);
    REQUIRE(pattern_definition->enum_def.get() != nullptr);
    const bool pattern_was_scripted = pattern_definition->enum_is_scripted();
    const ConfigOptionDef pattern_definition_before = *pattern_definition;
    const size_t definition_count = PrintConfigDef::instance().options.size();
    const PrintRegion *original = &layer.region(0).region();
    const PrintRegionConfig original_before = original->config();
    REQUIRE(original_before.parent == nullptr);
    REQUIRE(original_before.option("seam_gap") == nullptr);
    const PrintRegion *registered = nullptr;
    {
        PrintRegionConfig context = original_before;
        ConfigOptionDef projected = *definition;
        projected.container_type = ConfigOptionContainerType::Region;
        projected.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        projected.default_value.reset(source->clone());
        context.add_plugin_option(projected);
        const ConfigOption *local = context.option("seam_gap");
        REQUIRE(local != nullptr);
        CHECK(local != source);
        CHECK(*local == *source);
        CHECK(local->flags == source->flags);
        if (cached_anchoring) {
            ConfigOptionDef native_pattern = *pattern_definition;
            native_pattern.container_type = ConfigOptionContainerType::Region;
            native_pattern.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
            native_pattern.set_enum<InfillPattern>({"rectilinear", "rectiwithperimeter"});
            native_pattern.set_default_value(new ConfigOptionEnum<InfillPattern>(ipRectiWithPerimeter));
            CHECK_FALSE(native_pattern.enum_is_scripted());
            REQUIRE(native_pattern.enum_def->is_valid_closed_enum());
            REQUIRE(native_pattern.enum_def->enum_to_value(int(ipRectiWithPerimeter)).has_value());
            CHECK(native_pattern.enum_def->enum_to_value(int(ipRectiWithPerimeter)).value().get() == "rectiwithperimeter");
            context.add_plugin_option(native_pattern);
            const ConfigOptionEnumGeneric *pattern = context.opt<ConfigOptionEnumGeneric>("fill_pattern");
            REQUIRE(pattern != nullptr);
            CHECK(pattern->keys_map == &ConfigOptionEnum<InfillPattern>::get_enum_values());
            CHECK(pattern->is_enabled());
            CHECK(pattern->get_int() == int(ipRectiWithPerimeter));
            CHECK(pattern->serialize() == "rectiwithperimeter");
            CHECK(region_infill_pattern(context, "fill_pattern", ipStars) == ipRectiWithPerimeter);
        }
        registered = &prepared.print.register_print_region(context);
        CHECK(&prepared.print.register_print_region(context) == registered);
        CHECK(registered != original);
        CHECK(registered->config().option("seam_gap") != local);
        if (cached_anchoring)
            CHECK(registered->config().option("fill_pattern") != context.option("fill_pattern"));
    }
    REQUIRE(registered != nullptr);
    const PrintRegionConfig &snapshot = registered->config();
    CHECK(snapshot.parent == nullptr);
    const ConfigOptionFloatsOrPercents *owned = snapshot.opt<ConfigOptionFloatsOrPercents>("seam_gap");
    REQUIRE(owned != nullptr);
    CHECK(owned != source);
    CHECK(owned->type() == coFloatsOrPercents);
    CHECK(owned->get_values() == source_before.get_values());
    CHECK(owned->has_same_enabled(source_before));
    CHECK(owned->flags == source_before.flags);
    CHECK(boost::any_cast<FloatOrPercent>(owned->get_default_value()) ==
          boost::any_cast<FloatOrPercent>(source_before.get_default_value()));
    CHECK(owned->is_extruder_size() == source_before.is_extruder_size());
    CHECK(snapshot.keys().size() == original_before.keys().size() + 1);
    for (const std::string &key : original_before.keys()) {
        REQUIRE(snapshot.option(key) != nullptr);
        if (cached_anchoring && key == "fill_pattern") {
            const ConfigOptionEnumGeneric *pattern = snapshot.opt<ConfigOptionEnumGeneric>(key);
            REQUIRE(pattern != nullptr);
            CHECK(pattern->type() == coEnum);
            CHECK(pattern->keys_map == &ConfigOptionEnum<InfillPattern>::get_enum_values());
            CHECK(pattern->is_enabled());
            CHECK(pattern->get_int() == int(ipRectiWithPerimeter));
            CHECK(pattern->serialize() == "rectiwithperimeter");
            CHECK(region_infill_pattern(snapshot, "fill_pattern", ipStars) == ipRectiWithPerimeter);
            CHECK(snapshot.option(key) != original_before.option(key));
        } else {
            CHECK(*snapshot.option(key) == *original_before.option(key));
        }
        CHECK(snapshot.option(key)->flags == original_before.option(key)->flags);
    }
    for (const int64_t one_based : {snapshot.infill_extruder.value, snapshot.solid_infill_extruder.value}) {
        REQUIRE(one_based > 0);
        const extruder_index_t index = static_cast<extruder_index_t>(one_based - 1);
        CHECK(snapshot.get_computed_value("seam_gap", index) == global.get_computed_value("seam_gap", index));
    }
    CHECK(*source == source_before);
    CHECK(source->flags == source_before.flags);
    CHECK(original->config() == original_before);
    CHECK(original->config().parent == nullptr);
    CHECK(original->config().option("seam_gap") == nullptr);
    CHECK(PrintConfigDef::instance().options.size() == definition_count);
    CHECK(definition->type == definition_before.type);
    CHECK(definition->container_type == definition_before.container_type);
    CHECK(definition->option_preset_type == definition_before.option_preset_type);
    CHECK(*definition->default_value == *definition_before.default_value);
    CHECK(definition->default_value->flags == definition_before.default_value->flags);
    CHECK(pattern_definition->type == pattern_definition_before.type);
    CHECK(pattern_definition->container_type == pattern_definition_before.container_type);
    CHECK(pattern_definition->option_preset_type == pattern_definition_before.option_preset_type);
    CHECK(pattern_definition->enum_is_scripted() == pattern_was_scripted);
    CHECK(pattern_definition->enum_def->values() == pattern_definition_before.enum_def->values());
    CHECK(pattern_definition->enum_def->labels() == pattern_definition_before.enum_def->labels());
    CHECK(*pattern_definition->default_value == *pattern_definition_before.default_value);
    ApiInternal::LayerRegionAccess::set_print_region(layer.region(0), *registered);
    CHECK(&layer.region(0).region() == registered);
}

} // namespace
