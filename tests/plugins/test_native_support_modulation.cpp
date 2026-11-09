#include <catch2/catch.hpp>

#include <boost/uuid/detail/sha1.hpp>

#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>

#include "perimeter_test_helpers.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Support/SupportCommon.hpp"

/* Check native support modulation directly, without support generation or an
   exporter. Equal-flow fixtures use native, generic and inherited attributes.
   Overlapping layer rectangles exercise height reduction and reconnection while
   complete snapshots retain geometry, flow, order and permissions. A no-overlap
   call must leave handles unchanged; rebuilding must own all resulting geometry.

       TEST_CASE -> modulation_source(): own the uniform source cache
           |-- configure_overlap(): prepare sorted clipping layers
           |-- modulate_extrusion_by_overlapping_layers(): rebuild fragments
           `-- support_snapshot() / contextual_support_volume(): verify the result
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::FFFSupport;
using namespace Slic3r::Test::PerimeterPluginTests;

// Build equal-flow native paths, including two representations with identical stored trees.
ExtrusionEntityCollection modulation_source(const std::string &form, const std::string &representation = "native");
// Serialize stored topology, direct attributes and every ordered XY point without dynamic types or IDs.
void write_support_tree(const ExtrusionEntity &entity, std::ostringstream &out);
// Return the deterministic stored tree text used to compare source representations.
std::string support_snapshot(const ExtrusionEntity &entity);
// Hash full-precision tree text for the frozen regression oracle.
std::string support_digest(const std::string &text);
// Set one overlapping layer's height and trimming rectangle.
void configure_overlap(SupportGeneratorLayer &layer, double print_z, double min_x, double max_x, double min_y, double max_y);
// Remove direct attributes so a generic group can supply the same uniform flow by inheritance.
void remove_support_attributes(ExtrusionEntity &entity);
// Check exact node types only after reconstruction, not on the no-op input path.
void require_generic_support(const ExtrusionEntity &entity);
// Sum leaf lengths with inherited attributes, independently of modulation and its geometry reader.
double contextual_support_volume(const ExtrusionEntity &entity);

const char *const historical_digests[5][5] = {
    {"140efea1200d43d4c55215bc041060f576930647", "140efea1200d43d4c55215bc041060f576930647",
     "83f100520e81eb9286c889d27639966945d31ee5", "dea3a1cfcc67921d80ffceff3da34626f4a9230d", "2513132861da192768e5e9744ea5c5b654cfaf32"},
    {"158a4a8ebdb66bd9779ebc07ebc395bc98fa0f87", "84aed88eda646d19ab0626023704baa3d1d0be52",
     "2def0890934935c52bf335063d4f54fec6d9f37d", "94dce8e02d7aacc0e49223d169d7e16cd3c180be", "30d93e322add1f0a17eca40e53550cb9dcdd0284"},
    {"7635c37eef81bb4932ef85418cc389233ebd2950", "140efea1200d43d4c55215bc041060f576930647",
     "83f100520e81eb9286c889d27639966945d31ee5", "dea3a1cfcc67921d80ffceff3da34626f4a9230d", "2513132861da192768e5e9744ea5c5b654cfaf32"},
    {"7635c37eef81bb4932ef85418cc389233ebd2950", "fcb79d506fa9ec75a9807b10c20f81b906d52831",
     "3ad23417bccd1c4a91f9d88d4a08563c3cf4912e", "f0bb6ab2e1986a634e7ee9d71780c6d115bf7d9c", "12120d1f3e4c264d4056a913bf524d762e91e87d"},
    {"b1fb370ae9d90289d8f957cc9972fd9df5de7b7f", "b1fb370ae9d90289d8f957cc9972fd9df5de7b7f",
     "4c3f46f642b2e00dfb9879b931e5290947caeafd", "2616faa6fd101d0b4733738380707daedf8e26bf", "b0987d3bab88334dbfc22a123437f38fdc705962"}
};

ExtrusionEntityCollection modulation_source(const std::string &form, const std::string &representation)
{
    const ExtrusionAttributes attributes{ExtrusionRole::SupportMaterialInterface, ExtrusionFlow{0.12, 0.45f, 0.4f}};
    ExtrusionEntityCollection output;
    if (form == "path" || form == "separate") {
        output.append(ExtrusionPath(ArcPolyline(Points{{0, 0}, {scale_i(5.), 0}, {scale_i(12.), 0}}), attributes, nullptr, false));
        if (form == "separate")
            output.append(ExtrusionPath(ArcPolyline(Points{{0, scale_i(4.)}, {scale_i(12.), scale_i(4.)}}), attributes, nullptr, false));
    } else {
        ExtrusionPaths paths;
        if (form == "loop") {
            paths.emplace_back(ArcPolyline(Points{{0, 0}, {scale_i(12.), 0}, {scale_i(12.), scale_i(6.)}}), attributes, nullptr, false);
            paths.emplace_back(ArcPolyline(Points{{scale_i(12.), scale_i(6.)}, {0, scale_i(6.)}, {0, 0}}), attributes, nullptr, false);
            output.append(ExtrusionLoop(std::move(paths)));
        } else {
            paths.emplace_back(ArcPolyline(Points{{0, 0}, {scale_i(5.), 0}}), attributes, nullptr, false);
            paths.emplace_back(ArcPolyline(Points{{scale_i(5.), 0}, {scale_i(12.), 0}}), attributes, nullptr, false);
            if (form == "multipath") {
                output.append(ExtrusionMultiPath(paths));
            } else {
                REQUIRE(form == "fixed");
                ExtrusionEntityCollection fixed(false, false);
                fixed.set_can_rotate(false);
                fixed.append(std::move(paths));
                output.append(std::move(fixed));
            }
        }
    }
    if (representation != "native") {
        for (ExtrusionEntityUPtr &child : output.children()) {
            child = child->clone_canonical();
            if (representation == "inherited") {
                const bool can_rotate = child->can_rotate();
                remove_support_attributes(*child);
                ExtrusionEntity::Children children;
                children.emplace_back(std::move(child));
                child = std::make_unique<ExtrusionEntity>(std::move(children), false, false, true);
                child->set_can_rotate(can_rotate);
                child->add_property(attributes);
            } else {
                REQUIRE(representation == "generic");
            }
        }
    }
    return output;
}

void write_support_tree(const ExtrusionEntity &entity, std::ostringstream &out)
{
    out << '{' << entity.is_leaf() << entity.can_sort() << entity.can_reverse()
        << entity.can_rotate() << entity.ExtrusionEntity::is_loop() << ':';
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        out << int32_t(attributes->role) << ',' << attributes->mm3_per_mm << ',' << attributes->width
            << ',' << attributes->height << ',' << attributes->no_seam << ':';
    if (const ArcPolyline *line = entity.polyline_or_null()) {
        CHECK_FALSE(line->has_arc());
        CHECK_FALSE(line->has_z_offset());
        REQUIRE(line->size() >= 2);
        for (size_t index = 0; index < line->size(); ++index)
            out << '(' << line->get_point(index).x() << ',' << line->get_point(index).y() << ')';
    }
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        write_support_tree(*child, out);
    }
    out << '}';
}

std::string support_snapshot(const ExtrusionEntity &entity)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hexfloat;
    write_support_tree(entity, out);
    return out.str();
}

std::string support_digest(const std::string &text)
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

void configure_overlap(SupportGeneratorLayer &layer, double print_z, double min_x, double max_x, double min_y, double max_y)
{
    layer.layer_type = SupporLayerType::Base;
    layer.set_scaled_print_z(scale_to_layer_coord(print_z));
    layer.set_scaled_height(scale_to_layer_coord(0.2));
    layer.set_scaled_bottom_z(scale_to_layer_coord(print_z - 0.2));
    layer.polygons = {rectangle_expolygon(min_x, min_y, max_x, max_y).contour};
}

void remove_support_attributes(ExtrusionEntity &entity)
{
    entity.remove_property<ExtrusionAttributes>();
    CHECK(entity.get_property<ExtrusionAttributes>() == nullptr);
    if (!entity.is_leaf())
        for (ExtrusionEntityUPtr &child : entity.children())
            remove_support_attributes(*child);
}

void require_generic_support(const ExtrusionEntity &entity)
{
    REQUIRE(typeid(entity) == typeid(ExtrusionEntity));
    for (const ExtrusionEntityUPtr &child : entity.children()) {
        REQUIRE(child != nullptr);
        require_generic_support(*child);
    }
}

double contextual_support_volume(const ExtrusionEntity &entity)
{
    struct Volume : ExtrusionTreeConstVisitor<false> {
        double value = 0.;
        void visit_leaf(const ExtrusionEntity &leaf) override
        {
            if (const ArcPolyline *polyline = leaf.polyline_or_null()) {
                const ExtrusionAttributes *attributes = current_property<ExtrusionAttributes>();
                REQUIRE(attributes != nullptr);
                value += unscaled(polyline->length()) * attributes->mm3_per_mm;
            }
        }
    } volume;
    volume.traverse(entity);
    return volume.value;
}

} // namespace

TEST_CASE("Native support modulation characterizes source boundaries and flow reconstruction",
          "[plugins][native-support-modulation]")
{
    const ExtrusionEntityCollection multipath = modulation_source("multipath");
    const ExtrusionEntityCollection fixed = modulation_source("fixed");
    REQUIRE(support_snapshot(multipath) == support_snapshot(fixed));
    const char *const forms[] = {"path", "loop", "multipath", "fixed", "separate"};
    for (size_t shape = 0; shape < 5; ++shape) {
        const std::string form = forms[shape];
        for (size_t mode = 0; mode < 5; ++mode) {
            for (const std::string representation : {"native", "generic", "inherited"}) {
                CAPTURE(form, mode, representation);
                ExtrusionEntityCollection output = modulation_source(form, representation);
                const uint64_t root_id = output.get_id();
                const ExtrusionEntity *first = &output.child(0);
                const std::string before = support_snapshot(output);
                const double legacy_volume_before = output.total_volume();
                SupportGeneratorLayer this_layer;
                this_layer.layer_type = SupporLayerType::BottomContact;
                this_layer.set_scaled_print_z(scale_to_layer_coord(1.));
                this_layer.set_scaled_height(scale_to_layer_coord(0.4));
                this_layer.set_scaled_bottom_z(scale_to_layer_coord(0.6));
                SupportGeneratorLayer lower;
                SupportGeneratorLayer upper;
                configure_overlap(lower, 0.8, mode == 1 ? 30. : 3., mode == 1 ? 40. : (mode == 4 ? 5. : 8.), -2., 8.);
                configure_overlap(upper, mode == 4 ? 0.8 : 0.9, mode == 4 ? 5. : 6., mode == 4 ? 7. : 10., -2., 8.);
                SupportGeneratorLayersPtr overlaps;
                if (mode > 0)
                    overlaps.push_back(&lower);
                if (mode >= 3)
                    overlaps.push_back(&upper);
                ExtrusionEntity &borrowed_output = output;
                modulate_extrusion_by_overlapping_layers(borrowed_output, this_layer, overlaps);
                REQUIRE_FALSE(output.empty());
                CHECK(output.get_id() == root_id);
                if (mode != 0 || representation != "inherited")
                    CHECK(output.total_volume() > 0.);
                CHECK(contextual_support_volume(output) > 0.);
                const std::string after = support_snapshot(output);
                if (mode == 0) {
                    CHECK(&output.child(0) == first);
                    CHECK(after == before);
                    CHECK(output.total_volume() == legacy_volume_before);
                }
                if (mode != 0 || representation != "inherited") {
                    // Equal stored units share the historical MultiPath oracle. The four
                    // Collection projections remain frozen separately, not recalibrated.
                    const size_t reference = form == "fixed" && mode != 0 ? 2 : shape;
                    CHECK(support_digest(after) == historical_digests[reference][mode]);
                    if (form == "fixed" && mode != 0)
                        CHECK(support_digest(after) != historical_digests[shape][mode]);
                }
                if (mode != 0) {
                    for (const ExtrusionEntityUPtr &child : output.children())
                        require_generic_support(*child);
                }
                if (mode == 1 && (form == "path" || form == "multipath" || form == "fixed")) {
                    REQUIRE(output.child_count() == 1);
                    const ExtrusionEntity &path = output.child(0);
                    REQUIRE(path.has_polyline());
                    const Points expected{{0, 0}, {5000000, 0}, {12000000, 0}};
                    CHECK(path.polyline_ref().to_polyline().points == expected);
                    CHECK_FALSE(path.can_reverse());
                    CHECK(path.can_rotate());
                }
                if (mode == 2 && form != "loop") {
                    REQUIRE(output.child_count() == (form == "separate" ? 2 : 1));
                    for (const ExtrusionEntityUPtr &unit : output.children()) {
                        REQUIRE(unit->child_count() == 3);
                        CHECK_FALSE(unit->can_sort());
                        CHECK_FALSE(unit->can_reverse());
                        CHECK_FALSE(unit->can_rotate());
                        for (size_t index = 0; index < 3; ++index) {
                            const ExtrusionEntity &leaf = unit->child(index);
                            const ExtrusionAttributes *attributes = leaf.get_property<ExtrusionAttributes>();
                            REQUIRE(attributes != nullptr);
                            CHECK_FALSE(leaf.can_reverse());
                            CHECK(attributes->extrusion_role() == ExtrusionRole::SupportMaterialInterface);
                            CHECK(attributes->width == 0.45f);
                            CHECK(attributes->height == (index == 1 ? 0.2f : 0.4f));
                            CHECK(attributes->mm3_per_mm == (index == 1 ? 0.12 * 0.2f / 0.4f : 0.12));
                        }
                    }
                }
            }
        }
    }
}
