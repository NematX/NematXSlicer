#include <catch2/catch.hpp>

#include <cmath>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Support/Common/SupportZPlacement.hpp"

/* Z placement of the dense interface slabs, checked on synthetic contacts without slicing a model. Each scene places the
   intermediate slabs between a bottom contact on the first layer and a top contact, or between top contacts, and reads
   the base slabs bottom first. The checks cover the handed-over layout (dense stacks, the middle given to the splitter,
   the even split of an interval that the stacks cannot fill) and the grid layout as the reference. */

using namespace Slic3r;
namespace SC = slic3r_api::Support::Common;

namespace {
// Rules of the scenes: a 0.2-mm first layer, 0.05-mm thinnest slab, 0.01-mm machine grid, 0.3-mm sparse and 0.1-mm dense
// slabs, with the same number of dense slabs at the top and at the bottom.
SC::SlabSpacing spacing(int32_t interfaces)
{
    SC::SlabSpacing result;
    result.first_z = scale_to_layer_coord(0.2);
    result.min_height = scale_to_layer_coord(0.05);
    result.machine_step = scale_to_layer_coord(0.01);
    result.base_step = scale_to_layer_coord(0.3);
    result.interface_step = scale_to_layer_coord(0.1);
    result.interface_layers = interfaces;
    result.bottom_interface_layers = interfaces;
    return result;
}

// A bottom contact whose top is the first layer, and a top contact at top_z whose thickness is still to be assigned.
struct Scene {
    explicit Scene(coord_t top_z)
    {
        bottom = &SC::add_layer(support, SC::LayerType::BottomContact, scale_to_layer_coord(0.2), 0, scale_to_layer_coord(0.2));
        support.bottom.push_back(bottom);
        top = &SC::add_layer(support, SC::LayerType::TopContact, top_z, top_z, 0);
        support.top.push_back(top);
    }
    PluginStorage storage;
    SC::GeneratedSupport support{reinterpret_cast<storage_handle *>(&storage)};
    SC::GeneratorLayer *bottom = nullptr, *top = nullptr;
};

// A splitter that records each middle span and fills it with one slab of the support.
SC::MiddleSplitter recording(SC::GeneratedSupport &support, std::vector<std::pair<coord_t, coord_t>> &spans)
{
    return [&support, &spans](coord_t from, coord_t to) {
        spans.emplace_back(from, to);
        support.base.push_back(&SC::add_layer(support, SC::LayerType::Intermediate, to, from, to - from));
    };
}

// True when every base slab starts where the previous one ends, from the bed up to the given Z.
bool contiguous(const SC::GeneratorLayers &base, coord_t top_z)
{
    coord_t previous = 0;
    for (const SC::GeneratorLayer *slab : base) {
        if (slab->bottom_z != previous) return false;
        previous = slab->print_z;
    }
    return previous == top_z;
}

// True when every base slab after the first-layer slab lies between the minimum height and the dense step.
bool dense_within_bounds(const SC::GeneratorLayers &base)
{
    for (size_t i = 1; i < base.size(); ++i)
        if (base[i]->height < scale_to_layer_coord(0.05) || base[i]->height > scale_to_layer_coord(0.1)) return false;
    return true;
}

// Number of base slabs whose print Z is z.
size_t slabs_at(const SC::GeneratorLayers &base, coord_t z)
{
    size_t count = 0;
    for (const SC::GeneratorLayer *slab : base) count += slab->print_z == z ? 1 : 0;
    return count;
}
} // namespace

// The dense slabs stand right above the bottom contact and right under the top contact, the splitter gets the sparse span between
// the two stacks, and the top contact keeps its Z while its thickness becomes the dense step.
TEST_CASE("Handed-over middle keeps dense stacks at both ends of an interval", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(3.));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, scene.support, recording(scene.support, spans));
    CHECK(scene.top->print_z == scale_to_layer_coord(3.));
    CHECK(scene.top->height == scale_to_layer_coord(0.1));
    CHECK(scene.top->bottom_z == scale_to_layer_coord(2.9));
    REQUIRE(spans.size() == 1);
    const SC::GeneratorLayers &base = scene.support.base;
    REQUIRE(base.size() == 6);
    CHECK(base.front()->bottom_z == 0);
    CHECK(base.front()->print_z == scale_to_layer_coord(0.2));
    CHECK(base[1]->bottom_z == scene.bottom->print_z);
    CHECK(contiguous(base, scene.top->bottom_z));
    CHECK(spans.front() == std::make_pair(base[2]->print_z, base[4]->bottom_z));
    for (size_t dense : {size_t(1), size_t(2), size_t(4), size_t(5)}) {
        CAPTURE(dense);
        CHECK(base[dense]->height == scale_to_layer_coord(0.1));
    }
}

// An interval shorter than its dense stacks gets even dense slabs and no middle, with as many slabs as the grid spaces, the top
// contact being the last one.
TEST_CASE("Handed-over short interval reduces the dense counts like the grid", "[plugins][support-grid][support-placement]")
{
    const coord_t top_z = scale_to_layer_coord(0.45);
    Scene handed(top_z), uniform(top_z);
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, handed.support, recording(handed.support, spans));
    SC::place_intermediate_slabs(spacing(3), {}, uniform.support);
    CHECK(spans.empty());
    REQUIRE(handed.support.base.size() == uniform.support.base.size());
    CHECK(contiguous(handed.support.base, handed.top->bottom_z));
    // 0.25 mm over three slabs: the top contact and two dense slabs, each within a machine step of a third of the interval.
    CHECK(std::abs(unscaled(handed.top->height) - 0.25 / 3.) < 0.01);
    CHECK(handed.top->height >= scale_to_layer_coord(0.05));
    CHECK(handed.top->height <= scale_to_layer_coord(0.1));
    CHECK(dense_within_bounds(handed.support.base));
}

// When the two dense stacks fill the interval exactly, the splitter is not called and the stacks meet under the top contact.
TEST_CASE("Handed-over interval without a middle calls no splitter", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(0.7));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, scene.support, recording(scene.support, spans));
    CHECK(spans.empty());
    const SC::GeneratorLayers &base = scene.support.base;
    REQUIRE(base.size() == 5);
    CHECK(contiguous(base, scene.top->bottom_z));
    CHECK(scene.top->bottom_z == scale_to_layer_coord(0.6));
    CHECK(scene.top->height == scale_to_layer_coord(0.1));
    CHECK(dense_within_bounds(base));
}

// Without a splitter the grid spaces the middle uniformly: the dense slabs keep the interface step, the sparse slabs stay within the base step.
TEST_CASE("Grid spacing without a splitter keeps the uniform middle", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(3.));
    SC::place_intermediate_slabs(spacing(3), {}, scene.support);
    const SC::GeneratorLayers &base = scene.support.base;
    CHECK(contiguous(base, scene.top->bottom_z));
    CHECK(scene.top->bottom_z == scale_to_layer_coord(2.9));
    CHECK(scene.top->height == scale_to_layer_coord(0.1));
    for (const SC::GeneratorLayer *slab : base) {
        CHECK(slab->height >= scale_to_layer_coord(0.05));
        CHECK(slab->height <= scale_to_layer_coord(0.3));
    }
    CHECK(base[1]->height == scale_to_layer_coord(0.1));
    CHECK(base[2]->height == scale_to_layer_coord(0.1));
}

// A top contact at 0.72 mm leaves a 0.02-mm middle between the stacks, thinner than the minimum height: the interval is split
// evenly into dense slabs instead, and the splitter is not called.
TEST_CASE("Handed-over middle thinner than min_height gives way to even dense slabs", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(0.72));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, scene.support, recording(scene.support, spans));
    CHECK(spans.empty());
    const SC::GeneratorLayers &base = scene.support.base;
    REQUIRE(base.size() == 6);
    CHECK(scene.top->print_z == scale_to_layer_coord(0.72));
    CHECK(scene.top->bottom_z + scene.top->height == scale_to_layer_coord(0.72));
    CHECK(scene.top->bottom_z >= scale_to_layer_coord(0.2));
    CHECK(scene.top->height >= scale_to_layer_coord(0.05));
    CHECK(scene.top->height <= scale_to_layer_coord(0.1));
    CHECK(contiguous(base, scene.top->bottom_z));
    CHECK(dense_within_bounds(base));
}

// A middle of exactly min_height is still handed to the splitter, with the dense stacks at their full step around it.
TEST_CASE("Handed-over middle of exactly min_height is given to the splitter", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(0.75));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, scene.support, recording(scene.support, spans));
    REQUIRE(spans.size() == 1);
    CHECK(spans.front() == std::make_pair(scale_to_layer_coord(0.4), scale_to_layer_coord(0.45)));
    CHECK(scene.top->bottom_z == scale_to_layer_coord(0.65));
    CHECK(scene.top->height == scale_to_layer_coord(0.1));
    CHECK(contiguous(scene.support.base, scene.top->bottom_z));
    CHECK(dense_within_bounds(scene.support.base));
}

// Without interface layers a contact on a 0.07-mm interval must stay above the first layer: the whole interval becomes its thickness.
TEST_CASE("Without interface layers a short interval keeps its contact inside it", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(0.27));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(0), {}, scene.support, recording(scene.support, spans));
    CHECK(spans.empty());
    REQUIRE(scene.support.base.size() == 1);
    CHECK(scene.top->bottom_z == scale_to_layer_coord(0.2));
    CHECK(scene.top->height == scale_to_layer_coord(0.07));
    CHECK(scene.top->height >= scale_to_layer_coord(0.05));
    CHECK(contiguous(scene.support.base, scene.top->bottom_z));
}

// Without interface layers the contact is one dense slab, and the splitter receives the rest of the interval above the first layer.
TEST_CASE("Without interface layers a long interval hands its middle over", "[plugins][support-grid][support-placement]")
{
    Scene scene(scale_to_layer_coord(0.45));
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(0), {}, scene.support, recording(scene.support, spans));
    REQUIRE(spans.size() == 1);
    CHECK(spans.front() == std::make_pair(scale_to_layer_coord(0.2), scale_to_layer_coord(0.35)));
    CHECK(scene.top->height == scale_to_layer_coord(0.1));
    CHECK(scene.top->bottom_z == scale_to_layer_coord(0.35));
    CHECK(contiguous(scene.support.base, scene.top->bottom_z));
}

// A top contact whose thickness is already set keeps its span as one slab, and the interval above it starts at its top.
TEST_CASE("Handed-over interval above a top contact starts at its top", "[plugins][support-grid][support-placement]")
{
    PluginStorage storage;
    SC::GeneratedSupport support(reinterpret_cast<storage_handle *>(&storage));
    SC::GeneratorLayer &lower = SC::add_layer(support, SC::LayerType::TopContact, scale_to_layer_coord(0.8), scale_to_layer_coord(0.6), scale_to_layer_coord(0.2));
    support.top.push_back(&lower);
    SC::GeneratorLayer &upper = SC::add_layer(support, SC::LayerType::TopContact, scale_to_layer_coord(2.), scale_to_layer_coord(2.), 0);
    support.top.push_back(&upper);
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, support, recording(support, spans));
    REQUIRE(spans.size() == 1);
    CHECK(spans.front() == std::make_pair(scale_to_layer_coord(0.9), scale_to_layer_coord(1.7)));
    CHECK(upper.height == scale_to_layer_coord(0.1));
    CHECK(upper.bottom_z == scale_to_layer_coord(1.9));
    CHECK(slabs_at(support.base, scale_to_layer_coord(0.8)) == 1);
    bool covered = false;
    for (const SC::GeneratorLayer *slab : support.base)
        covered = covered || (slab->bottom_z == scale_to_layer_coord(0.6) && slab->print_z == scale_to_layer_coord(0.8));
    CHECK(covered);
    CHECK(contiguous(support.base, upper.bottom_z));
}

// Two synthetic contacts—a bed contact and an upper contact of unknown thickness—are placed without a splitter; the bed span appears once and the interval above follows the native distribution.
TEST_CASE("Grid interval above a bed contact does not double the first layer", "[plugins][support-grid][support-placement]")
{
    PluginStorage storage;
    SC::GeneratedSupport support(reinterpret_cast<storage_handle *>(&storage));
    const coord_t first = scale_to_layer_coord(0.2);
    SC::GeneratorLayer &bed = SC::add_layer(support, SC::LayerType::TopContact, first, 0, first);
    support.top.push_back(&bed);
    SC::GeneratorLayer &upper = SC::add_layer(support, SC::LayerType::TopContact, scale_to_layer_coord(0.9), scale_to_layer_coord(0.9), 0);
    support.top.push_back(&upper);
    SC::place_intermediate_slabs(spacing(3), {}, support);
    const SC::GeneratorLayers &base = support.base;
    REQUIRE(base.size() == 6);
    const double boundaries[] = {0., 0.2, 0.3, 0.4, 0.6, 0.7, 0.8};
    for (size_t i = 0; i < base.size(); ++i) {
        CAPTURE(i);
        CHECK(base[i]->bottom_z == scale_to_layer_coord(boundaries[i]));
        CHECK(base[i]->print_z == scale_to_layer_coord(boundaries[i + 1]));
    }
    CHECK(base.front()->bottom_z == 0);
    CHECK(base.front()->print_z == first);
    CHECK(slabs_at(base, first) == 1);
    CHECK(base[1]->bottom_z == first);
    CHECK(contiguous(base, upper.bottom_z));
}

// A top contact on the first layer already covers that layer with the first-layer slab: the interval above it must not add a second copy.
TEST_CASE("Handed-over interval above a bed contact does not double the first layer", "[plugins][support-grid][support-placement]")
{
    PluginStorage storage;
    SC::GeneratedSupport support(reinterpret_cast<storage_handle *>(&storage));
    SC::GeneratorLayer &bed = SC::add_layer(support, SC::LayerType::TopContact, scale_to_layer_coord(0.2), 0, scale_to_layer_coord(0.2));
    support.top.push_back(&bed);
    SC::GeneratorLayer &upper = SC::add_layer(support, SC::LayerType::TopContact, scale_to_layer_coord(0.9), scale_to_layer_coord(0.9), 0);
    support.top.push_back(&upper);
    std::vector<std::pair<coord_t, coord_t>> spans;
    SC::place_intermediate_slabs(spacing(3), {}, support, recording(support, spans));
    REQUIRE(spans.size() == 1);
    CHECK(spans.front() == std::make_pair(scale_to_layer_coord(0.4), scale_to_layer_coord(0.6)));
    CHECK(slabs_at(support.base, scale_to_layer_coord(0.2)) == 1);
    CHECK(upper.bottom_z == scale_to_layer_coord(0.8));
    CHECK(contiguous(support.base, upper.bottom_z));
}
