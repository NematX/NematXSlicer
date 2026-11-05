/* Native support keeps its temporary slabs inside generate(), then destroys
   them after path generation. These tests run its geometry phases themselves, once, then
   finish with the same layer installation and toolpath generation, to inspect the slab
   thicknesses before paths can be split by overlapping slabs.
   Explicit member-pointer instantiation gives this test access to seven private
   phases without changing native source or redefining its class. The deque
   owns the slabs and the vectors point into it for the duration of a comparison. */
#ifndef support_grid_legacy_slabs_hpp_
#define support_grid_legacy_slabs_hpp_
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Support/SupportMaterial.hpp"

namespace Slic3r::Test::SupportGrid {
using NativeLayers = FFFSupport::SupportGeneratorLayersPtr;
using NativeStorage = FFFSupport::SupportGeneratorLayerStorage;
template<class Tag, typename Tag::Type Member> struct NativeStage {
    // Return the selected native geometry phase's member pointer for this test.
    friend typename Tag::Type native_stage(Tag) { return Member; }
};
struct TopStage {
    using Type = NativeLayers (PrintObjectSupportMaterial::*)(const PrintObject &, const std::vector<Polygons> &, NativeStorage &) const;
    // Select native top-contact geometry generation.
    friend Type native_stage(TopStage);
};
struct BottomStage {
    using Type = NativeLayers (PrintObjectSupportMaterial::*)(const PrintObject &, const NativeLayers &, std::vector<Polygons> &, NativeStorage &, std::vector<Polygons> &) const;
    // Select native bottom-contact geometry generation.
    friend Type native_stage(BottomStage);
};
struct IntermediateStage {
    using Type = NativeLayers (PrintObjectSupportMaterial::*)(const PrintObject &, const NativeLayers &, const NativeLayers &, NativeStorage &) const;
    // Select native intermediate support slab allocation.
    friend Type native_stage(IntermediateStage);
};
struct TrimStage {
    using Type = void (PrintObjectSupportMaterial::*)(const PrintObject &, NativeLayers &, coord_t, coord_t, coord_t) const;
    // Select native object-clearance trimming.
    friend Type native_stage(TrimStage);
};
struct BaseStage {
    using Type = void (PrintObjectSupportMaterial::*)(const PrintObject &, const NativeLayers &, const NativeLayers &, NativeLayers &, const std::vector<Polygons> &) const;
    // Select native sparse support footprint generation.
    friend Type native_stage(BaseStage);
};
struct TrimContactStage {
    using Type = void (PrintObjectSupportMaterial::*)(const PrintObject &, const NativeLayers &, NativeLayers &) const;
    // Select native contact-overlap trimming.
    friend Type native_stage(TrimContactStage);
};
struct CoveredStage {
    using Type = std::vector<Polygons> (PrintObjectSupportMaterial::*)(const PrintObject &) const;
    // Select the native build-plate coverage used by buildplate-only support.
    friend Type native_stage(CoveredStage);
};
template struct NativeStage<CoveredStage, &PrintObjectSupportMaterial::buildplate_covered>;
template struct NativeStage<TopStage, &PrintObjectSupportMaterial::top_contact_layers>;
template struct NativeStage<BottomStage, &PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas>;
template struct NativeStage<IntermediateStage, &PrintObjectSupportMaterial::raft_and_intermediate_support_layers>;
template struct NativeStage<TrimStage, &PrintObjectSupportMaterial::trim_auxiliary_layers_by_object>;
template struct NativeStage<BaseStage, &PrintObjectSupportMaterial::generate_base_layers>;
template struct NativeStage<TrimContactStage, &PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts>;

struct NativeSlabs {
    NativeStorage storage;
    // The slabs. Layer installation and toolpath generation consume them (print Z grouping, merging of slabs of one
    // support layer, cleared overhang polygons): read them only for what native_slabs() does not record in heights.
    NativeLayers top, bottom, base, interfaces, raft;
    // The (print Z, height) of the non-empty top, bottom, base and interface slabs, in this order, recorded after the
    // geometry phases and before layer installation: the printable thicknesses of the original slabs.
    std::vector<std::pair<coord_t, coord_t>> heights;
};

// Run the native generate() once, phase by phase as PrintObjectSupportMaterial::generate() does (SupportMaterial.cpp),
// recording the original slab heights for the comparison: install the support layers into the object (which must have none)
// and generate their toolpaths from the same slabs. Like generate(), nothing is installed when there is no top contact; the
// Debug checks of generate() are repeated here.
inline void native_slabs(PrintObject &object, const PrintObjectSupportMaterial &generator, NativeSlabs &result)
{
    const FFFSupport::SupportParameters params(object);
    const SlicingParameters &slicing = object.slicing_parameters();
    // Same checks as the assert_check and assert_z of generate(), made when _DEBUG is defined.
    const auto assert_check = [](const NativeLayers &layers) {
#ifdef _DEBUG
        for (const FFFSupport::SupportGeneratorLayer *layer : layers)
            assert_valid(layer->polygons);
#else
        (void)layers;
#endif
    };
    const auto assert_z = [](const NativeLayers &layers) {
#ifdef _DEBUG
        for (const FFFSupport::SupportGeneratorLayer *layer : layers) {
            assert(layer->scaled_print_z() > layer->scaled_bottom_z());
            assert(layer->scaled_print_z() >= layer->scaled_bottom_z() + layer->scaled_height());
        }
#else
        (void)layers;
#endif
    };
    // The comparison models never request a raft. The build-plate coverage is computed as the native
    // generate() does; it is empty unless buildplate-only support is requested.
    REQUIRE(object.config().raft_layers.value == 0);
    std::vector<Polygons> covered = (generator.*native_stage(CoveredStage{}))(object), areas;
    result.top = (generator.*native_stage(TopStage{}))(object, covered, result.storage);
    // generate() returns here: nothing is supported, no support layer is installed.
    if (result.top.empty())
        return;
    assert_check(result.top);
    result.bottom = (generator.*native_stage(BottomStage{}))(object, result.top, covered, result.storage, areas);
    assert_check(result.bottom);
    assert_z(result.bottom);
    result.base = (generator.*native_stage(IntermediateStage{}))(object, result.bottom, result.top, result.storage);
    assert_check(result.base);
    assert_z(result.base);
    (generator.*native_stage(TrimStage{}))(object, result.top, scale_to_layer_coord(slicing.gap_support_object),
        scale_to_layer_coord(slicing.gap_object_support), params._gap_xy);
    assert_check(result.top);
    (generator.*native_stage(BaseStage{}))(object, result.bottom, result.top, result.base, areas);
    assert_check(result.bottom);
    assert_check(result.top);
    assert_check(result.base);
    (generator.*native_stage(TrimContactStage{}))(object, result.bottom, result.top);
    assert_check(result.bottom);
    assert_check(result.top);
    assert_z(result.bottom);
    assert_z(result.top);
    NativeLayers empty;
    std::pair<NativeLayers, NativeLayers> interfaces = FFFSupport::generate_interface_layers(object.config(), params,
        result.bottom, result.top, empty, empty, result.base, result.storage);
    result.interfaces = std::move(interfaces.first);
    REQUIRE(interfaces.second.empty());
    assert_check(result.bottom);
    assert_check(result.top);
    assert_check(result.base);
    assert_check(result.interfaces);
    assert_check(interfaces.second);
    assert_z(result.bottom);
    assert_z(result.top);
    assert_z(result.base);
    assert_z(result.interfaces);
    assert_z(interfaces.second);
    result.raft = FFFSupport::generate_raft_base(object, params, slicing, result.top, result.interfaces, interfaces.second, result.base, result.storage);
    assert_check(result.raft);
    assert_check(result.bottom);
    assert_check(result.top);
    assert_check(result.base);
    assert_check(result.interfaces);
    assert_check(interfaces.second);
    // The thicknesses of the slabs as the geometry phases made them: generate_support_layers() and the toolpaths change them.
    for (const NativeLayers *layers : {&result.top, &result.bottom, &result.base, &result.interfaces})
        for (const FFFSupport::SupportGeneratorLayer *slab : *layers)
            if (!slab->polygons.empty())
                result.heights.emplace_back(slab->scaled_print_z(), slab->scaled_height());
    FFFSupport::generate_support_layers(object, result.raft, result.bottom, result.top, result.base, result.interfaces, interfaces.second);
    // generate() checks all six lists again before the toolpaths.
    assert_check(result.raft);
    assert_check(result.bottom);
    assert_check(result.top);
    assert_check(result.base);
    assert_check(result.interfaces);
    assert_check(interfaces.second);
    FFFSupport::generate_support_toolpaths(object, object.mutable_auxiliary_layers(), object.config(), params, slicing,
        result.raft, result.bottom, result.top, result.base, result.interfaces, interfaces.second);
}
}
#endif
