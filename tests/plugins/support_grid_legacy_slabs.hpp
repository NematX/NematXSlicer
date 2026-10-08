/* Native support keeps its temporary slabs inside generate(), then destroys
   them after path generation. These tests also run its geometry phases to
   inspect the slab thicknesses before paths can be split by overlapping slabs.
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
    NativeLayers top, bottom, base, interfaces;
};

// Run native non-raft geometry phases and retain their original slab heights for comparison.
inline void native_slabs(const PrintObject &object, const PrintObjectSupportMaterial &generator, NativeSlabs &result)
{
    const FFFSupport::SupportParameters params(object);
    const SlicingParameters &slicing = object.slicing_parameters();
    // The comparison models never request a raft. The build-plate coverage is computed as the native
    // generate() does; it is empty unless buildplate-only support is requested.
    REQUIRE(object.config().raft_layers.value == 0);
    std::vector<Polygons> covered = (generator.*native_stage(CoveredStage{}))(object), areas;
    result.top = (generator.*native_stage(TopStage{}))(object, covered, result.storage);
    result.bottom = (generator.*native_stage(BottomStage{}))(object, result.top, covered, result.storage, areas);
    result.base = (generator.*native_stage(IntermediateStage{}))(object, result.bottom, result.top, result.storage);
    (generator.*native_stage(TrimStage{}))(object, result.top, scale_to_layer_coord(slicing.gap_support_object),
        scale_to_layer_coord(slicing.gap_object_support), params._gap_xy);
    (generator.*native_stage(BaseStage{}))(object, result.bottom, result.top, result.base, areas);
    (generator.*native_stage(TrimContactStage{}))(object, result.bottom, result.top);
    NativeLayers empty;
    std::pair<NativeLayers, NativeLayers> interfaces = FFFSupport::generate_interface_layers(object.config(), params,
        result.bottom, result.top, empty, empty, result.base, result.storage);
    result.interfaces = std::move(interfaces.first);
    REQUIRE(interfaces.second.empty());
    FFFSupport::generate_raft_base(object, params, slicing, result.top, result.interfaces, empty, result.base, result.storage);
}
}
#endif
