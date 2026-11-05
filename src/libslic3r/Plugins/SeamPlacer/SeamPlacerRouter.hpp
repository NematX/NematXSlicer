///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_SeamPlacerRouter_hpp_
#define slic3r_SeamPlacerRouter_hpp_

#include <memory>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

struct orchestrator_handle;

namespace slic3r_api { namespace SeamPlacement {

/*
Regional seam-provider router used by extrusion-tree ordering plugins.

Construction reads every PrintRegion selector, creates one shared session per
selected provider, and resolves every atomic plan loop that carries a perimeter
marker (an EXTRUSION_PROPERTY_TYPE_PERIMETER property on the loop, an ancestor
or a descendant) to one LayerRegion. Other closed loops get no session, whatever
the selected provider, and place_seam() projects the approach position onto
them. The resulting tables are immutable, so
place_seam() may be called concurrently by ordering workers. The Print,
PrintingPlan, regions, and loop handles are all borrowed and must outlive the
router.
*/
class SeamPlacerRouter
{
public:
    /*
    Construct the complete routing table before ordering workers may request
    seams. Provider initialization, region selection, and loop-routing errors
    are therefore reported synchronously during construction.
    */
    SeamPlacerRouter(orchestrator_handle *orchestrator,
                     const Print &print,
                     PrintingPlan plan,
                     storage_handle *scratch_storage);
    /* Release the immutable route table and its shared provider sessions. */
    ~SeamPlacerRouter();

    /* Immutable identities selected on input, including routes without a provider. */
    bool has_loop_route(const ExtrusionEntity &entity) const;

    /* Before skipping an empty borrowed subtree, reject any initially routed
       unit it still contains. No geometry, flags or provider state is mutated. */
    void validate_empty_subtree(const ExtrusionEntity &entity) const;

    SeamPlacerRouter(const SeamPlacerRouter &) = delete;
    SeamPlacerRouter &operator=(const SeamPlacerRouter &) = delete;

    /*
    Ask the provider preselected for a marked loop to choose its final seam. A
    loop without a marker receives the point of the loop nearest to
    start_position. loop is borrowed for the call and start_position is a value.
    */
    c_point place_seam(const ExtrusionEntity &loop,
                       c_point start_position) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}} // namespace slic3r_api::SeamPlacement

#endif // slic3r_SeamPlacerRouter_hpp_
