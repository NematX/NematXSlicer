///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Estimate how much bridge material fits below a row for the default finalizer
   in BridgeVolume.cpp. This is a mean volume budget, not a measurement of gaps
   between individual lower extrusion paths or a local collision certificate.

   Index copies the object's material areas and their bottom/top Z. Solid fill
   surfaces and perimeter bands are full; void is empty. Sparse fill uses its
   owning lower region's density, including regional modifiers. DenseInfill's
   sparse bridge surfaces use its 50% recipe. Unclassified fill and auxiliary
   support slices without a density payload remain unknown. When support material
   is unfinished, request masks locate potentially occupied areas, not empty space.
   The copied geometry, densities and Z values also form a signature that lets
   finalization detect changed inputs when a correction is replayed.

   A is the incoming section in mm2, p the actual row pitch in mm, and h the
   current layer height in mm. A/p is the mean deposited height across one row's
   share of the surface. The current layer provides h, leaving a lower-space need
   of max(0, A/p - h). For example, 160% deposition on a 0.2 mm layer needs another
   0.12 mm. A lower 0.1 mm band at 30% density provides 0.07 mm of empty height,
   leaving 0.05 mm to seek farther down.

   sections() descends from the row's bottom Z toward the build plate through
   the actual band boundaries. Clipping the row against each band's material
   areas splits it into portions that accumulate their own empty-height budget:
   (1 - density) * band_height. Each portion stops independently on full material,
   unknown material, enough accumulated space, or the plate. No material is
   credited below the plate, and a solid barrier cannot be searched through.

   Known free height gives an area cap of min(A, p * (h + free_height)). The
   printable height floor can force an area above that cap;
   BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED records this rather than claiming a fit.
   BRIDGE_CONFIDENCE_DENSITY_ESTIMATE means the assigned area fits this mean budget
   only. Unknown portions keep A without a fit claim.
   Adjacent equal results are joined, then known sub-grid sections may be absorbed
   into a smaller neighbor without increasing that neighbor's flow.

   Representative direct calls, not a timeline:

       Index::Index()
           |-- BridgeGeometry::subtract() [perimeter and unclassified areas]
           |-- BridgeGeometry::intersect() [lower region ownership]
           |-- hash_scalar() [density and Z]
           `-- BridgeGeometry::geometry_key() [copied material areas]

       Index::sections()
           |-- intervals() [clip one row against a material area]
           |   `-- clipper_intersection_polyline_expolygons()
           |-- BridgeClearance::area() [printable floor]
           |-- BridgeClearance::height_for_area() [assigned section height]
           `-- BridgeClearance::coalesce_short_sections()

   Row endpoints use scaled integer XY coordinates; heights, pitch and returned
   distances use mm, and sections use mm2. The index owns its copied geometry in
   the supplied plugin storage and borrows the run context for cancellation;
   both must outlive it. Results own their numeric values. Calls on one index are
   sequential, while different objects use independent indexes.
*/
#include "BridgeSurfaceVolume.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include "libslic3r/Plugins/Surface/BridgeGeometry.hpp"

namespace slic3r_api { namespace BridgeSurfaceVolume {
namespace {
using namespace BridgeGeometry;
// One lower material interval along a row; -1 denotes an unknown density.
struct Occupied { double begin,end,density; };
// Clip a borrowed row against the areas and return distances from a in mm.
std::vector<Occupied> intervals(storage_handle *storage,c_point a,c_point b,const ExPolygonCollection &areas,double density);
// Hash a finite scalar without padding, retaining exact replay sensitivity.
uint64_t hash_scalar(uint64_t signature,double value);
}

Index::Index(const Object &object,const run_ctx_finalize_slicing &context,const plugin_run_context *run)
    : m_storage(run->plugin_storage),m_run(run)
{
    for (uint32_t idx=0;idx<object.layer_count();++idx) {
        throw_if_cancelled(run);
        const Layer layer=object.layer(idx);
        Band band{unscaled(layer.print_z()-layer.height()),unscaled(layer.print_z()),{}};
        for (uint32_t i=0;i<layer.island_count();++i) {
            const LayerIsland island=layer.island(i);
            StoredExPolygonCollection slice(m_storage);slice.push_back(island.slice());
            StoredExPolygonCollection perimeters=subtract(m_storage,slice.readonly(),island.infill_areas());
            band.domains.push_back({std::move(perimeters),1.});
            StoredExPolygonCollection covered(m_storage);
            for (uint32_t j=0;j<island.region_island_count();++j) {
                const LayerRegionIsland group=island.region_island(j);
                for (const Surface surface : group.fill_surfaces_collection()) {
                    StoredExPolygonCollection area(m_storage);area.push_back(surface.expolygon());
                    if (surface_type_is_solid(surface.type()) || surface_type_is_void(surface.type())) {
                        covered.append_copy_from(area.readonly());
                        band.domains.push_back({std::move(area),surface_type_is_solid(surface.type()) ? 1. : 0.});
                        continue;
                    }
                    // Regional modifiers belong to the lower surface, not the bridge.
                    // DenseInfill's sparse bridge recipe is explicitly 50%, not solid.
                    for (const LayerRegion owner : group.regions()) {
                        StoredExPolygonCollection part=intersect(m_storage,area.readonly(),owner.slices());
                        if (part.empty()) continue;
                        const double density=surface_type_is_bridge(surface.type()) ? .5 :
                            owner.print_region().config().get("fill_density").get_effective_value(1.);
                        if (!std::isfinite(density) || density<0. || density>1.)
                            throw std::runtime_error("A lower surface has an invalid infill density.");
                        covered.append_copy_from(part.readonly());band.domains.push_back({std::move(part),density});
                    }
                }
            }
            StoredExPolygonCollection missing=subtract(m_storage,island.infill_areas(),covered.readonly());
            if (!missing.empty()) band.domains.push_back({std::move(missing),-1.});
            // Without completed material generation, a request locates possible
            // support but contributes no volume to the estimate.
            if (!context.support_material_generated) {
                StoredExPolygonCollection requested=ExPolygonCollection(context.get_requested_support_areas(island.handle())).clone(m_storage);
                if (!requested.empty()) {
                    Band unknown{0.,band.bottom,{}};unknown.domains.push_back({std::move(requested),-1.});m_bands.push_back(std::move(unknown));
                }
            }
        }
        m_bands.push_back(std::move(band));
    }
    for (uint32_t idx=0;idx<object.auxiliary_layer_count();++idx) {
        const Layer layer=object.auxiliary_layer(idx);
        if (!layer.properties().has(LayerSupportProperty::key)) continue;
        Band band{unscaled(layer.print_z()-layer.height()),unscaled(layer.print_z()),{}};
        for (uint32_t i=0;i<layer.island_count();++i) {
            // Support surface types describe roles, not a guaranteed material
            // fraction. No density payload exists here, so keep these areas unknown.
            StoredExPolygonCollection area(m_storage);area.push_back(layer.island(i).slice());
            band.domains.push_back({std::move(area),-1.});
        }
        m_bands.push_back(std::move(band));
    }
    for (const Band &band : m_bands) {
        m_signature=hash_scalar(hash_scalar(m_signature,band.bottom),band.top);
        for (const Domain &domain : band.domains) {
            m_signature=hash_scalar(m_signature,domain.density);
            m_signature=(m_signature^geometry_key(domain.areas.readonly()))*1099511628211ull;
        }
    }
}

std::vector<BridgeClearance::SectionPiece> Index::sections(c_point a,c_point b,double bottom_z,
    double layer_height,double nominal_area,double pitch,double width,double floor) const
{
    // Use the layer grid so float roundoff cannot include the bridge's own band.
    bottom_z=unscaled(scale_i(bottom_z));
    const double length=unscaled(norm(b-a)),needed=std::max(0.,nominal_area/pitch-layer_height);
    if (!(length>0.) || !(pitch>0.) || !(layer_height>0.) || !std::isfinite(needed))
        throw std::runtime_error("A surface-volume bridge has invalid row spacing or layer height.");
    struct Span { double begin,end,free; bool stopped,unknown; };
    std::vector<Span> spans{{0.,length,0.,needed==0.,false}};
    std::vector<double> levels{0.,std::max(0.,bottom_z)};
    for (const Band &band : m_bands)
        for (const double z : {band.bottom,band.top}) if (z>0. && z<bottom_z) levels.push_back(z);
    std::sort(levels.begin(),levels.end(),std::greater<double>());
    levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
    for (size_t level=1;level<levels.size();++level) {
        throw_if_cancelled(m_run);
        if (std::all_of(spans.begin(),spans.end(),[](const Span &span) { return span.stopped; })) break;
        const double top=levels[level-1],bottom=levels[level];
        std::vector<Occupied> material;
        for (const Band &band : m_bands) if (band.bottom<top && band.top>bottom)
            for (const Domain &domain : band.domains) {
                if (domain.density==0.) continue;
                std::vector<Occupied> parts=intervals(m_storage,a,b,domain.areas.readonly(),domain.density);
                material.insert(material.end(),parts.begin(),parts.end());
            }
        std::vector<Span> next;
        for (const Span &span : spans) {
            if (span.stopped) { next.push_back(span);continue; }
            std::vector<double> cuts{span.begin,span.end};
            for (const Occupied &part : material)
                for (const double at : {part.begin,part.end}) if (at>span.begin && at<span.end) cuts.push_back(at);
            std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
            for (size_t c=1;c<cuts.size();++c) {
                Span part{cuts[c-1],cuts[c],span.free,false,false};
                const double midpoint=(part.begin+part.end)*.5;
                double density=0.;
                for (const Occupied &o : material) if (midpoint>=o.begin && midpoint<o.end) {
                    part.unknown=part.unknown || o.density<0.;density=std::max(density,o.density);
                }
                part.unknown=part.unknown && density<1.;
                part.stopped=part.unknown || density>=1.;
                if (!part.stopped) {
                    part.free=std::min(needed,part.free+(1.-density)*(top-bottom));
                    part.stopped=part.free>=needed;
                }
                next.push_back(part);
            }
        }
        spans=std::move(next);
    }
    std::vector<BridgeClearance::SectionPiece> pieces;
    for (const Span &span : spans) {
        const double cap=std::min(nominal_area,pitch*(layer_height+span.free));
        const double area=span.unknown ? nominal_area : std::min(nominal_area,std::max(cap,BridgeClearance::area(width,std::min(width,floor))));
        const uint8_t confidence=span.unknown ? BRIDGE_CONFIDENCE_UNKNOWN :
            area>cap+1e-10 ? BRIDGE_CONFIDENCE_DENSITY_FLOOR_LIMITED : BRIDGE_CONFIDENCE_DENSITY_ESTIMATE;
        BridgeClearance::SectionPiece piece{span.begin,span.end,area,BridgeClearance::height_for_area(width,area),
            layer_height+span.free,cap,confidence};
        if (!pieces.empty() && pieces.back().end==piece.begin && pieces.back().area==piece.area && pieces.back().confidence==piece.confidence) {
            pieces.back().end=piece.end;pieces.back().cap=std::min(pieces.back().cap,piece.cap);
            pieces.back().clearance=std::min(pieces.back().clearance,piece.clearance);
        } else pieces.push_back(piece);
    }
    return BridgeClearance::coalesce_short_sections(pieces,2.*unscaled(SCALED_EPSILON));
}

namespace {
std::vector<Occupied> intervals(storage_handle *storage,c_point a,c_point b,const ExPolygonCollection &areas,double density)
{
    StoredPolyline line(storage);line.push_back(a);line.push_back(b);
    const double length=unscaled(norm(b-a)),ux=unscaled(b.x-a.x)/length,uy=unscaled(b.y-a.y)/length;
    const StoredPolylineCollection clipped=clipper_intersection_polyline_expolygons(storage,line.readonly(),areas);
    std::vector<Occupied> result;
    for (const Polyline part : clipped.readonly()) {
        double begin=length,end=0.;
        for (const c_point point : part.points()) {
            const double at=std::clamp(unscaled(double(point.x-a.x)*ux+double(point.y-a.y)*uy),0.,length);
            begin=std::min(begin,at);end=std::max(end,at);
        }
        if (end>begin) result.push_back({begin,end,density});
    }
    return result;
}
uint64_t hash_scalar(uint64_t signature,double value)
{
    uint64_t bits=0;std::memcpy(&bits,&value,sizeof(bits));return (signature^bits)*1099511628211ull;
}
}
}}
