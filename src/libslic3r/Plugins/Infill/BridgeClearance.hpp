///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgeClearance_hpp_
#define slic3r_BridgeClearance_hpp_
/* Measure the space below a bridge path and choose smaller thread cross sections.
   BridgeVolume.cpp uses these helpers to reduce bridge over-extrusion.

   Add lower model/support paths to an Index using their XY widths and bottom/top Z.
   clearance() returns the vertical gap along a straight bridge path; contacts()
   finds lower paths touching its centerline. Pass the gaps to section_pieces()
   to choose the amount extruded on each part of that path. Before native path
   splitting, coalesce_short_sections() absorbs known sub-grid pieces into a
   smaller neighbor without changing that neighbor's flow.

   All lengths are in mm and cross-sectional areas in mm2. The index copies its
   inputs and returned vectors own their values. A model slice or a support request
   is not a printed path, so neither belongs in this index.

   Example, in namespace slic3r_api::BridgeClearance:
       Index index;
       index.add({{0., 0.}, {5., 0.}, .2, 0., .2, false});
       const std::vector<Interval> gaps = index.clearance({0., 0.}, {5., 0.}, .4, .4, .4);
       const std::vector<SectionPiece> pieces = section_pieces(gaps, .4, .05);
*/
#include <memory>
#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_bridge.h"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
struct plugin_run_context;

namespace slic3r_api { namespace BridgeClearance {
// Space filled by a lower extrusion: thicken a-b by radius in XY, with round ends,
// and fill from z_low to z_high. unknown means its exact occupied space is not known.
struct Occupation {
    c_vec2d a, b;
    double radius, z_low, z_high;
    bool unknown=false;
};
// One part of the bridge path, measured from its first endpoint. clearance is the
// gap down to lower material, or +infinity if none limits it; unknown means no proven gap.
struct Interval {
    double begin, end, clearance;
    bool unknown=false;
};
// One part printed with a constant cross section. height is the chosen thread
// height; area is its cross section and cap is the largest area that fits the gap.
// Lengths are mm, areas mm2; confidence (BRIDGE_CONFIDENCE_*) says whether it fits.
struct SectionPiece {
    double begin, end, area, height, clearance, cap;
    uint8_t confidence;
};
class Index {
public:
    // Borrow the run's cancellation context until this index is destroyed.
    explicit Index(const plugin_run_context *context=nullptr);
    // Release the stored lower paths and their lookup structure, not the run context.
    ~Index();
    // Copying is disabled because each index owns its footprint lookup structure.
    Index(const Index &)=delete;
    // Assignment cannot replace another index's owned footprint lookup structure.
    Index &operator=(const Index &)=delete;
    // Record one lower path's occupied space; reject non-positive radius or non-finite Z.
    // Cancellation and allocation failures propagate without changing the input.
    void add(const Occupation &occupation);
    // Return the gap below each part of a-b, accounting for the bridge width.
    // Only lower material reaching into [z-height, z] matters; cancellation and allocation throw.
    std::vector<Interval> clearance(c_vec2d a,c_vec2d b,double width,double height,double z) const;
    // Find where known lower paths reach the bridge centerline within its height.
    // Unlike clearance(), do not widen the query by the bridge's own width.
    std::vector<std::pair<double,double>> contacts(c_vec2d a,c_vec2d b,double height,double z) const;
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
// Return the thread area in mm2: a rectangle with semicircular ends, keeping the
// total width fixed as height decreases. Requires 0 <= height <= width.
double area(double width,double height);
// Recover height in mm from that cross-sectional area; requires an area between
// zero and the circle of diameter width, and a positive width.
double height_for_area(double width,double area);
// Reduce a round thread's height where the gap is too small, keeping its width.
// floor is the minimum printable height. If it exceeds the gap, keep that height
// and mark the overlap FloorLimited. For known gaps, spread changes over at most one diameter
// with area steps no larger than 5% of the original circle's area.
// Requires adjacent intervals and 0 < floor <= diameter; invalid heights throw.
// Unknown gaps keep the original circle with no fit guarantee. A caller whose
// incoming flow is not circular must restore that flow on these Unknown parts.
std::vector<SectionPiece> section_pieces(const std::vector<Interval> &intervals,double diameter,double floor);
// Merge sub-resolution known sections into a neighboring section with no greater
// area or height, without changing that neighbor's flow. Keep the strictest cap.
// Unmergeable minima and Unknown sections remain for the caller to reject or handle.
// Input pieces must be adjacent and minimum_length is in mm; the result owns its values.
std::vector<SectionPiece> coalesce_short_sections(const std::vector<SectionPiece> &pieces,double minimum_length);
}}
#endif
