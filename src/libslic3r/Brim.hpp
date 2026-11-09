///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2021 Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Brim_hpp_
#define slic3r_Brim_hpp_

#include <cstdint>
#include <vector>

#include "ExPolygon.hpp"
#include "Polygon.hpp"
#include "Polyline.hpp"
#include "PrintBase.hpp"

/* Supply brim geometry to adhesion generators. Resolve BrimGenerationParameters
   and Flow, then call the outer, ear, patch or interior builder with valid
   first-layer objects, an occupied area and an owned Children destination.
   Builders append generic extrusion nodes and extend the occupied area, whose
   order must be retained across calls. They do not publish auxiliary layers.
   BrimLoop describes explicit contour nesting for the internal extrusion pass.
*/

namespace Slic3r {
    
class ExtrusionEntity;

enum class BrimEarPattern : uint8_t
{
    Concentric,
    Rectilinear
};

/*
Geometry controls supplied by the brim generator.

The core brim routines receive resolved values instead of looking up plugin
configuration keys themselves. This keeps Brim.cpp usable by another generator
while the plugin remains responsible for defining, reading and validating its
settings.
*/
struct BrimGenerationParameters
{
    bool fill_enclosed_holes = false;
    double ear_max_angle_degrees = 125.0;
    double ear_detection_length_mm = 1.0;
    BrimEarPattern ear_pattern = BrimEarPattern::Concentric;
};

class Flow;
class Print;
class PrintObject;
using PrintObjectPtrs = std::vector<PrintObject*>;
class ExtrusionEntityCollection;

class BrimLoop {
public:
    BrimLoop(const Polygon& p) : lines(Polylines{ p.split_at_first_point() }), is_loop(true) {}
    BrimLoop(const Polyline& l) : lines(Polylines{l}), is_loop(false) {}
    Polylines lines;
    std::vector<BrimLoop> children;
    bool is_loop; // has only one polyline stored and front == back
    Polygon polygon() const{
        assert(is_loop);
        assert(lines.size() == 1);
        assert(lines.front().points.front() == lines.front().points.back());
        Polygon poly = Polygon(lines.front().points);
        return poly;
    }
};

#if 0
// Produce brim lines around those objects, that have the brim enabled.
// Collect islands_area to be merged into the final 1st layer convex hull.
ExtrusionEntityCollection make_brim(const Print &print, PrintTryCancel try_cancel, Polygons &islands_area);
#endif
// Append outer brim to a borrowed Children destination for valid objects with first layers; retain forbidden-area order.
// Output and forbidden areas require exclusive access; cancellation or allocation failure can leave partial changes.
void make_brim(const Print& print, const Flow& flow, const PrintObjectPtrs& objects, const BrimGenerationParameters &parameters, ExPolygons& unbrimmable, ExtrusionEntity& out);
// Append corner ears and extend forbidden areas under the same exclusive-access and partial-failure contract.
void make_brim_ears(const Print& print, const Flow& flow, const PrintObjectPtrs& objects, const BrimGenerationParameters &parameters, ExPolygons& unbrimmable, ExtrusionEntity& out);
// Append brim around patches to an exclusively borrowed Children destination and extend forbidden areas; failures propagate after any earlier changes.
void make_brim_patch(const Print &print, const Flow &flow, const Polygons &patches, ExPolygons &unbrimmable_areas, ExtrusionEntity &out);
// Append inner brim for valid objects with first layers and extend forbidden areas under the same exclusive-access and partial-failure contract.
void make_brim_interior(const Print& print, const Flow& flow, const PrintObjectPtrs& objects, ExPolygons& unbrimmable_areas, ExtrusionEntity& out);

} // Slic3r

#endif // slic3r_Brim_hpp_
