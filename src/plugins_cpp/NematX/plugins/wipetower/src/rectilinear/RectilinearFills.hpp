///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtech Bubnik @bubnikv, Lukas Hejl @hejllukas, Lukas Matena @lukasmatena
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef nematx_wipetower_copy_rectilinear_fills_hpp_
#define nematx_wipetower_copy_rectilinear_fills_hpp_

/*
Rectilinear fill patterns for the plugin port
=============================================

Shared raster and fill declarations used by the rectilinear pattern family.
The implementation uses private mutable work buffers and borrowed boundary
views provided by the plugin connect helpers. Sawtooth reuses FillRectilinear
and publishes its Z-profile separately; rectiwithperimeter wraps the base
filler, and supportbase overrides the boundary connection for sparse support
paths. The canonical pattern-id list is declared by RectilinearPatterns.hpp.
*/

#include <cassert>
#include <cmath>
#include <utility>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

#include "ClipperHelpers.hpp"
#include "RectilinearConnect.hpp"

namespace slic3r_api { namespace NematXWipeTowerCopy { namespace RectilinearPatternsPlugin {
namespace fills {

using namespace work;
using namespace clipper;

// Connect generated lines along the boundary through the plugin dispatchers.
using namespace connect;
// Select mutable work types only in this namespace; RectilinearWithPerimeter.cpp uses API views with the same type names.
using work::ExPolygon;
using work::Polygon;
using work::Polyline;

// The ported code calls init_spacing and _infill_direction on this base class.

// Implementation details of the ported algorithm, defined in the .cpp.
struct SegmentedIntersectionLine;
struct ExPolygonWithOffset;

// A layer-wide sampling frame keeps points on separate components at least spacing apart.
Points sample_grid_pattern(storage_handle *storage, const slic3r_api::ExPolygon &polygon,
                           coord_t spacing, const BoundingBox &bounds);

class FillRectilinear
{
public:
    FillRectilinear() = default;
    virtual ~FillRectilinear() = default;
    void init_spacing(coordf_t spacing, const raw_infill_pattern_params &params);
    virtual Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const;
    virtual bool no_sort() const { return false; }

    coordf_t get_spacing() const { return this->spacing_priv; }

    size_t      layer_id = size_t(-1);
    double      z = 0.;
    float       angle = 0.f;
    bool        can_angle_cross = false;
    BoundingBox bounding_box;
    coordf_t    overlap = 0.;
    coord_t     link_max_length = 0;
    coord_t     loop_clipping = 0;
    storage_handle *plugin_storage = nullptr;

protected:
    virtual float _layer_angle(size_t idx) const {
        return can_angle_cross && (idx & 1) ? float(M_PI / 2.) : 0.f;
    }

    virtual coord_t _line_spacing_for_density(const raw_infill_pattern_params &params) const {
        assert(get_spacing() >= 0);
        assert(params.density > 0);
        if (params.max_sparse_infill_spacing > 0)
            return scale_i(params.max_sparse_infill_spacing / params.density);
        return scale_i(this->get_spacing() / params.density);
    }

    std::pair<float, Point> _infill_direction(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const {
        float out_angle = this->angle;

        // The bounding box is the object bounding box; it is only undefined in
        // unit tests, then the source contour center is used.
        const c_bounding_box source_bbox = expolygon.contour().bounding_box();
        const Point source_center((source_bbox.min.x + source_bbox.max.x) / 2,
                                  (source_bbox.min.y + source_bbox.max.y) / 2);
        Point out_shift = empty(this->bounding_box) ? source_center : this->bounding_box.center();

        if (params.bridge_angle >= 0) {
            // Use the bridge angle.
            out_angle = float(params.bridge_angle);
        } else if (this->layer_id != size_t(-1)) {
            // Alternate fill direction.
            // Each plugin invocation represents one combined layer.
            out_angle += this->_layer_angle(this->layer_id);
        }

        out_angle += float(M_PI / 2.);
        return std::pair<float, Point>(out_angle, out_shift);
    }

    virtual std::vector<SegmentedIntersectionLine> _vert_lines_for_polygon(const ExPolygonWithOffset &poly_with_offset,
                                                                           const BoundingBox &bounding_box,
                                                                           const raw_infill_pattern_params &params,
                                                                           coord_t line_spacing) const;

    // Fill by single directional lines, interconnect the lines along perimeters.
    bool fill_surface_by_lines(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params, float angleBase, float pattern_shift,
                               Polylines &polylines_out) const;

    // Fill by multiple sweeps of differing directions.
    struct SweepParams
    {
        float angle_base;
        float pattern_shift;
    };
    void make_fill_lines(const ExPolygonWithOffset &poly_with_offset, Point refpt, double angle, coord_t x_margin,
                         coord_t line_spacing, coord_t pattern_shift, Polylines &fill_lines,
                         const raw_infill_pattern_params &params) const;
    bool fill_surface_by_multilines(const slic3r_api::ExPolygon &expolygon, raw_infill_pattern_params params,
                                    const std::initializer_list<SweepParams> &sweep_params,
                                    Polylines &polylines_out) const;

private:
    coordf_t spacing_priv = 0.;
};

class FillAlignedRectilinear : public FillRectilinear
{
public:
    ~FillAlignedRectilinear() override = default;

protected:
    // Always generate infill at the same angle.
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillMonotonic : public FillRectilinear
{
public:
    ~FillMonotonic() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;
    bool no_sort() const override { return true; }
};

class FillGrid : public FillRectilinear
{
public:
    ~FillGrid() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    // The grid fill will keep the angle constant between the layers, see the implementation of Slic3r::Fill.
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillTriangles : public FillRectilinear
{
public:
    ~FillTriangles() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    // The grid fill will keep the angle constant between the layers, see the implementation of Slic3r::Fill.
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillStars : public FillRectilinear
{
public:
    ~FillStars() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    // The grid fill will keep the angle constant between the layers, see the implementation of Slic3r::Fill.
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillCubic : public FillRectilinear
{
public:
    ~FillCubic() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    // The grid fill will keep the angle constant between the layers, see the implementation of Slic3r::Fill.
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillSupportBase : public FillRectilinear
{
public:
    ~FillSupportBase() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    float _layer_angle(size_t idx) const override { return 0.f; }
};

class FillScatteredRectilinear : public FillRectilinear
{
public:
    ~FillScatteredRectilinear() override = default;
    Polylines fill_surface(const slic3r_api::ExPolygon &expolygon, const raw_infill_pattern_params &params) const override;

protected:
    float _layer_angle(size_t idx) const override;
    std::vector<SegmentedIntersectionLine> _vert_lines_for_polygon(const ExPolygonWithOffset &poly_with_offset,
                                                                   const BoundingBox &bounding_box,
                                                                   const raw_infill_pattern_params &params,
                                                                   coord_t line_spacing) const override;
    coord_t _line_spacing_for_density(const raw_infill_pattern_params &params) const override;
};

// Copy the recipe frame (layer, angle, crossing, link and loop limits, overlap,
// object bounds) and the plugin storage into a filler, then set its spacing.
void configure_filler(FillRectilinear &filler, const raw_infill_pattern_params &params, storage_handle *storage);

} // namespace fills
}}} // namespace slic3r_api::NematXWipeTowerCopy::RectilinearPatternsPlugin

#endif // nematx_wipetower_copy_rectilinear_fills_hpp_
