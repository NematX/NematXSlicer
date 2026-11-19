///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Pure millimeter layout assigns print-ordered source groups to increasing tower
   layers. A close-Z group (its bottom overlaps the tower top) always fuses with
   the top layer: it carries one or two bands, and stacking it would push the
   tower top above the source Zs. An equal or lower-Z group fuses while the fused
   depth stays within the largest used depth so far, otherwise it stacks above the
   tower top, so a descending group does not pile all its purges into one layer.
   Whole-line bands and suffix depth are computed without a plan; layers after the
   last wipe vanish. A floating band at or below the regular tower top takes part
   in that layout; above it, its source layer becomes one floating layer holding
   only that band, added after depth and cone so it changes neither.
   Depth is the suffix maximum of the used band depth, with no minimum, so the
   tower never widens upward and never reserves more than its bands need. Cone
   radius/scaling follow WipeTower2 geometry without a dependency on it and are
   calculated whenever the configured angle is positive. */
#include "WipeTowerLayout.hpp"
#include "TowerFlow.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace nematx_wipetower {
namespace {
// Select which band kinds one layout pass reserves.
enum class BandSelection { Regular, All, Floating };
// Return true when one band belongs to a pass with the given selection.
bool band_selected(const LayoutBandInput &band, BandSelection selection);
// Append whole-line bands to a tower layer in source request order.
void reserve_bands(LayoutLayer &layer, double width, double extra_spacing_ratio,
                   const std::vector<LayoutBandInput> &bands, BandSelection selection);
// Reserve one pass of tower layers from print-ordered inputs; only bands selected by the pass
// count, so layers after the last selected band vanish.
std::vector<LayoutLayer> layout_layers(double width, double extra_spacing_ratio, bool no_sparse_layers,
                                       const std::vector<LayoutLayerInput> &layers, BandSelection selection);
// Enforce every layer's suffix depth and calculate the cone base from the final layer whenever the angle is positive.
void finish_layout(TowerLayout &layout, double width, double cone_angle);
}

// Read borrowed print-ordered groups with positive source heights, fuse or stack into owned increasing tower layers and enforce suffix depth; a floating band above the regular top gets its own layer after depth and cone, invalid dimensions throw and inputs remain unchanged.
TowerLayout layout_tower(const double width, const double extra_spacing_ratio, const bool no_sparse_layers,
                         const double cone_angle, const std::vector<LayoutLayerInput> &layers)
{
    if (!std::isfinite(width) || width <= 0. || !std::isfinite(extra_spacing_ratio) || extra_spacing_ratio <= 0.)
        throw std::invalid_argument("Tower width and line spacing must be positive and finite.");
    // A floating band never makes the tower grow: lay out the regular bands first, then decide
    // whether each floating band already reaches that tower top.
    std::vector<LayoutLayer> regular = layout_layers(width, extra_spacing_ratio, no_sparse_layers, layers,
                                                     BandSelection::Regular);
    const double top_z = regular.empty() ? 0. : regular.back().print_z;
    bool has_floating = false;
    bool reaches_top = false;
    for (const LayoutLayerInput &input : layers)
        for (const LayoutBandInput &band : input.bands)
            if (band.floating) {
                has_floating = true;
                reaches_top = reaches_top || input.print_z <= top_z + 1e-9;
            }
    TowerLayout result;
    if (has_floating && !reaches_top) {
        // Every floating band sits above the regular top: each source layer with floating bands
        // becomes its own floating layer, added after depth and cone so it changes neither.
        result.layers = std::move(regular);
        finish_layout(result, width, cone_angle);
        for (size_t i = 0; i < layers.size(); ++i) {
            const LayoutLayerInput &input = layers[i];
            bool has_band = false;
            for (const LayoutBandInput &band : input.bands)
                has_band = has_band || band_selected(band, BandSelection::Floating);
            if (!has_band)
                continue;
            if (!std::isfinite(input.print_z) || input.print_z <= 0. ||
                !std::isfinite(input.height) || input.height <= 0.)
                throw std::invalid_argument("Tower source groups need positive finite Z and height.");
            // The floating layer keeps the source layer thickness: no tower layer lies below it,
            // and its band is reserved like the first band of a normal layer. Its depth is its own
            // used depth, since it is never part of the regular suffix depth.
            LayoutLayer layer{i, input.print_z, input.height, 0., 0., {}, true};
            reserve_bands(layer, width, extra_spacing_ratio, input.bands, BandSelection::Floating);
            layer.depth = layer.used;
            result.layers.push_back(std::move(layer));
        }
        return result;
    }
    result.layers = has_floating ?
        layout_layers(width, extra_spacing_ratio, no_sparse_layers, layers, BandSelection::All) : std::move(regular);
    finish_layout(result, width, cone_angle);
    return result;
}

namespace {
// Return true when one band belongs to a layout pass with the given selection.
bool band_selected(const LayoutBandInput &band, const BandSelection selection)
{
    return selection == BandSelection::All || band.floating == (selection == BandSelection::Floating);
}

// Read borrowed nonnegative volumes and positive widths/nozzles, appending owned bands at rounded-flow pitch and volume including connectors; bands outside the pass selection are skipped, invalid flow/allocation failures throw and no source ownership changes.
void reserve_bands(LayoutLayer &layer, const double width, const double extra_spacing_ratio,
                   const std::vector<LayoutBandInput> &bands, const BandSelection selection)
{
    for (const LayoutBandInput &band : bands) {
        if (!band_selected(band, selection))
            continue;
        if (!std::isfinite(band.volume) || band.volume < 0. || !std::isfinite(band.line_width) || band.line_width <= 0.)
            throw std::invalid_argument("Tower bands need nonnegative volume and positive line width.");
        const TowerFlow flow = TowerFlow::from_width(float(band.line_width), float(band.nozzle_diameter),
            float(layer.height));
        const double pitch = flow.spacing() * extra_spacing_ratio;
        const size_t lines = wipe_band_line_count(band.volume, width, flow.mm3_per_mm(), pitch);
        const double y1 = layer.used + double(lines) * pitch;
        layer.bands.push_back({band.request_index, layer.used, y1, band.line_width, layer.bands.size() % 2 != 0, pitch});
        layer.used = y1;
    }
}

// Read borrowed print-ordered groups with positive source heights, fuse or stack their selected bands into owned increasing tower layers, and stop at the last selected band; a close-Z group always fuses with the top layer, an equal or lower-Z group only while the fused depth stays within the largest used depth so far, sparse look-ahead bounds skipped gaps, invalid dimensions throw and inputs remain unchanged.
std::vector<LayoutLayer> layout_layers(const double width, const double extra_spacing_ratio,
                                       const bool no_sparse_layers, const std::vector<LayoutLayerInput> &layers,
                                       const BandSelection selection)
{
    size_t last = 0;
    bool has_request = false;
    for (size_t i = 0; i < layers.size(); ++i) {
        bool selected = false;
        for (const LayoutBandInput &band : layers[i].bands)
            if (band_selected(band, selection))
                selected = true;
        if (selected) {
            last = i;
            has_request = true;
        }
    }
    std::vector<LayoutLayer> result;
    if (!has_request)
        return result;
    const double epsilon = 1e-9;
    double maximum_used = 0.;
    double common_max = std::numeric_limits<double>::max();
    for (size_t i = 0; i <= last; ++i) {
        const LayoutLayerInput &input = layers[i];
        if (!std::isfinite(input.print_z) || input.print_z <= 0. ||
            !std::isfinite(input.height) || input.height <= 0. ||
            !std::isfinite(input.max_height) || input.max_height <= 0.)
            throw std::invalid_argument("Tower source groups need positive finite Z, height and maximum height.");
        bool selected = false;
        for (const LayoutBandInput &band : input.bands)
            if (band_selected(band, selection))
                selected = true;
        const double top_z = result.empty() ? 0. : result.back().print_z;
        common_max = std::min(common_max, input.max_height);
        if (result.empty() || (input.print_z > top_z && input.print_z - input.height >= top_z - epsilon)) {
            if (no_sparse_layers && !result.empty() && !selected && i < last &&
                layers[i + 1].print_z > input.print_z &&
                layers[i + 1].print_z - layers[i + 1].height >= top_z - epsilon &&
                layers[i + 1].print_z - top_z <= std::min(common_max, layers[i + 1].max_height) + epsilon)
                continue;
            LayoutLayer layer{i, input.print_z, input.print_z - top_z, 0., 0., {}, false};
            reserve_bands(layer, width, extra_spacing_ratio, input.bands, selection);
            result.push_back(std::move(layer));
        } else {
            if (!selected)
                continue;
            // A close group (its bottom overlaps the tower top) always fuses: it brings one or two
            // bands, and stacking it would push the tower top above the source Zs. An equal or
            // lower group fuses only while the fused depth stays within the largest used depth so
            // far, so a descending group does not pile all its purges into one layer.
            const bool close = input.print_z > top_z;
            LayoutLayer fused = result.back();
            reserve_bands(fused, width, extra_spacing_ratio, input.bands, selection);
            if (close || fused.used <= maximum_used + epsilon)
                result.back() = std::move(fused);
            else {
                LayoutLayer stacked{i, top_z + input.height, input.height, 0., 0., {}, false};
                reserve_bands(stacked, width, extra_spacing_ratio, input.bands, selection);
                result.push_back(std::move(stacked));
            }
        }
        maximum_used = std::max(maximum_used, result.back().used);
        common_max = std::numeric_limits<double>::max();
    }
    return result;
}

// Enforce every regular layer's suffix depth (the largest used depth of the layer and of the layers above, with no minimum) and calculate the cone base from the final regular layer whenever the angle is positive; floating layers are appended only after this run.
void finish_layout(TowerLayout &layout, const double width, const double cone_angle)
{
    double depth = 0.;
    for (std::vector<LayoutLayer>::reverse_iterator it = layout.layers.rbegin(); it != layout.layers.rend(); ++it) {
        depth = std::max(depth, it->used);
        it->depth = depth;
    }
    if (cone_angle > 0. && !layout.layers.empty()) {
        wipe_tower_cone_base(width, layout.layers.back().print_z, depth, cone_angle,
                             layout.cone_base_radius, layout.cone_scale);
        layout.cone_slope = std::tan(cone_angle * 3.14159265358979323846 / 360.);
    }
}
}

// Return the smallest whole-line count whose rounded cross-section times X lines and inter-line connectors reaches a nonnegative volume; finite positive dimensions/flow are required, invalid or excessive counts throw without mutation.
size_t wipe_band_line_count(const double volume, const double width, const double mm3_per_mm, const double pitch)
{
    if (!std::isfinite(volume) || volume < 0. || !std::isfinite(width) || width <= 0. ||
        !std::isfinite(mm3_per_mm) || mm3_per_mm <= 0. || !std::isfinite(pitch) || pitch <= 0.)
        throw std::invalid_argument("Wipe lines require nonnegative volume and positive finite geometry and flow.");
    if (volume == 0.)
        return 0;
    const double lines = std::ceil((volume / mm3_per_mm + pitch) / (width + pitch));
    if (lines >= double(std::numeric_limits<size_t>::max()))
        throw std::invalid_argument("Wipe line count exceeds addressable geometry.");
    return size_t(lines);
}

// Evaluate finite positive dimensions and angle in degrees into caller-owned radius/scaling outputs; the formula changes no geometry and allocation is unnecessary.
void wipe_tower_cone_base(const double width, const double height, const double depth, const double angle,
                          double &radius, double &support_scale)
{
    radius = std::tan(angle * 3.14159265358979323846 / 360.) * height;
    const double fake_width = 0.66 * width;
    const double diagonal = std::hypot(fake_width / 2., depth / 2.);
    support_scale = 1.;
    if (radius > diagonal) {
        const double sine = 0.5 * depth / diagonal;
        const double tangent = depth / fake_width;
        const double extension = (radius - diagonal) * sine;
        support_scale = (fake_width / 2. + extension / tangent + extension * tangent) / (fake_width / 2.);
    }
}
}
