///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
 * Geometry of the XY size compensation, for one layer (see the vocabulary in
 * XYSizeCompensationHelpers.hpp). Entry point: compensate_layer().
 *
 *   compensate_layer()
 *     for each island: process_island()
 *       compose_island()               outside moved by outer deltas, holes by inner deltas
 *         compensate_outer_contour()   filled exterior after outer offsets
 *         compensate_hole_voids()      filled voids after inner offsets
 *           hierarchical_offset()      a different distance for each region
 *       positive_growth_candidates()   where each region may add new material
 *       if needs_collapse_protection(): protect_island()
 *         shared_settings_of()         choose a shared or regional calculation
 *         protect_uniform_island()     shared settings across the island
 *         protect_regional_island()    different settings, limited by regional claims
 *     union of all compensated islands
 *     assign_regions()                 give every piece of material to exactly one region
 *       assign_unclaimed()             measure contacts for any remaining pieces
 *         choose_owner()               select their region using the fallback rules
 *
 * Known limit: where two regions with different deltas meet, the region clip
 * can leave a small notch or step. Unclaimed material is assigned by contact
 * with the regional masks and reported when its area is significant.
 *
 * All geometry is allocated in the caller's storage, which locks its own
 * allocations: several threads may call these functions with the same storage.
 * Returned objects stay valid until destroyed; never clear the storage while
 * any of them is alive.
 */

#include "XYSizeCompensationHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "ElephantFootCompensation.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api { namespace XYSizeCompensationPlugin { namespace detail {
namespace {

constexpr double k_miter_limit = 3.0;
constexpr std::uint64_t k_miter_factor = 3;
constexpr std::uint64_t k_frame_factor = 6;
constexpr coord_t k_clipper_padding = 2 * SCALED_EPSILON;
constexpr std::uint64_t k_clipper_hi_range = (std::uint64_t(1) << 62) - 1;

using ExPolygon = slic3r_api::ExPolygon;
using ExPolygonCollection = slic3r_api::ExPolygonCollection;
using StoredExPolygon = slic3r_api::StoredExPolygon;
using StoredExPolygonCollection = slic3r_api::StoredExPolygonCollection;
using Polygon = slic3r_api::Polygon;
using StoredPolygon = slic3r_api::StoredPolygon;
using PolygonCollection = slic3r_api::PolygonCollection;
using StoredPolygonCollection = slic3r_api::StoredPolygonCollection;

// Merge only nonempty contours. The ABI box does not retain the host's
// "defined" flag, so asking for an empty contour's box is ambiguous.
c_bounding_box bounds_of(const ExPolygonCollection &geometry)
{
    c_bounding_box bounds = bounding_box_empty();
    for (const ExPolygon area : geometry) {
        const Polygon contour = area.contour();
        if (!contour.empty())
            bounding_box_merge(bounds, contour.bounding_box());
    }
    return bounds;
}

std::uint64_t coordinate_magnitude(coord_t value)
{
    // Avoid negating INT64_MIN, which is not representable as a signed coord_t.
    return value < 0 ? static_cast<std::uint64_t>(-(value + 1)) + 1
                     : static_cast<std::uint64_t>(value);
}

coord_t checked_miter_margin(std::uint64_t delta_magnitude, std::uint64_t factor,
                             coord_t padding, const char *error_message)
{
    if (padding < 0 || factor == 0)
        throw std::out_of_range(error_message);
    const std::uint64_t unsigned_padding = static_cast<std::uint64_t>(padding);
    if (unsigned_padding > k_clipper_hi_range ||
        delta_magnitude > (k_clipper_hi_range - unsigned_padding) / factor)
        throw std::out_of_range(error_message);
    return static_cast<coord_t>(delta_magnitude * factor + unsigned_padding);
}

coord_t checked_miter_margin(coord_t delta, std::uint64_t factor,
                             coord_t padding, const char *error_message)
{
    return checked_miter_margin(coordinate_magnitude(delta), factor, padding, error_message);
}

StoredExPolygonCollection union_collections(storage_handle *storage,
                                            const std::vector<ExPolygonCollection> &collections)
{
    ClipperOperand combined(storage);
    bool has_paths = false;
    for (const ExPolygonCollection &collection : collections) {
        if (collection.empty())
            continue;
        ClipperOperand next(storage, collection);
        if (!has_paths) {
            combined = std::move(next);
            has_paths = true;
        } else {
            combined.concat_replace(next);
        }
    }
    if (!has_paths)
        return StoredExPolygonCollection(storage);
    return clipper_union(storage, combined).to_expolygon_collection();
}

StoredExPolygonCollection union_polygons(storage_handle *storage,
                                         const PolygonCollection &polygons)
{
    if (polygons.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand paths(storage, polygons);
    return clipper_union(storage, paths).to_expolygon_collection();
}

StoredExPolygonCollection union_polygons(storage_handle *storage,
                                         const StoredPolygonCollection &polygons)
{
    return union_polygons(storage, polygons.readonly());
}

StoredExPolygonCollection union_two(storage_handle *storage,
                                    const ExPolygonCollection &first,
                                    const ExPolygonCollection &second)
{
    const std::vector<ExPolygonCollection> collections{ first, second };
    return union_collections(storage, collections);
}

StoredExPolygonCollection intersect(storage_handle *storage,
                                    const ExPolygonCollection &subject,
                                    const ExPolygonCollection &clip)
{
    if (subject.empty() || clip.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_intersection(storage, subject_operand, clip_operand).to_expolygon_collection();
}

StoredExPolygonCollection difference(storage_handle *storage,
                                     const ExPolygonCollection &subject,
                                     const ExPolygonCollection &clip)
{
    if (subject.empty())
        return StoredExPolygonCollection(storage);
    if (clip.empty())
        return subject.clone(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_diff(storage, subject_operand, clip_operand).to_expolygon_collection();
}

StoredExPolygonCollection offset_or_copy(storage_handle *storage,
                                         const ExPolygonCollection &geometry,
                                         coord_t delta)
{
    if (geometry.empty())
        return StoredExPolygonCollection(storage);
    if (delta == 0)
        return geometry.clone(storage);

    // Miter joins can extend three times the requested distance. Check the
    // integer bounding box before handing any coordinates to Clipper.
    const c_bounding_box bounds = bounds_of(geometry);
    const coord_t margin = checked_miter_margin(delta, k_miter_factor, k_clipper_padding,
                                                "XY offset exceeds Clipper coordinate limits.");
    const coord_t safe_limit = static_cast<coord_t>(k_clipper_hi_range) - margin;
    if (!bounding_box_valid(bounds) || bounds.min.x < -safe_limit || bounds.min.y < -safe_limit ||
        bounds.max.x > safe_limit || bounds.max.y > safe_limit)
        throw std::out_of_range("XY offset exceeds Clipper coordinate limits.");

    ClipperOperand subject(storage, geometry);
    return clipper_offset(storage, subject, double(delta), CLIPPER_JOIN_MITER,
                          k_miter_limit).to_expolygon_collection();
}

StoredExPolygonCollection polygon_collection(storage_handle *storage, const Polygon &polygon)
{
    StoredPolygonCollection polygons(storage);
    polygons.push_back(polygon);
    return union_polygons(storage, polygons);
}

// One borrowed geometry mask per region, in the same order as the settings.
using RegionMasks = std::vector<ExPolygonCollection>;

// Borrowed views of owned collections. The collections must outlive these views.
std::vector<ExPolygonCollection> views_of(const std::vector<StoredExPolygonCollection> &collections)
{
    std::vector<ExPolygonCollection> views;
    views.reserve(collections.size());
    for (const StoredExPolygonCollection &collection : collections)
        views.push_back(collection.readonly());
    return views;
}

enum class Boundary { Outer, Inner };

coord_t delta_for(const RegionSettings &settings, Boundary boundary)
{
    return boundary == Boundary::Inner ? settings.inner_delta : settings.outer_delta;
}

// The largest amount of material a region asks to remove, as a negative
// distance, or 0 when the region only adds material.
coord_t withdrawal_delta_of(const RegionSettings &settings)
{
    return std::min<coord_t>(0, std::min(settings.outer_delta, settings.inner_delta));
}

// Move the boundary of `domain` by a different distance in each region.
//
// A single offset can only move the whole boundary by one distance. To give
// each region its own distance, every distinct delta used in this island is
// handled as a "level", from the smallest to the largest:
// - the smallest delta is applied to the whole domain;
// - each larger delta is applied only to the part of the domain covered by the
//   regions whose delta is at least that large.
// The union of these pieces moves the boundary next to each region by that
// region's delta; where two regions meet, the larger delta wins.
// Boundary::Inner selects inner_delta for hole walls; Boundary::Outer selects
// outer_delta for the outside contour. Regions with an empty mask are ignored.
StoredExPolygonCollection hierarchical_offset(storage_handle *storage,
                                              const ExPolygonCollection &domain,
                                              const std::vector<RegionSettings> &settings,
                                              const RegionMasks &masks,
                                              Boundary boundary)
{
    if (domain.empty())
        return StoredExPolygonCollection(storage);

    std::set<coord_t> levels;
    for (std::size_t region = 0; region < settings.size(); ++region)
        if (!masks[region].empty())
            levels.insert(delta_for(settings[region], boundary));
    if (levels.empty())
        throw std::runtime_error("XY compensation has no region mask for a source island.");

    std::vector<StoredExPolygonCollection> contributions;
    auto level = levels.begin();
    contributions.push_back(offset_or_copy(storage, domain, *level));
    for (++level; level != levels.end(); ++level) {
        // Gather the masks of the regions that move at least this far.
        std::vector<ExPolygonCollection> selected_masks;
        selected_masks.reserve(masks.size());
        for (std::size_t region = 0; region < settings.size(); ++region)
            if (!masks[region].empty() && delta_for(settings[region], boundary) >= *level)
                selected_masks.push_back(masks[region]);
        StoredExPolygonCollection selected_regions = union_collections(storage, selected_masks);
        StoredExPolygonCollection selected = intersect(storage, domain, selected_regions);
        if (!selected.empty())
            contributions.push_back(offset_or_copy(storage, selected, *level));
    }

    return union_collections(storage, views_of(contributions));
}

void write_ring(polygon_handle *destination, const std::vector<c_point> &points)
{
    multipoint_handle *mutable_points = polygon_as_multipoint(destination);
    multipoint_clear(mutable_points);
    for (const c_point point : points)
        multipoint_push_back(mutable_points, point);
}

StoredExPolygonCollection single_expolygon(storage_handle *storage, const ExPolygon &expolygon)
{
    StoredExPolygonCollection result(storage);
    result.push_back(expolygon);
    return result;
}

// Reverse clockwise hole rings into ordinary filled polygons. Append to the
// caller's collection so holes from several frame pieces can be unioned once.
void append_holes_as_filled_polygons(storage_handle *storage, const ExPolygon &area,
                                     StoredPolygonCollection &filled_holes)
{
    for (const Polygon hole : area.holes()) {
        StoredPolygon filled_hole(storage);
        filled_hole.copy_from(hole);
        filled_hole.reverse();
        filled_holes.push_back(filled_hole);
    }
}

// Move only the outside boundary. Clipping each region's offset to its mask
// can open small holes; they are discarded here because the island's real
// holes are handled separately, by compensate_hole_voids().
StoredExPolygonCollection compensate_outer_contour(storage_handle *storage,
                                                    const ExPolygon &source,
                                                    const std::vector<RegionSettings> &settings,
                                                    const RegionMasks &masks)
{
    // Outside: offset the island as if it had no holes, and keep only the
    // resulting outer contours.
    StoredExPolygonCollection outer_domain = polygon_collection(storage, source.contour());
    StoredExPolygonCollection outer_composition = hierarchical_offset(storage, outer_domain, settings, masks, Boundary::Outer);
    StoredPolygonCollection outer_contours(storage);
    for (const ExPolygon area : outer_composition)
        outer_contours.push_back(area.contour());
    return union_polygons(storage, outer_contours);
}

// Return the compensated holes as filled voids, ready to subtract from the
// outside. The frame, its safety margins and hole extraction stay local here.
StoredExPolygonCollection compensate_hole_voids(storage_handle *storage,
                                                const ExPolygon &source,
                                                const std::vector<RegionSettings> &settings,
                                                const RegionMasks &masks)
{
    // Holes: build a big rectangle ("frame") around the island and punch the
    // island's holes into it. Offsetting this frame moves the hole walls in
    // the right direction, and the frame's own border is far enough away
    // not to interfere. The frame margin also leaves room for the miter
    // joins of the offset, within Clipper's coordinate range.
    const c_bounding_box bounds = source.contour().bounding_box();
    std::uint64_t maximum_delta = 0;
    for (std::size_t region = 0; region < settings.size(); ++region)
        if (!masks[region].empty())
            maximum_delta = std::max(maximum_delta, coordinate_magnitude(settings[region].inner_delta));
    const coord_t margin = checked_miter_margin(maximum_delta, k_frame_factor, k_clipper_padding,
                                                "XY inner compensation frame exceeds Clipper coordinate limits.");
    const coord_t inner_offset_margin = checked_miter_margin(maximum_delta, k_miter_factor, k_clipper_padding,
                                                             "XY inner compensation frame exceeds Clipper coordinate limits.");
    const coord_t safe_frame_limit = static_cast<coord_t>(k_clipper_hi_range) - inner_offset_margin;
    if (!bounding_box_valid(bounds) || margin > safe_frame_limit)
        throw std::out_of_range("XY inner compensation frame exceeds Clipper coordinate limits.");
    const coord_t safe_source_limit = safe_frame_limit - margin;
    if (bounds.min.x < -safe_source_limit || bounds.min.y < -safe_source_limit ||
        bounds.max.x > safe_source_limit || bounds.max.y > safe_source_limit)
        throw std::out_of_range("XY inner compensation frame exceeds Clipper coordinate limits.");

    const std::vector<c_point> frame_points{
        { bounds.min.x - margin, bounds.min.y - margin },
        { bounds.max.x + margin, bounds.min.y - margin },
        { bounds.max.x + margin, bounds.max.y + margin },
        { bounds.min.x - margin, bounds.max.y + margin }
    };
    StoredExPolygon inner_domain(storage);
    // Preserve the island's holes, then replace its contour with the frame.
    inner_domain.copy_from(source);
    write_ring(expolygon_contour(inner_domain.mutable_handle()), frame_points);
    StoredExPolygonCollection inner_domains(storage);
    inner_domains.push_back(inner_domain.readonly());

    // The moved holes are the holes of the offset frame. Turn them into
    // ordinary filled shapes (reverse their direction) and cut them out of
    // the outside shape computed above.
    StoredExPolygonCollection inner_composition = hierarchical_offset(storage, inner_domains, settings, masks, Boundary::Inner);
    StoredPolygonCollection filled_holes(storage);
    for (const ExPolygon area : inner_composition)
        append_holes_as_filled_polygons(storage, area, filled_holes);
    return union_polygons(storage, filled_holes);
}

// Compose one original island before merging it with neighboring islands:
// compensated outside minus compensated hole voids. Settings and masks use
// the same region order throughout both independent boundary passes.
StoredExPolygonCollection compose_island(storage_handle *storage,
                                         const ExPolygon &source,
                                         const std::vector<RegionSettings> &settings,
                                         const RegionMasks &masks)
{
    if (source.contour().empty())
        return StoredExPolygonCollection(storage);
    StoredExPolygonCollection outer = compensate_outer_contour(storage, source, settings, masks);
    if (source.hole_size() == 0 || outer.empty())
        return outer;
    StoredExPolygonCollection hole_voids = compensate_hole_voids(storage, source, settings, masks);
    if (hole_voids.empty())
        return outer;
    return difference(storage, outer, hole_voids);
}

// Where a region could add new material, and how strongly it claims it.
// When the compensated island has new material, this tells which region the
// material should belong to. A larger priority (larger positive delta) wins.
struct RegionCandidate
{
    explicit RegionCandidate(storage_handle *storage) : geometry(storage) {}
    RegionCandidate(RegionCandidate &&) noexcept = default;
    RegionCandidate &operator=(RegionCandidate &&) noexcept = default;

    std::size_t region_index = 0;
    coord_t priority = 0;
    StoredExPolygonCollection geometry;
};

// For each region that adds material, compute the area it could grow into:
// - outside: the band between the island and the island grown by the region's
//   outer delta, limited to what the region's own mask can reach;
// - inside holes: the parts of the holes that the region's mask can reach
//   when grown by its inner delta.
std::vector<RegionCandidate> positive_growth_candidates(storage_handle *storage,
                                                        const ExPolygon &source,
                                                        const std::vector<RegionSettings> &settings,
                                                        const RegionMasks &masks)
{
    std::vector<RegionCandidate> candidates;
    // Both kinds of growth transfer their geometry here. Preserve insertion
    // order: assign_unclaimed() uses it to break equal-priority ties.
    const auto add_candidate = [storage, &candidates](std::size_t region_index, coord_t priority,
                                                     StoredExPolygonCollection geometry) {
        if (geometry.empty())
            return;
        RegionCandidate candidate(storage);
        candidate.region_index = region_index;
        candidate.priority = priority;
        candidate.geometry = std::move(geometry);
        candidates.push_back(std::move(candidate));
    };
    StoredExPolygonCollection outer_domain = polygon_collection(storage, source.contour());

    // The holes of the island, as filled shapes.
    StoredPolygonCollection positive_hole_polygons(storage);
    append_holes_as_filled_polygons(storage, source, positive_hole_polygons);
    StoredExPolygonCollection hole_voids = union_polygons(storage, positive_hole_polygons);

    for (std::size_t idx = 0; idx < settings.size(); ++idx) {
        const RegionSettings &region = settings[idx];
        const ExPolygonCollection &mask = masks[idx];
        if (mask.empty())
            continue;
        if (region.outer_delta > 0) {
            StoredExPolygonCollection expanded_domain = offset_or_copy(storage, outer_domain, region.outer_delta);
            StoredExPolygonCollection outer_band = difference(storage, expanded_domain, outer_domain);
            StoredExPolygonCollection expanded_region = offset_or_copy(storage, mask, region.outer_delta);
            StoredExPolygonCollection outer_growth = intersect(storage, expanded_region, outer_band);
            add_candidate(idx, region.outer_delta, std::move(outer_growth));
        }
        if (region.inner_delta > 0 && !hole_voids.empty()) {
            StoredExPolygonCollection expanded_region = offset_or_copy(storage, mask, region.inner_delta);
            StoredExPolygonCollection inner_growth = intersect(storage, expanded_region, hole_voids);
            add_candidate(idx, region.inner_delta, std::move(inner_growth));
        }
    }
    return candidates;
}

#ifndef NDEBUG
// Debug-only: the host validates region slices before and after each
// post-slicing plugin, and the masks cover their island by construction.
void validate_original_masks(storage_handle *storage,
                             const ExPolygon &source,
                             const RegionMasks &masks)
{
    StoredExPolygonCollection material_union = union_collections(storage, masks);
    StoredExPolygonCollection source_collection = single_expolygon(storage, source);
    StoredExPolygonCollection missing = difference(storage, source_collection, material_union);
    if (!missing.empty())
        throw std::runtime_error("Layer regions do not cover an original XY compensation island.");

    for (std::size_t lhs = 0; lhs < masks.size(); ++lhs) {
        if (masks[lhs].empty())
            continue;
        for (std::size_t rhs = lhs + 1; rhs < masks.size(); ++rhs) {
            if (masks[rhs].empty())
                continue;
            StoredExPolygonCollection overlap = intersect(storage, masks[lhs], masks[rhs]);
            double overlap_area = 0.;
            for (const ExPolygon part : overlap)
                overlap_area += std::abs(part.area());
            if (overlap_area > k_insignificant_area)
                throw std::runtime_error("Original XY compensation layer-region masks overlap.");
        }
    }
}
#endif

double total_area(const ExPolygonCollection &geometry)
{
    double area = 0.;
    for (const ExPolygon part : geometry)
        area += std::abs(part.area());
    return area;
}

// Select a region without doing geometry work. Zero contact is not a match.
// Equal contacts keep the first region; equal priorities keep the first valid
// candidate. Candidates are scanned in insertion order here; assign_regions()
// sorts them first.
std::size_t choose_owner(const std::vector<double> &contact,
                         const std::vector<RegionCandidate> &candidates,
                         const std::vector<StoredExPolygonCollection> &assigned)
{
    const auto most_contact = std::max_element(contact.begin(), contact.end());
    if (most_contact != contact.end() && *most_contact > 0.)
        return static_cast<std::size_t>(most_contact - contact.begin());

    // No contact: prefer the strongest growth candidate, keeping ties in order.
    const RegionCandidate *best_candidate = nullptr;
    for (const RegionCandidate &candidate : candidates)
        if (candidate.region_index < assigned.size() &&
            (best_candidate == nullptr || candidate.priority > best_candidate->priority))
            best_candidate = &candidate;
    if (best_candidate != nullptr)
        return best_candidate->region_index;

    // No candidate: reuse the first region that already contains material.
    for (std::size_t region = 0; region < assigned.size(); ++region)
        if (!assigned[region].empty())
            return region;
    throw std::logic_error("XY compensation has no layer region to receive unclaimed material.");
}

// Give each unclaimed component to the region it touches most, through its
// assigned material or its growth candidates. A component touching no region
// goes to the highest-priority candidate's region, else to the first region
// with material. Returns the area handed out this way.
double assign_unclaimed(storage_handle *storage,
                        const ExPolygonCollection &unclaimed,
                        const std::vector<RegionCandidate> &candidates,
                        std::vector<StoredExPolygonCollection> &assigned)
{
    constexpr coord_t k_contact_probe = 10 * SCALED_EPSILON;
    double handed_out = 0.;
    for (const ExPolygon component : unclaimed) {
        StoredExPolygonCollection piece = single_expolygon(storage, component);
        StoredExPolygonCollection probe = offset_or_copy(storage, piece, k_contact_probe);
        std::vector<double> contact(assigned.size(), 0.);
        for (std::size_t region = 0; region < assigned.size(); ++region) {
            const StoredExPolygonCollection touching = intersect(storage, probe, assigned[region]);
            contact[region] += total_area(touching);
        }
        for (const RegionCandidate &candidate : candidates)
            if (candidate.region_index < assigned.size()) {
                const StoredExPolygonCollection touching = intersect(storage, probe, candidate.geometry);
                contact[candidate.region_index] += total_area(touching);
            }

        const std::size_t target = choose_owner(contact, candidates, assigned);

        assigned[target] = union_two(storage, assigned[target], piece);
        handed_out += std::abs(component.area());
    }
    return handed_out;
}

// Split `geometry` between the regions, without overlap:
// 1. material inside a region's original mask stays in that region;
// 2. the rest is new material, given to the growth candidates by decreasing
//    priority (ties: lower region index first);
// 3. anything still unassigned goes through assign_unclaimed().
// `geometry` must not overlap itself (a Clipper union, intersection or
// difference result): it is copied as is, not merged again.
// original_masks has one entry per region. If fallback_area is given, the
// area handed out at step 3 is added to it.
std::vector<StoredExPolygonCollection> assign_regions(storage_handle *storage,
                                                      const ExPolygonCollection &geometry,
                                                      const std::vector<ExPolygonCollection> &original_masks,
                                                      const std::vector<RegionCandidate> &candidates,
                                                      double *fallback_area = nullptr)
{
    const std::size_t count = original_masks.size();
    std::vector<StoredExPolygonCollection> assigned;
    assigned.reserve(count);
    for (std::size_t idx = 0; idx < count; ++idx)
        assigned.emplace_back(storage);

    StoredExPolygonCollection remaining = geometry.clone(storage);

    // Preserve original material in its region before assigning any new material.
    for (std::size_t region = 0; region < count && !remaining.empty(); ++region) {
        assigned[region] = intersect(storage, remaining, original_masks[region]);
        if (!assigned[region].empty())
            remaining = difference(storage, remaining, assigned[region]);
    }

    std::vector<const RegionCandidate *> ordered;
    ordered.reserve(candidates.size());
    for (const RegionCandidate &candidate : candidates)
        ordered.push_back(&candidate);
    std::stable_sort(ordered.begin(), ordered.end(), [](const RegionCandidate *lhs, const RegionCandidate *rhs) {
        if (lhs->priority != rhs->priority)
            return lhs->priority > rhs->priority;
        return lhs->region_index < rhs->region_index;
    });

    for (const RegionCandidate *candidate : ordered) {
        if (remaining.empty())
            break;
        if (candidate->region_index >= count)
            throw std::logic_error("XY compensation candidate refers to a missing region.");
        if (candidate->geometry.empty())
            continue;
        StoredExPolygonCollection claim = intersect(storage, remaining, candidate->geometry);
        if (!claim.empty()) {
            assigned[candidate->region_index] = union_two(storage, assigned[candidate->region_index], claim);
            remaining = difference(storage, remaining, claim);
        }
    }
    // Unclaimed material may be a rounding sliver or a significant miter
    // artifact where regional offsets meet (a known limit, see the file header).
    // Keep all of it: the host rebuilds the layer from region slices, so
    // unassigned material would silently disappear.
    if (!remaining.empty()) {
        const double area = assign_unclaimed(storage, remaining, candidates, assigned);
        if (fallback_area != nullptr)
            *fallback_area += area;
    }
    return assigned;
}

// Result of process_island(): the compensated island and the growth
// candidates used later to decide which region owns its new material.
struct IslandResult
{
    explicit IslandResult(storage_handle *storage) : geometry(storage) {}
    IslandResult(IslandResult &&) noexcept = default;
    IslandResult &operator=(IslandResult &&) noexcept = default;

    StoredExPolygonCollection geometry;
    std::vector<RegionCandidate> candidates;
};

// Run the elephant foot routine on every part of `geometry`: it removes up to
// `withdrawal_distance` of material, but less where a wall is close to
// `minimum_contour_width_mm`, so narrow walls survive.
StoredExPolygonCollection protect_narrow_walls(storage_handle *storage,
                                               const ExPolygonCollection &geometry,
                                               double minimum_contour_width_mm,
                                               coord_t withdrawal_distance)
{
    if (!std::isfinite(minimum_contour_width_mm) || minimum_contour_width_mm <= 0.)
        throw std::invalid_argument("First-layer collapse protection requires a positive external-perimeter width.");
    StoredExPolygonCollection protected_geometry(storage);
    for (const ExPolygon area : geometry) {
        StoredExPolygonCollection protected_area = elephant_foot_compensation(
            storage, area, minimum_contour_width_mm, withdrawal_distance);
        protected_geometry.append_move_from(protected_area);
    }
    return protected_geometry;
}

// Restore what the elephant foot step removed beyond each boundary's delta.
// compose_island() only uses these two deltas; the other settings stay neutral.
RegionSettings regrow_settings(const RegionSettings &region, coord_t withdrawal_distance)
{
    RegionSettings regrow;
    regrow.outer_delta = std::min<coord_t>(region.outer_delta, 0) + withdrawal_distance;
    regrow.inner_delta = std::min<coord_t>(region.inner_delta, 0) + withdrawal_distance;
    return regrow;
}

// Protect narrow walls using the strongest withdrawal requested for this island.
// Then grow back the part of that withdrawal not requested for the contour or
// hole walls, using their separate deltas. Keep the result inside the positive
// geometry, so regrowth cannot add material beyond the earlier growth step.
StoredExPolygonCollection protect_uniform_island(storage_handle *storage,
                                                 const ExPolygonCollection &growth_only_geometry,
                                                 const RegionSettings &shared)
{
    const coord_t withdrawal_distance = -withdrawal_delta_of(shared);
    StoredExPolygonCollection protected_geometry = protect_narrow_walls(
        storage, growth_only_geometry, shared.minimum_contour_width_mm, withdrawal_distance);

    // Regrow each protected part using the separate contour and hole corrections.
    const std::vector<RegionSettings> regrow{ regrow_settings(shared, withdrawal_distance) };
    StoredExPolygonCollection regrown(storage);
    for (const ExPolygon area : protected_geometry) {
        const StoredExPolygonCollection area_mask = single_expolygon(storage, area);
        StoredExPolygonCollection part = compose_island(storage, area, regrow, RegionMasks{ area_mask.readonly() });
        regrown.append_move_from(part);
    }
    return intersect(storage, regrown, growth_only_geometry);
}

// Protect each enabled region with its own settings. Claims decide which
// material that region may keep, including material added by positive offsets.
StoredExPolygonCollection protect_regional_island(storage_handle *storage,
                                                  const ExPolygonCollection &growth_only_geometry,
                                                  const std::vector<RegionSettings> &settings,
                                                  const RegionMasks &masks,
                                                  const std::vector<RegionCandidate> &candidates)
{
    // Different settings: first split the positive geometry between regions
    // ("claims"). Claims include the new material, so a hole that opens at a
    // region edge is still seen by the region that owns that edge.
    std::vector<StoredExPolygonCollection> positive_claims =
        assign_regions(storage, growth_only_geometry, masks, candidates);
    const RegionMasks claim_masks = views_of(positive_claims);

    StoredExPolygonCollection protected_union(storage);
    for (std::size_t region_idx = 0; region_idx < settings.size(); ++region_idx) {
        const RegionSettings &region = settings[region_idx];
        const coord_t withdrawal_delta = withdrawal_delta_of(region);
        if (masks[region_idx].empty() || !region.no_collapse || withdrawal_delta >= 0 || positive_claims[region_idx].empty())
            continue;

        const coord_t withdrawal_distance = -withdrawal_delta;
        StoredExPolygonCollection protected_geometry = protect_narrow_walls(
            storage, growth_only_geometry, region.minimum_contour_width_mm, withdrawal_distance);

        // Regrow with this region's deltas only; the other regions do not move,
        // so their claims only act as masks for the levels.
        std::vector<RegionSettings> regrow(settings.size());
        regrow[region_idx] = regrow_settings(region, withdrawal_distance);
        StoredExPolygonCollection protected_for_region(storage);
        for (const ExPolygon area : protected_geometry) {
            StoredExPolygonCollection part = compose_island(storage, area, regrow, claim_masks);
            protected_for_region.append_move_from(part);
        }
        // Union before clipping to the claim. With integer coordinates, this
        // step rounds intersection vertices; clipping the raw pieces directly
        // can change the final coordinates. Keep these operations in this order.
        StoredExPolygonCollection protected_union_for_region = union_collections(
            storage, std::vector<ExPolygonCollection>{ protected_for_region.readonly() });
        protected_for_region = intersect(storage, protected_union_for_region, positive_claims[region_idx]);
        protected_union.append_move_from(protected_for_region);
    }
    return protected_union;
}

// A second geometry calculation is needed only for material-bearing regions
// that enable no-collapse and request a withdrawal.
bool needs_collapse_protection(const std::vector<RegionSettings> &settings,
                               const RegionMasks &masks)
{
    for (std::size_t region = 0; region < settings.size(); ++region)
        if (!masks[region].empty() && settings[region].no_collapse && withdrawal_delta_of(settings[region]) < 0)
            return true;
    return false;
}

// Borrow the common settings of material-bearing regions, or return null if
// they differ. At least one mask must be present: protect_island() is called
// only after needs_collapse_protection() found a participating region.
const RegionSettings *shared_settings_of(const std::vector<RegionSettings> &settings,
                                         const RegionMasks &masks)
{
    const RegionSettings *shared = nullptr;
    for (std::size_t region = 0; region < settings.size(); ++region) {
        if (masks[region].empty())
            continue;
        const RegionSettings &current = settings[region];
        if (shared == nullptr)
            shared = &current;
        else if (current.outer_delta != shared->outer_delta ||
                 current.inner_delta != shared->inner_delta ||
                 current.minimum_contour_width_mm != shared->minimum_contour_width_mm ||
                 current.no_collapse != shared->no_collapse)
            return nullptr;
    }
    assert(shared != nullptr);
    return shared;
}

// Compute only the protected material; the caller merges it with the ordinary
// result. Call only when needs_collapse_protection(settings, masks) is true.
//
// The ordinary result can lose narrow walls entirely when material is removed. The
// protection (same idea as the legacy slicer) runs a second, careful path:
// 1. compose the island with only the material-adding part of each delta
//    ("positive" geometry);
// 2. remove material with the elephant foot routine, which keeps narrow
//    walls (protect_narrow_walls);
// 3. grow back what that routine removed beyond the region's own deltas
//    (the "regrow" deltas), and keep the result inside the positive geometry.
// Candidates are read-only inputs used to assign the positive regional claims.
// When all regions of the island share the same settings, this is done once
// for the whole island; otherwise each protected region only keeps the
// protected material inside its own part of the positive geometry.
StoredExPolygonCollection protect_island(storage_handle *storage,
                                         const ExPolygon &source,
                                         const std::vector<RegionSettings> &settings,
                                         const RegionMasks &masks,
                                         const std::vector<RegionCandidate> &candidates)
{
    // Build the island with only the material-adding part of each delta.
    std::vector<RegionSettings> growth_only_settings = settings;
    for (RegionSettings &region : growth_only_settings) {
        region.outer_delta = std::max<coord_t>(0, region.outer_delta);
        region.inner_delta = std::max<coord_t>(0, region.inner_delta);
    }
    StoredExPolygonCollection growth_only_geometry = compose_island(storage, source, growth_only_settings, masks);

    if (const RegionSettings *shared = shared_settings_of(settings, masks))
        return protect_uniform_island(storage, growth_only_geometry, *shared);
    return protect_regional_island(storage, growth_only_geometry, settings, masks, candidates);
}

// Compute the ordinary XY offsets, the ownership candidates for new material,
// then add any first-layer protection. Settings and masks use the same region order.
IslandResult process_island(storage_handle *storage,
                            const std::vector<RegionSettings> &settings,
                            const IslandInput &island)
{
    const RegionMasks masks = views_of(island.region_masks);
#ifndef NDEBUG
    validate_original_masks(storage, island.original, masks);
#endif

    IslandResult result(storage);
    result.geometry = compose_island(storage, island.original, settings, masks);
    result.candidates = positive_growth_candidates(storage, island.original, settings, masks);
    // The second calculation preserves thin details that the ordinary offsets
    // may remove. Only its returned material is merged into the main result.
    if (needs_collapse_protection(settings, masks)) {
        StoredExPolygonCollection protected_geometry = protect_island(
            storage, island.original, settings, masks, result.candidates);
        if (!protected_geometry.empty())
            result.geometry = union_two(storage, result.geometry, protected_geometry);
    }
    return result;
}

} // namespace

coord_t checked_scaled_delta(double millimeters)
{
    if (!std::isfinite(millimeters))
        throw std::invalid_argument("XY size compensation must be a finite number of millimeters.");
    const double scaled = scale_d(millimeters);
    const double limit = double(k_clipper_hi_range);
    if (!std::isfinite(scaled) || std::abs(scaled) > limit)
        throw std::out_of_range("XY size compensation exceeds Clipper coordinate limits.");
    const long long rounded = std::llround(scaled);
    const long long exact_limit = static_cast<long long>(k_clipper_hi_range);
    if (rounded < -exact_limit || rounded > exact_limit)
        throw std::out_of_range("XY size compensation exceeds Clipper coordinate limits.");
    return static_cast<coord_t>(rounded);
}

LayerOutput compensate_layer(storage_handle *storage,
                             const std::vector<RegionSettings> &settings,
                             const std::vector<IslandInput> &islands)
{
    const std::size_t region_count = settings.size();
    if (storage == nullptr)
        throw std::invalid_argument("XY compensation requires a valid SDK storage.");
    if (region_count == 0 && !islands.empty())
        throw std::runtime_error("XY compensation cannot assign islands without any layer regions.");

    // 1. Compensate each island on its own, and collect, per region, the
    //    original material of every island and every growth candidate.
    StoredExPolygonCollection compensated_islands(storage);
    std::vector<StoredExPolygonCollection> original_by_region;
    original_by_region.reserve(region_count);
    for (std::size_t region = 0; region < region_count; ++region)
        original_by_region.emplace_back(storage);
    std::vector<RegionCandidate> candidates;

    for (const IslandInput &island : islands) {
        if (island.region_masks.size() != region_count)
            throw std::runtime_error("XY compensation island region count does not match its layer.");
        IslandResult island_result = process_island(storage, settings, island);
        for (std::size_t region = 0; region < region_count; ++region)
            original_by_region[region].append_copy_from(island.region_masks[region]);
        candidates.insert(candidates.end(),
                          std::make_move_iterator(island_result.candidates.begin()),
                          std::make_move_iterator(island_result.candidates.end()));
        compensated_islands.append_move_from(island_result.geometry);
    }

    // 2. Merge the compensated islands. Growing islands may now touch or
    //    overlap, so this union is done on the whole layer.
    LayerOutput output(storage);
    output.slices = union_collections(storage, { compensated_islands.readonly() });

    // 3. Split the merged layer between regions.
    std::vector<StoredExPolygonCollection> normalized_original_masks;
    normalized_original_masks.reserve(region_count);
    for (const StoredExPolygonCollection &region : original_by_region) {
        normalized_original_masks.push_back(union_collections(
            storage, std::vector<ExPolygonCollection>{ region.readonly() }));
    }
    output.regions = assign_regions(storage, output.slices, views_of(normalized_original_masks),
                                    candidates, &output.fallback_area);

#ifndef NDEBUG
    // Debug-only: assignment covers the result by construction, and the host
    // checks region overlaps (with its tolerance) after each post-slicing plugin.
    StoredExPolygonCollection assigned_union = union_collections(storage, views_of(output.regions));
    StoredExPolygonCollection missing = difference(storage, output.slices, assigned_union);
    if (!missing.empty())
        throw std::runtime_error("XY compensation region assignment does not cover the result geometry.");
    for (std::size_t lhs = 0; lhs < output.regions.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < output.regions.size(); ++rhs) {
            if (!output.regions[lhs].empty() && !output.regions[rhs].empty() &&
                !intersect(storage, output.regions[lhs], output.regions[rhs]).empty())
                throw std::runtime_error("XY compensation assigned overlapping layer-region slices.");
        }
    }
#endif
    // No final ensure_valid(): every non-empty region comes from a Clipper
    // operation, and Clipper already removes points closer than SCALED_EPSILON,
    // as the host requires. Simplifying each region separately could also make
    // neighboring regions disagree along their shared border.
    return output;
}

}}} // namespace slic3r_api::XYSizeCompensationPlugin::detail
