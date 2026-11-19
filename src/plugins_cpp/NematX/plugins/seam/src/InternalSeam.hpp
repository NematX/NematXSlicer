// Copyright (c) 2026 NematX. AGPL-3.0-or-later.
#pragma once

#include <vector>
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

/*
Internal-perimeter attractor service
====================================
make_internal_seam_instance() decorates an already initialized host service.
Pass the same plan and assigned regions as for that service, plus temporary
plugin storage. The factory takes ownership of base (and clears it), even if
preparation fails. The returned C instance owns both sessions. Its immutable
geometry caches allow concurrent queries while the host keeps the borrowed
plan, loops, regions and configuration alive and unchanged.

    raw_seam_placer_instance base = ...; // initialized host Cost service
    auto instance = make_internal_seam_instance(base, plan, regions, storage);
    // The consumer ultimately calls instance.vtable->destroy(instance.session).

No source mesh, extrusion or configuration is modified. Attractors use the
center of their transformed layer cross-section, only for internal perimeter
geometry. Other loops and inactive layers return the host's answer unchanged.
*/
namespace nematx_seam {

// Initialize copied constraints/attractors, consume base, and publish one owned service.
raw_seam_placer_instance make_internal_seam_instance(
    raw_seam_placer_instance &base,
    const slic3r_api::PrintingPlan &plan,
    const std::vector<slic3r_api::PrintRegion> &regions,
    storage_handle *storage,
    const plugin_run_context *context = nullptr);

} // namespace nematx_seam
