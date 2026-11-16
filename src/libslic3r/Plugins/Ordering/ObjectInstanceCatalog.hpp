///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_Ordering_ObjectInstanceCatalog_hpp_
#define slic3r_Plugins_Ordering_ObjectInstanceCatalog_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_printing_plan.h"

namespace slic3r_api { namespace Ordering { namespace ObjectInstanceCatalogPlugin {

/*
PrintingPlan object-instance catalog
====================================

The ordering plan records object-instance IDs while its groups and extrusion
trees are built. This provider completes the descriptive metadata for those
entries through the public plugin API. Registration is append-only: the
provider never assigns IDs, freezes the catalog, or retains data-tree views.

Physical instances keep their source Object and local instance index. Synthetic
entries, used by work that has no physical source instance, keep only a copied
name and projected outline. The plan owns the copied metadata; source views are
borrowed only for the duration of this call.
Physical-copy names are unique after firmware sanitization. Only colliding names
receive catalog-ID suffixes; existing unique names and all IDs remain unchanged.
*/
// Complete copied catalog metadata and disambiguate colliding physical-copy names.
// Borrow Print/plan/storage only during the call and preserve original unique names,
// IDs and outlines. Null handles, invalid sources or rejected edits throw; the plan
// owns copied metadata and no borrowed views escape.
void populate_object_instance_catalog(
    const print_handle *print, printing_plan_handle *plan, storage_handle *storage);

void register_object_instance_catalog_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::Ordering::ObjectInstanceCatalogPlugin

#endif // slic3r_Plugins_Ordering_ObjectInstanceCatalog_hpp_
