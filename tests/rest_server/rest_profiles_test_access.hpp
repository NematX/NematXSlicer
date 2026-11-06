/*
 * Test access to the profile catalog internals.
 */

#pragma once

#include "profiles.hpp"

namespace Slic3r::Rest {

struct ProfileCatalogTestAccess {
    static const PresetBundle &bundle(const ProfileCatalog &catalog) { return catalog.bundle_; }
    // Full catalog copies are the reference behavior for reduced bundles.
    static void set_full_bundle_copies(ProfileCatalog &catalog, bool enabled) { catalog.full_bundle_copies_ = enabled; }
};

} // namespace Slic3r::Rest
