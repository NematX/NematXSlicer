/*
 * REST configuration catalog projection.
 *
 * The catalog exposes finalized libslic3r option definitions as stable JSON
 * metadata without exposing the mutable configuration objects themselves.
 */

#pragma once

#include <nlohmann/json_fwd.hpp>

namespace Slic3r::Rest {

// True once PrintConfigDef::finalize() has run, meaning the catalog can be built.
bool settings_catalog_ready() noexcept;
// Build the catalog. Throws std::logic_error when the definitions are not
// finalized; the route answers 503 before calling it.
nlohmann::json settings_catalog();

} // namespace Slic3r::Rest
