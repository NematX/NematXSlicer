///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2021 Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GCode_PostProcessor_hpp_
#define slic3r_GCode_PostProcessor_hpp_

#include <string>

#include <boost/filesystem.hpp>

#include "libslic3r/libslic3r.h"

namespace Slic3r {

class DynamicPrintConfig;

// Run the post-processing scripts of the `post_process` option on a temporary copy of src_path, which is left untouched.
// Returns the path of the processed copy ("<src_path>.pp"), which the caller must publish or delete,
// or an empty string when no script is configured (nothing is read or written then).
// Throws an exception on error, after removing the copy.
// The scripts receive the path of the copy as their last argument and the configuration as SLIC3R_* environment variables
// (only the keys that a project file may contain, see ConfigBase::setenv_()).
// The call waits for every script to end; it cannot be cancelled.
extern std::string run_post_process_scripts(const std::string &src_path, const DynamicPrintConfig &config);

} // namespace Slic3r

#endif /* slic3r_GCode_PostProcessor_hpp_ */
