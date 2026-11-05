///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Support families publish auxiliary surfaces for each object. Pattern callbacks share a lazy step-owned service;
   the final host pass aligns support-layer Z values before toolpath generation.
   register_selector_traits installs the presentation defaults for support groups at host startup. */
#ifndef steps_stepgeneratesupport_hpp_
#define steps_stepgeneratesupport_hpp_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"
#include "libslic3r/Plugins/Support/Common/SupportCloseZ.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r {
class Layer;
class Orchestrator;
class Print;

namespace Steps::StepGenerateSupport {

// Declare presentation defaults for support groups without selecting a whole-step provider.
void register_selector_traits(Orchestrator &orchestrator);

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
// Run active support families on each object using the demand stored in its islands. Then keep the print z of the auxiliary (support)
// layers more than the close-Z tolerance (at least 4 * SCALED_EPSILON, max(z_step / 2, 0.5 um)) apart from the other z of the print (the object
// layers and each other), as the tool ordering and the print plan require: one pass per object when the print is printed object by object, one for
// the whole print otherwise. The precondition is that the object layers are already that far apart (two object layers of different objects closer
// than that are a native case, out of scope).
void run_step(Orchestrator &orchestrator, Print &print);

namespace Detail {
// The sorted (source, target) pairs of the movable values that move. A value that is not listed maps to itself.
using CloseZMapping = std::vector<std::pair<coord_t, coord_t>>;
// Merges the movable z onto the fixed z and onto each other: a movable value within the tolerance (inclusive) of a fixed one takes the
// nearest fixed one (the lower on a tie); the other movable values are swept upwards, each one within that distance of the last kept one takes
// it. Inputs are scaled print z (1 unit = 1 nm), unsorted, repeats allowed. The tolerance is at least 4 * SCALED_EPSILON (0.5 um, the default):
// the pass computes it as Common::close_z_tolerance() of the Z step, max(z_step / 2, 0.5 um). Precondition: the fixed values are more than the
// tolerance apart. Result: distinct results (fixed values included) are more than the tolerance apart and no value moves by more than that.
CloseZMapping close_z_mapping(std::vector<coord_t> fixed, std::vector<coord_t> movable,
                              coord_t tolerance = slic3r_api::Support::Common::min_close_z_tolerance);
// The target of z in a mapping, z itself when it is not listed.
coord_t close_z_target(const CloseZMapping &mapping, coord_t z);
// Sorts support layers by print_z ascending, the thickest first at an equal print_z (the order of the publication).
void sort_support_layers(std::vector<std::unique_ptr<Layer>> &layers);
} // namespace Detail

} // namespace Steps::StepGenerateSupport
} // namespace Slic3r

#endif // steps_stepgeneratesupport_hpp_
