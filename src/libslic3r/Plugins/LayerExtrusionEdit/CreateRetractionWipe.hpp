///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_CreateRetractionWipe_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_CreateRetractionWipe_hpp_

/*
Retraction-wipe creation for compact PrintingPlan scopes
========================================================

CreateRetractionWipe runs after semantic retractions have been placed and
before layer entry state and travels are generated. It converts part of an
existing Retract request into a fast movement over the source scope's freshly
printed path. Open paths are retraced; a final perimeter loop continues from
its seam in the loop direction.

Prerequisites
-------------

- CreateTransitionScope has marked non-nested, ordered printable scopes.
- CreateRetraction has placed any eligible Retract in source.after and the
  matching Unretract in target.before.
- Scope content is final, non-sortable and non-reversible.

Mutations and guarantees
------------------------

The plugin reads printable geometry only from source.content. It inserts Wipe
and any remaining E-only Retract into source.after. An optional inside-start
approach turns the existing target Unretract into geometry inside
target.before. It never changes the marked scope root, the phase order, or the
printable content.

CreateCrossingWipe runs after travel generation and before lift. When
wipe_only_crossing is enabled, the first pass wipes within a layer when exactly
one side is outside object material, or when both sides are object material on
different instances or slice islands. Object material excludes global,
regionless and auxiliary-layer geometry.
These normal wipes retain wipe_return and inside-end behavior. Other local gaps
and inter-layer gaps leave outgoing retractions for the second pass, whose
decision depends on the generated travel.
The second pass tests a generated travel against target perimeter boundaries only
when its phase contains a single segmented Travel leaf, not a composite tree.
It adds a returning wipe only when its source start agrees with the travel start
within SCALED_EPSILON in XY distance and absolute Z. Short loops use whole turns;
longer loops and open paths go out and back. Inside-end and wipe_return do not
control this second pass. Existing departure geometry and terminal deposits prevent it.

Layer workers publish plain first-travel summaries before a barrier. A source
worker reads the next non-empty layer's summary, never its geometry, and edits
only its own departure. Crossing inputs and caches belong to individual calls.

After CreateRetractionWipe, DefaultLayerEntryState observes the position left by
the wipe. The selected travel provider starts there and aims at an inside-start
approach when one exists. A later lift provider may add Z motion without
changing the semantic E-axis request. CreateCrossingWipe runs after the entry
state has been stored. It returns to its source start, checked against the travel
start with the tolerance above; travel geometry and stored entry state are unchanged.
*/

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CreateRetractionWipePlugin {

/* Register the built-in provider which distributes retraction over wipe motion. */
void register_create_retraction_wipe_plugin(orchestrator_handle *orchestrator);

/* Register the independent travel-based returning wipe provider. */
void register_create_crossing_wipe_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin

#endif // slic3r_Plugins_LayerExtrusionEdit_CreateRetractionWipe_hpp_
