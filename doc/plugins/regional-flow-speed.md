# Regional flow and speed

`layer_extrusion_edit.first_layer_flow` applies both `first_layer_flow_ratio`
and the tool's `filament_first_layer_flow_ratio` before the default speed editor.
It edits deposited mm3/mm flow on source layers whose bottom is at Z zero.
Volumetric speed limits then consume that final flow, without multiplying either
ratio a second time. A non-positive ratio fails visibly rather than emitting
zero-flow paths.

Ordinary paths crossing different regional ratios split at the regional border.
Skirt, brim and support use their object's default region instead, even when all
material regions share another ratio. Print-level auxiliary paths use Print
defaults. Stationary deposits select one ratio at their point without splitting
or duplicating their command. Travel, wipe, retraction, milling, wipe-tower work
and paths already expressed in E/mm are unchanged. Other inherited attributes
and process fields are preserved.

After ordering validation, every region of a printing extrusion's source island
must belong to the same layer and object. Editors may therefore resolve the
source layer through any region. Region-less service work stays outside this
first-layer flow pass.

The default speed provider publishes `DefaultSpeedLimits` on the first-layer
leaves it resolves. Percentages use the original role or per-flow autospeed base,
before the first-layer minimum and physical print/filament caps. A zero
`first_layer_infill_speed` (absolute or percent) falls back to `first_layer_speed`.
Late editors read the absolute ceiling instead of applying a percentage to an
already capped speed. No metadata is invented for an explicit upstream override.
Equal final speeds with different ceilings remain distinct regional fragments;
stationary deposits choose one speed/ceiling pair without duplicating a command.

An explicit zero (including `0%`) selects autospeed for external perimeters,
gap fill, thin walls, top solid infill, internal bridges, ordinary infill and
overhangs. It no longer inherits the parent role's explicit speed. These roles
participate in the autospeed minimum-flow scan unless the thin-flow setting
excludes them. Without an autospeed target, zero falls back to `max_print_speed`.
Percentages of a zero parent use the autospeed base when one is available,
including ironing over a zero top-solid speed. First-layer and filament caps
still apply after role resolution. Existing profiles using zero to inherit a
parent speed must use `100%` explicitly to retain that behavior.
