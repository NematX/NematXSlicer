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
