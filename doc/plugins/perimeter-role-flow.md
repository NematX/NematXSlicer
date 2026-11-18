# Perimeter role and flow

The classic and simple generators, including Python simple, give shell zero
the external-perimeter role and flow. Deeper shells use the internal-perimeter
role, flow, width and spacing. Thus existing profiles with distinct internal
settings now apply those settings to the corresponding shells instead of using
external speed or flow for every loop.

The closed-loop wrapper carries the shell index and LOOP/HOLE flags. Its child
path owns extrusion attributes. Overhang detection can split the path into open
fragments without falsely marking each fragment as a complete closed loop.
The direction of each loop is set afterwards by the loop direction post-process
(see perimeter-loop-direction.md).

Generator preference is unchanged: simple remains the default, classic retains
priority 5 and Arachne priority 10 within the exclusive group. Explicit stored
selections remain in effect.
