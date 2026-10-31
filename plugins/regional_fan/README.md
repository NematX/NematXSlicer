# NematX regional fan overrides

Package `nematx-regional-fan`; plugin `nematx_regional_fan`.

Four region settings force the fan speed of the paths printed in a region, usually through a modifier. They are percentages that can be disabled, disabled by default:

| Setting | Paths |
| --- | --- |
| `print_fan_speed` | every deposited path of the region |
| `print_perimeters_fan_speed` | internal and external perimeters (not overhang perimeters or thin walls) |
| `print_solid_infill_fan_speed` | solid and top solid infill, ironing |
| `print_bridge_fan_speed` | bridge and internal bridge infill |

A role-specific setting wins over `print_fan_speed`. An enabled 0% stops the fan; a disabled setting leaves the path to the standard fan settings.

## Behavior

- Each value becomes an Override fan request on the path. The plugin runs at the regional fan priority (18), before the host's standard fan (20), which only fills the paths left without a request. The short-layer cooling, the first-layer program and the dynamic overhang fan never change an Override, including 0%.
- A request already set before the plugin is kept when it is an Override or belongs to another plugin's mode; an Automatic request is replaced.
- A path crossing regions with different values is split at the region boundary, each part taking its region's value. The region masks follow the offset of each print instance.
- Skirt, brim and support use the default region configuration of their object (for the print-level skirt and brim, the print's values), without splitting.
- Travel, wipe tower and milling paths, empty event leaves and paths whose E is replaced by a raw `EXTRUSION` command (the zero-flow connectors of the around-holes infill) get no request: they keep the fan state of their neighbours.
- A split copies the path's properties into every part. Splitting a path that carries a one-shot event (custom G-code, special command, heater, extrusion axis) rejects the export instead of repeating the event.

## Differences with the legacy slicer

- The legacy slicer kept the last override active until another one replaced it, so an override could leak into the next paths of another role or region. Here each path gets the value of its own role and region only.
- The legacy cooling buffer clamped the override between the default fan speed raised by the short-layer cooling and the first-layer limit (0% on the layers without fan). Here the override is kept as set on every layer, including 0%: short-layer cooling, first layers and dynamic overhang fan do not change it.
- The legacy slicer split the infill of regions with different overrides when generating it; here any path is split at the region boundary after ordering.

## Tests

SDK tests run the tree pass on stored entities against the host library: role choice, protected requests, excluded connectors, splits at a region boundary and the rejection of a split that would repeat a heater event.

The end-to-end checks export generated models through the host executable and read the fan value active during every extrusion move:

- `print_fan_speed` holds on the first layer and on short layers, including the skirt;
- per-role values on a plate over a pillar (perimeters, overhang perimeters, bridge infill at 0%, solid, top solid and sparse infill);
- an object of two regions printed twice: no path crosses the boundary and each side of each instance has its own value;
- the brim keeps the print's value while the object uses its volume override;
- with the around-holes infill at 0% connector flow, the fan stays at the override value along the infill and its connectors.

```
cmake --build .build --config Debug --target nematx_regional_fan nematx_infill_aroundholes regional_fan_sdk_tests
ctest --test-dir .build -C Debug -R regional_fan
python plugins/regional_fan/tests/test_regional_fan.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin"
```

`--runtime-path` adds directories to `PATH` for the host's DLL dependencies. The around-holes check copies both staged packages into one datadir.
