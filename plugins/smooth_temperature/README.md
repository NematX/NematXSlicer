# NematX temperature smoothing

Package `nematx-smooth-temperature`; plugin `nematx_smooth_temperature`.

The region setting `print_temperature_smooth_change` (N layers, default 0) spreads a change of the general nozzle temperature of each object over N layers instead of applying it at once. A change restarts the ramp from the value printed on the previous layer; the layer of the change already takes the first step and the new temperature is reached on the N-th layer. Values are rounded to the nearest degree, halves towards the target. N = 0 or 1 applies changes at once.

Example with N = 2: the first layer asks for 230 and prints 230; the second asks for 210 and prints 220; the third asks for 180 and prints 200; the fourth prints 180.

## Behavior

- The plugin is a member of the host's `layer_extrusion_edit.temperature` group, preferred to the host provider `layer_extrusion_edit.temperature.default` when both are active. It derives from that provider: the host planner computes every heater request, then the plugin rewrites the nozzle setpoints before they are written. The standalone package compiles the planner's two host sources, `DefaultTemperature.cpp` and `ExtrusionProcessParameterHelpers.cpp`.
- Each object instance has one ramp per tool. The temperature asked by all the regions of an instance on a layer is general: object settings, height ranges, or a modifier covering the whole instance on that layer. When the regions disagree, the general temperature comes from the object settings and the height range of the layer, and the other temperatures, for example those of a partial modifier, are overrides: they apply at once and the ramp continues underneath, as if they did not exist.
- N is read with the general temperature: from the regions on a layer where they agree, otherwise from the object settings and the height range.
- Setting N to 0 or 1 cancels an active ramp immediately, even when its target has not changed.
- The first layer of an instance applies its temperature at once, also with complete objects.
- A tool's ramp waits on the layers where the tool does not print the instance. With ooze prevention, the standby temperature is an override: a returning tool is heated to its next ramp value, with the wait decided by the host.
- The plugin adds no wait; the host waits stay, on the ramp value. Ramp steps never wait.
- With N <= 1 everywhere, the G-code is the host provider's.

## Limitations

- No smoothing with a single extruder multi-material printer: the wipe tower resets the shared heater there.
- When the host provider is explicitly selected in the layer temperature group, `print_temperature_smooth_change` is ignored without a message.
- Supports, skirt, brim and the wipe tower have no region: they keep the current setpoint, as with the host provider.
- Several instances at different temperatures on one layer switch the setpoint, without waiting, at each change of instance, as with the host provider.

## Differences with the legacy slicer

- The legacy slicer kept the layer of a change at the previous value, wrote N intermediate values and reached the target on the next layer, with a floating-point step and truncation. Here the change layer takes the first step and the target is reached on the N-th layer; N = 1 no longer smooths.
- The legacy ramp was per tool; here it is per object instance and tool, and modifier overrides do not restart it.
- The legacy first layer override was delayed by the ramp; here the first layer of an instance applies its temperature at once.
- In the current legacy slicer, the ramp only advanced with an active wipe tower; this plugin follows the original intent.

## Tests

Kernel tests check the ramp values: the example above, rounding up and down, a difference smaller than N, N = 0 and 1, and a restart during a ramp. The end-to-end checks export boxes through the host executable and read the setpoint during the extrusions of each layer, tool and object: height ranges with several ramp lengths, N set in one range, the host provider and N <= 1, partial and covering modifiers, two objects printed together or one by one, and two extruders, with and without ooze prevention.

```
cmake --build .build --config Debug --target nematx_smooth_temperature smooth_temperature_tests
ctest --test-dir .build -C Debug -R smooth_temperature
python plugins/smooth_temperature/tests/test_smooth_temperature.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin"
```
