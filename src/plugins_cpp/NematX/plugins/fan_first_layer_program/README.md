# NematX first-layer fan program

Package `nematx-fan-first-layer-program`; plugin `nematx_fan_first_layer_program`.

The legacy slicer controlled the fan of the first layers with three filament settings, per extruder:

| Setting | Owner | Meaning |
| --- | --- | --- |
| `enable_fan_first_layers` (E, default 0) | this plugin | the fan runs normally on the first E layers |
| `disable_fan_first_layers` (D) | host | then it stops for D layers |
| `fan_speed_layer_gradient` (G, default -1) | this plugin | the changes are ramped over G layers |

With k the layer number from 0: normal fan while k < E; from E, ramp down over at most G layers (factor 1 - (k + 1 - E) / (min(G, D - 1) + 1)); off until E + D; from E + D, ramp up over G layers (factor (k + 1 - (E + D)) / (G + 1)); then normal. A ramp scales the rampable roles only (not bridges, overhangs, support interfaces, skirt), even at factor 0; off stops every role. These are the conditions of the legacy cooling buffer, including its overlaps when G is larger than D.

## Behavior

- The plugin only publishes the program of each layer and tool, at the fan program priority (23). The host first-layer stage (24) applies it to the automatic requests and references; an Override, or a request in another plugin's mode, is kept.
- With E = 0 and G = -1, the defaults, nothing is published: the host keeps its program from `disable_fan_first_layers` and `full_fan_speed_layer`. With E > 0 and G = -1 the gradient is 4 layers, the legacy default. A published program replaces `full_fan_speed_layer`.
- The layer number is the index of the layer in its printing group, as for the host stage.

## Differences with the legacy slicer

- The legacy default gradient was 4 layers, and its import converted `full_fan_speed_layer` into a gradient. Here G defaults to -1 so that existing profiles keep the host program; profiles that set E or G keep the legacy program.
- When the default fan speed is disabled, the dynamic overhang ceiling during a ramp is the ramped 100%, not the disabled value.

## Tests

Kernel tests check the program on each layer: enabled then disabled layers, a gradient longer than the disabled layers, G = 0, D = 0, the legacy defaults and the -1 sentinel. The end-to-end checks export boxes through the host executable and read the fan during the first perimeter of each layer and tool, including two extruders with different programs.

```
cmake --build .build --config Debug --target nematx_fan_first_layer_program fan_first_layer_program_tests
ctest --test-dir .build -C Debug -R fan_first_layer_program
python plugins/fan_first_layer_program/tests/test_fan_first_layer_program.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin" [--legacy-executable "C:/path/to/legacy/nematxslicer_console.exe"]
```

With `--legacy-executable`, the cases using the legacy settings are also exported by the legacy slicer and must give the same values.
