# NematXSlicer Plugins

A collection of plugins for NematXSlicer.

## Plugins

- [nematx_top_surface_retraction](plugins/top_surface_retraction/README.md): travel retraction over exposed top areas

- [nematx_second_layer_flow](doc/nematx_second_layer_flow.md)
- [nematx_corner_stretch](doc/nematx_corner_stretch.md)
- [nematx_infill_aroundholes](plugins/infill_aroundholes/README.md)
- [nematx_infill_gridvarspeed](plugins/infill_gridvarspeed/README.md)
- [nematx-firmware](plugins/nematx_firmware/README.md): NematX G-code firmware and reader
- [nematx_hole_size_curve](plugins/hole_size_curve/README.md): hole compensation by diameter
- [nematx_regional_fan](plugins/regional_fan/README.md): fan speed overrides per region and role
- [nematx_fan_first_layer_program](plugins/fan_first_layer_program/README.md): first-layer fan program with enabled, disabled and ramped layers
- [nematx_fan_speedup](plugins/fan_speedup/README.md): fan anticipation and kickstart (`fan_speedup_time`, `fan_kickstart`)
- [nematx_smooth_temperature](plugins/smooth_temperature/README.md): general nozzle temperature changes spread over several layers (`print_temperature_smooth_change`)
- [nematx-wipetower](plugins/wipetower/README.md): wipe tower manager of the NematX printers (`step_wipetower_plugin = nematx_wipetower`)
- [nematx-support-interface-perimeters](plugins/support_interface_perimeters/README.md): configurable contour loops around support contacts and interfaces

## Build targets

The packaging helper follows the target toolchain and provides output directories for Windows x86_64 and ARM64, Linux x86_64 and ARM64, and macOS ARM64. The ZIP filename stays compatible with the host plugin repository; binaries and ZIPs are separated by target OS, architecture, and build configuration. Only Windows x86_64 has been built and tested in this workspace. The other four combinations are supported by the CMake layout but have not been validated here.

## Cross-package tests

`tests/fan_chain/test_fan_chain.py` exports one generated model through the three fan packages (regional fan, first-layer program, anticipation and kickstart) with the host fan policies, and compares it with the same export without `nematx-fan-speedup`: Override region keeping its value except during the 100% pulses of kicks (a pulse may cover a whole short sequence; the region may start up to `fan_speedup_time` earlier), instants compared with the plugin's 0.1 s placement tolerance, first-layer program and short-layer cooling outside it, kicks followed by their target, extruded volume, layers and geometry kept, deterministic exports. It builds its datadirs from the ones prepared by the build:

```
cmake --build .build --config Debug -- /m
python tests/fan_chain/test_fan_chain.py --host-executable "C:/path/to/build/src/Debug/Slic3r_console.exe" --runtime-path "C:/path/to/deps/usr/local/bin"
```
