# NematXSlicer Plugins

A collection of plugins for NematXSlicer.

## Plugins

- [nematx_second_layer_flow](doc/nematx_second_layer_flow.md)
- [nematx_corner_stretch](doc/nematx_corner_stretch.md)
- [nematx_infill_aroundholes](plugins/infill_aroundholes/README.md)
- [nematx_infill_gridvarspeed](plugins/infill_gridvarspeed/README.md)
- [nematx-firmware](plugins/nematx_firmware/README.md): NematX G-code firmware and reader
- [nematx_hole_size_curve](plugins/hole_size_curve/README.md): hole compensation by diameter
- [nematx_regional_fan](plugins/regional_fan/README.md): fan speed overrides per region and role
- [nematx_fan_first_layer_program](plugins/fan_first_layer_program/README.md): first-layer fan program with enabled, disabled and ramped layers
- [nematx_fan_speedup](plugins/fan_speedup/README.md): fan anticipation and kickstart (`fan_speedup_time`, `fan_kickstart`)

## Build targets

The packaging helper follows the target toolchain and provides output directories for Windows x86_64 and ARM64, Linux x86_64 and ARM64, and macOS ARM64. The ZIP filename stays compatible with the host plugin repository; binaries and ZIPs are separated by target OS, architecture, and build configuration. Only Windows x86_64 has been built and tested in this workspace. The other four combinations are supported by the CMake layout but have not been validated here.
