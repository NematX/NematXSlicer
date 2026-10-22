# NematXSlicer Plugins

A collection of plugins for NematXSlicer.

## Build targets

The packaging helper follows the target toolchain and provides output directories for Windows x86_64 and ARM64, Linux x86_64 and ARM64, and macOS ARM64. The ZIP filename stays compatible with the host plugin repository; binaries and ZIPs are separated by target OS, architecture, and build configuration. Only Windows x86_64 has been built and tested in this workspace. The other four combinations are supported by the CMake layout but have not been validated here.
