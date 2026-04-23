# NematXSlicer — Agent Architecture Reference

This document is written for AI agents. It captures the complete architectural picture of this codebase so you can orient yourself without re-reading all the source files. Read this first before exploring any subsystem.

---

## 1. Project Identity

| Key | Value |
|-----|-------|
| Name | **NematXSlicer** |
| Origin | Fork of SuperSlicer, customized for NematX printers and firmware |
| Version | `2.7.63-beta` (authoritative source: `version.inc`) |
| App key | `NematXSlicer` (registry / filesystem key) |
| CLI name | `nematxslicer` |
| Windows GUID | `ee58a199-3c77-4865-bbe9-52b9cbb2bc32` |
| GitHub | `NematX/NematXSlicer` |
| Language | C++17 |
| Build system | CMake 3.13+ |

---

## 2. Repository Layout

```
slicer/
├── src/                  # All C++ source code
│   ├── libslic3r/        # Core slicing engine (no GUI dependency)
│   ├── slic3r/           # GUI application (wxWidgets)
│   ├── admesh/           # STL mesh repair library (bundled)
│   ├── agg/              # Anti-Grain Geometry renderer (bundled)
│   ├── angelscript/      # Scripting engine (bundled)
│   ├── boost/            # Boost.Nowide Unicode shim (bundled)
│   ├── clipper/          # Polygon clipping (bundled)
│   ├── eigen/            # Linear algebra (bundled header-only)
│   ├── imgui/            # Immediate-mode GUI overlay (bundled)
│   ├── libigl/           # Geometry processing (bundled)
│   ├── libnest2d/        # 2D bin-packing (bundled)
│   ├── avrdude/          # AVR firmware upload (bundled)
│   ├── hidapi/           # USB HID access (bundled)
│   ├── miniz/            # ZIP/zlib (bundled)
│   ├── glu-libtess/      # OpenGL tessellation (bundled)
│   └── PrusaSlicer.cpp   # CLI entry point
│
├── deps/                 # Dependency build system (separate CMake project)
│   ├── CMakeLists.txt    # Downloads and builds all third-party libs
│   └── autobuild.cmake   # Preset-based automation
│
├── tests/                # Catch2 unit tests
│   ├── libslic3r/        # Core library tests (42 files)
│   ├── fff_print/        # FFF pipeline tests (27 files)
│   ├── sla_print/        # SLA pipeline tests
│   ├── arrange/          # Arrangement tests
│   ├── thumbnails/       # Thumbnail generation tests
│   ├── slic3rutils/      # GUI utility tests
│   └── data/             # Test input models and configs (CC-By-SA 3.0)
│
├── resources/            # Runtime assets (copied to build output)
│   ├── profiles/         # Vendor printer/filament/print bundles (INI)
│   ├── icons/            # 200+ SVG UI icons
│   ├── shaders/          # GLSL shaders for 3D rendering
│   ├── fonts/            # NotoSans TTF/TTC
│   ├── localization/     # .po translation files (20+ languages)
│   ├── calibration/      # Calibration test prints + HTML guides
│   └── ui_layout/        # UI panel layout definitions
│
├── sandboxes/            # Experimental/demo targets (not production)
├── doc/                  # Build guides, profile authoring guide, localization
├── cmake/                # Custom CMake modules and find scripts
├── .github/workflows/    # 11 CI/CD workflow files
│
├── CMakeLists.txt        # Main CMake project
├── CMakePresets.json     # Build presets
├── version.inc           # Single source of truth for version strings
├── build_win.bat         # Windows build script
├── BuildLinux.sh         # Linux build script
├── BuildMacOS.sh         # macOS build script
└── create_release.py     # Download and package GitHub Actions artifacts
```

---

## 3. Build System

### CMake Presets (`CMakePresets.json`)

| Preset | Binary Dir | Key Difference |
|--------|-----------|----------------|
| `default` | `build-default/` | Static linking, all features including STEP (OCCT) |
| `no-occt` | `build-no-occt/` | Static, but STEP file support disabled |
| `shareddeps` | `shareddeps/` | Dynamic system libraries |

### Important CMake Options

| Option | Default | Effect |
|--------|---------|--------|
| `SLIC3R_GUI` | ON | Build wxWidgets GUI; disabling gives a CLI-only build |
| `SLIC3R_STATIC` | ON (Win/macOS), OFF (Linux) | Link dependencies statically |
| `SLIC3R_BUILD_TESTS` | OFF | Enable Catch2 test targets |
| `SLIC3R_BUILD_SANDBOXES` | OFF | Enable sandbox experiment targets |
| `SLIC3R_ENABLE_FORMAT_STEP` | ON | Include STEP file support via OpenCASCADE |
| `SLIC3R_PCH` | ON | Precompiled headers (speeds up compilation) |
| `SLIC3R_FHS` | OFF | Unix FHS-compliant installation layout |

### Build Targets

| Target | Output | Role |
|--------|--------|------|
| `Slic3r` | `Slic3r.dll` (Win) / `libSlic3r.so` (Unix) | Core shared library |
| `Slic3r_app_gui` | `nematxslicer.exe` | GUI application (no console) |
| `Slic3r_app_console` | `nematxslicer_console.exe` | GUI application with console |
| `PrusaSlicer_app_gcodeviewer` | `nematxslicer-gcodeviewer.exe` | Standalone G-code viewer |
| `libslic3r` | static lib | Core slicing engine |
| `libslic3r_gui` | static lib | GUI widgets and dialogs |

### Dependency Build

Dependencies are a **separate CMake project** under `deps/`. Build them first:

```bash
# Windows (example)
build_win.bat -d "C:\nematx-deps" -s all

# Linux
./BuildLinux.sh -d    # builds deps only
./BuildLinux.sh -s    # builds slicer only
./BuildLinux.sh -dsi  # deps + slicer + AppImage

# macOS (ARM)
./BuildMacOS.sh -d -s -i -a
```

Deps output lands in `deps/build/destdir/usr/local/`. The main CMake project locates them via `CMAKE_PREFIX_PATH`.

---

## 4. Source Architecture

### 4a. Core Library: `src/libslic3r/`

This is the engine. It has **no GUI dependency** and can be used headlessly.

#### Geometry Primitives

| File | Purpose |
|------|---------|
| `Point.hpp` | 2D/3D point types (`Point`, `Vec3d`, etc.) |
| `Polygon.hpp`, `ExPolygon.hpp` | Polygon and polygon-with-holes types |
| `Line.hpp` | Line segment |
| `BoundingBox.hpp/.cpp` | AABB in 2D and 3D |
| `TriangleMesh.hpp/.cpp` (126 KB) | 3D mesh: load, repair, transform, slice |
| `TriangleMeshSlicer.cpp` | Clips triangles at Z heights → 2D layer polygons |
| `ClipperUtils.hpp/.cpp` | All 2D polygon boolean ops and offsetting (wraps Clipper) |

#### Model Tree

```
Model
  └─ ModelMaterial[]          (material definitions)
  └─ ModelObject[]
       └─ ModelVolume[]       (mesh volumes, modifiers, supports)
       └─ ModelInstance[]     (placement on the bed)
```

Files: `Model.hpp/.cpp`

#### Configuration System

The config system is **hierarchical and type-safe**. Every option is declared once with metadata.

```
ConfigBase (abstract)
  └─ DynamicConfig            (runtime key-value, any options)
       └─ DynamicPrintConfig  (print-specific options)

StaticPrintConfig (compile-time fixed option set)
  └─ PrintConfig              (machine / printer parameters)
  └─ PrintObjectConfig        (per-object slicing params)
  └─ PrintRegionConfig        (per-region material/extrusion params)
  └─ FullPrintConfig          (all of the above merged)

AppConfig                     (application preferences, INI-backed)
```

Files: `Config.hpp/.cpp` (163 KB header), `PrintConfig.hpp`, `AppConfig.hpp/.cpp`

To **add a new config option**: declare it in `PrintConfig.hpp` using the option macro system. The option is then automatically available in the UI, CLI, and profile serialization.

#### Slicing Data Model

```
Print
  ├─ PrintConfig              (global settings)
  ├─ PrintRegion[]            (unique config combos across all objects)
  └─ PrintObject[]            (one per ModelInstance)
       ├─ Layer[]
       │    └─ LayerRegion[]  (one per PrintRegion)
       │         ├─ SurfaceCollection  slices       (raw slice polygons)
       │         ├─ SurfaceCollection  fill_surfaces (classified surfaces)
       │         └─ ExtrusionEntityCollection perimeters / infill / support
       └─ SupportLayer[]
```

Files: `Print.hpp/.cpp`, `Layer.hpp/.cpp`, `Surface.hpp/.cpp`, `SurfaceCollection.hpp/.cpp`

#### Extrusion System

```
ExtrusionEntity (abstract, visitor pattern)
  ├─ ExtrusionPath            (single continuous path)
  ├─ ExtrusionMultiPath       (multiple connected paths)
  ├─ ExtrusionLoop            (closed loop)
  └─ ExtrusionEntityCollection (container, can nest)
```

Each entity carries an `ExtrusionRole` enum (perimeter, external perimeter, infill, solid infill, support, bridge, etc.) and flow parameters.

Files: `ExtrusionEntity.hpp/.cpp`, `ExtrusionRole.hpp/.cpp`, `Flow.hpp/.cpp`

#### Infill Patterns (`src/libslic3r/Fill/`)

| File | Pattern |
|------|---------|
| `FillBase.hpp/.cpp` | Abstract base + polygon tiling |
| `FillRectilinear.cpp` (249 KB) | Grid / rectilinear / aligned / stars / cubic |
| `FillAdaptive.cpp` | Adaptive density (stress-based) |
| `FillGyroid.cpp` | Gyroid surface |
| `FillHoneycomb.cpp` | Honeycomb |
| `FillConcentric.cpp` | Concentric rings |
| `FillLine.cpp` | Simple lines |
| `FillSmooth*.cpp` | Smooth variants |
| `Lightning/` | Lightning infill (tree-like for fast sparse fills) |

To **add a new infill pattern**: subclass `FillBase`, implement `fill_surface()`, register in `Fill.cpp`'s factory function, and add an enum value to the infill type option in `PrintConfig.hpp`.

#### Wall Generation: `src/libslic3r/Arachne/`

Implements the Arachne algorithm for variable-width walls:

| File | Role |
|------|------|
| `SkeletalTrapezoidation.hpp/.cpp` (96 KB) | Medial axis decomposition of the polygon |
| `WallToolPaths.hpp/.cpp` (43 KB) | Convert skeleton to printable wall paths |
| `BeadingStrategy/` | Width distribution strategies along the skeleton |

#### G-Code Generation

The G-code subsystem is the largest and most complex:

| File | Role |
|------|------|
| `GCode.hpp/.cpp` (638 KB impl, 41 KB header) | Main orchestrator; state machine over all layers/objects |
| `GCode/GCodeWriter.hpp` | Low-level G-code command emission |
| `GCode/ToolOrdering.hpp` | Multi-extruder tool-change sequencing |
| `GCode/WipeTower.hpp`, `WipeTower2.hpp` | Wipe tower algorithms |
| `GCode/RetractWhenCrossingPerimeters.hpp` | Retraction suppression inside perimeters |
| `GCode/AvoidCrossingPerimeters.hpp` | Travel path planning (avoid crossing printed walls) |
| `GCode/CoolingBuffer.hpp` | Insert cooling time (slow-down / fan control) |
| `GCode/PressureAdvance.hpp` | Pressure advance macro generation |
| `GCode/SeamPlacer.hpp` | Seam placement optimization |
| `GCode/GCodeProcessor.hpp` | G-code parser and analyzer (used by viewer) |
| `GCode/ThumbnailData.hpp` | Encoded thumbnail images embedded in G-code |
| `GCode/Travels.hpp` | Travel path optimization |

To **add a G-code post-processor**: implement it in `src/libslic3r/GCode/` and hook it into the pipeline in `GCode.cpp`.

#### Support Generation: `src/libslic3r/Support/`

| File | Role |
|------|------|
| `SupportSpotsGenerator.hpp/.cpp` (79 KB) | Detect which regions need support |
| `SupportMaterial.hpp/.cpp` | Generate FFF support structures |
| `TreeSupport.hpp/.cpp` | Tree-style organic supports |

#### File Format I/O: `src/libslic3r/Format/`

| Format | File | Notes |
|--------|------|-------|
| STL | `STL.hpp/.cpp` | Import/export via admesh |
| 3MF | `3mf.cpp` (219 KB), `bbs_3mf.cpp` (435 KB) | Primary interchange format |
| AMF | `AMF.cpp` (63 KB) | Additive Manufacturing Format |
| OBJ | `OBJ.cpp`, `objparser.cpp` | Wavefront OBJ |
| STEP | `STEP.hpp/.cpp` | Via OpenCASCADE (disabled with `no-occt` preset) |
| SLA | `SL1.cpp`, `CWS.cpp`, `AnycubicSLA.cpp` | SLA/DLP output formats |
| SVG | `SVG.cpp` | Vector output |

To **add a new import format**: implement a loader in `Format/`, register it in `Format/Format.hpp` and in the CLI argument parser in `PrusaSlicer.cpp`.

#### Geometry Algorithms

| Location | Content |
|----------|---------|
| `Geometry/` | ArcWelder (arc fitting), convex hull, Voronoi |
| `Algorithm/` | Region expansion, polygon morphology |
| `ClipperUtils.hpp/.cpp` (136 KB) | All polygon offsets, unions, differences |
| `EdgeGrid.hpp/.cpp` | Spatial grid for fast edge proximity queries |
| `AABBMesh.hpp/.cpp` | AABB tree for ray-casting against meshes |
| `TriangleSelector.hpp/.cpp` | Per-triangle selection (painting tools) |

### 4b. GUI: `src/slic3r/GUI/`

wxWidgets-based, 200+ source files. Key files:

| File | Role |
|------|------|
| `MainFrame.cpp` | Main application window |
| `3DScene.hpp/.cpp` | OpenGL 3D scene graph |
| `3DBed.hpp/.cpp` | Build plate visualization |
| `GCodeViewer.hpp/.cpp` | G-code preview renderer |
| `Gizmos/` | Interactive 3D tools: cut, emboss, supports, seam, etc. |
| `ConfigWizard.hpp/.cpp` | First-run printer setup wizard |
| `Plater.hpp/.cpp` | Central plater view (object list + 3D view) |
| `Tab.hpp/.cpp` | Settings tab base class |
| `PresetBundle.hpp/.cpp` | Manages active preset selections |

The GUI calls into `libslic3r` for all computation — it has no slicing logic itself.

### 4c. Entry Points

| File | Role |
|------|------|
| `src/PrusaSlicer.cpp` | CLI entry point — `CLI::run()` parses args, loads models, runs pipeline |
| `src/PrusaSlicer_app_msvc.cpp` | Windows WinMain — bootstraps GUI on Windows |
| `src/slic3r/GUI/GUI_App.hpp/.cpp` | wxWidgets `wxApp` subclass — GUI main loop |

---

## 5. Slicing Pipeline

The pipeline is orchestrated by `Print::process()` via a `PrintStep` enum. Steps execute in order; each step invalidates downstream steps when its inputs change.

| Step | What Happens |
|------|-------------|
| `posSlice` | TriangleMeshSlicer clips the 3D mesh at each Z height → raw 2D polygons per layer |
| `posPerimeters` | Arachne wall generation → `ExtrusionLoop` paths stored in `LayerRegion::perimeters` |
| `posPrepareInfill` | Classify surfaces (top/bottom/internal), compute fill areas, handle bridges |
| `posInfill` | Run the selected fill pattern → `ExtrusionPath` entities in `LayerRegion::fills` |
| `posIroning` | Optional top-surface ironing pass |
| `posSupportSpotsSearch` | Detect overhangs needing support |
| `posSupportMaterial` | Generate support extrusion paths |
| `posEstimateCurledExtrusions` | Heuristic curling detection for quality warnings |
| `posCalculateOverhangingPerimeters` | Tag perimeters over voids (affects speed/cooling) |
| `posSimplifyPath` | Arc fitting (ArcWelder) and path simplification |
| `psWipeTower` | Compute wipe tower layout for multi-material |
| `psToolOrdering` | Sequence tool changes across all objects/layers |
| `psSkirtBrim` | Generate skirt and brim extrusions |
| `psGCodeExport` | Walk all layers/regions/objects → emit G-code via `GCode.cpp` |

The SLA pipeline is separate: `SLAPrint::process()` with its own `SLAPrintStep` enum.

---

## 6. Coordinate System

**All integer coordinates are scaled integers** to avoid floating-point drift:

| Type | C++ type | Unit |
|------|----------|------|
| `coord_t` | `int64_t` | 1 µm (micrometer) |
| `coordf_t` | `double` | millimeters (for output/display) |
| `SCALING_FACTOR` | `1e-6` | converts coord_t → mm |

Conversions:
```cpp
coord_t scaled = scale_(mm_value);        // mm → µm (int64_t)
double  mm     = unscale<double>(coord);  // µm → mm
```

Maximum model size: ~100 m in any axis. All Clipper polygon operations happen in `coord_t` space.

---

## 7. Key Data Flow

```
File (STL/3MF/AMF/OBJ)
  → Model (ModelObject + ModelVolume)
    → PrintObject (slicing state + config)
      → Layer[] (one per Z height)
        → LayerRegion[] (one per PrintRegion config)
          ├─ slices            (SurfaceCollection — raw polygons from slicer)
          ├─ fill_surfaces     (SurfaceCollection — classified for infill)
          └─ perimeters        (ExtrusionEntityCollection)
          └─ fills             (ExtrusionEntityCollection)
          └─ ironing_surfaces  (ExtrusionEntityCollection)
      → SupportLayer[]
        → support_fills (ExtrusionEntityCollection)
  → GCode (text output, one file)
```

---

## 8. Configuration Option Declaration Pattern

Options are declared using macros in `src/libslic3r/PrintConfig.hpp`. Example pattern:

```cpp
// In the OPT_DEF block for the relevant config struct:
OPT_DEF(ConfigOptionFloat, layer_height)
    .label(L("Layer height"))
    .tooltip(L("..."))
    .min(0.01)
    .default_value(new ConfigOptionFloat(0.2));
```

The macro system generates serialization, UI bindings, and CLI help text automatically. After declaring an option you can read it anywhere via:

```cpp
print_config.opt_float("layer_height")
// or the typed accessor:
print_config.layer_height.value
```

---

## 9. Testing

**Framework:** Catch2 (enable with `-DSLIC3R_BUILD_TESTS=ON`)

```bash
cmake --preset default -DSLIC3R_BUILD_TESTS=ON
cmake --build build-default -j8
cd build-default && ctest
```

Default ctest run excludes `[Slow]` and `[NotWorking]` tags.

**Test directories** (`tests/`):

| Directory | Coverage |
|-----------|---------|
| `libslic3r/` | Geometry, mesh, config, infill, format parsers (42 files) |
| `fff_print/` | Full FFF pipeline integration tests (27 files) |
| `sla_print/` | SLA pipeline |
| `arrange/` | NFP-based arrangement |
| `thumbnails/` | Thumbnail generation |
| `slic3rutils/` | GUI utilities |

Test data lives in `tests/data/` — one subfolder per test file, named to match the `.cpp` file. License: CC-By-SA 3.0 unless a `.license` sidecar says otherwise.

---

## 10. Resources & Profiles

### Profiles (INI format with inheritance)

```
resources/profiles/<VendorName>.ini
  [vendor]          name, version, config_update_url
  [printer_model]   id, name, variants
  [printer]         inherits = <model>, overrides
  [filament]        inherits = <base_filament>, overrides
  [print]           inherits = <base_print>, overrides
```

The version field follows `MAJOR.MINOR.COUNTER.PATCH`. PrusaSlicer checks `config_update_url` at startup to auto-update bundles.

Full authoring guide: `doc/How to create a vendor profiles.md` (600+ lines).

### Localization

`.po` files live in `resources/localization/<lang>/`. Workflow:

```bash
./BuildLinux.sh -l   # regenerate .pot template
# then msgfmt .po → .mo (done by CMake gettext targets at build time)
```

---

## 11. CI/CD

11 GitHub Actions workflows in `.github/workflows/`:

| Workflow | Platform | Architecture |
|----------|----------|--------------|
| `ccpp_win.yml` | Windows Server 2022 | x64 |
| `ccpp_mac.yml` | macOS 15 | Intel x64 |
| `ccpp_mac_arm.yml` | macOS 15 | ARM64 (Apple Silicon) |
| `ccpp_ubuntu_gtk3.yml` | Ubuntu 22.04 | x64, GTK3 |
| `ccpp_ubuntu_gtk2.yml` | Ubuntu | x64, GTK2 |
| `*_rc` variants | Same as above | Release-candidate builds |

**Pipeline stages:** dependency build (cached by version string) → slicer build → package → upload artifacts.

**Artifacts:**
- Windows: ZIP (EXE+DLLs) + MSI installer (WiX)
- Linux: `.AppImage` + `.tgz`
- macOS: `.dmg`

Release packaging: `create_release.py` downloads artifacts from GitHub Actions and renames them with version + date.

---

## 12. External Dependencies

| Library | Purpose |
|---------|---------|
| Boost 1.66+ | Filesystem, threads, log, locale, regex, iostreams |
| wxWidgets 3.2 | GUI framework |
| TBB | Threading Building Blocks (parallel algorithms) |
| Eigen3 3.3+ | Linear algebra |
| CGAL | Computational geometry (mesh boolean ops) |
| OpenVDB 5.0+ | Volumetric data structures |
| Clipper | 2D polygon clipping (bundled in `src/clipper/`) |
| libigl | Geometry processing (bundled in `src/libigl/`) |
| libnest2d | 2D bin-packing / arrangement (bundled in `src/libnest2d/`) |
| NLopt 1.4+ | Non-linear optimization |
| GMP + MPFR | Arbitrary precision arithmetic (for CGAL) |
| CURL | HTTP (profile updates, telemetry) |
| OCCT | OpenCASCADE — STEP file support (optional, `no-occt` preset disables) |
| OpenSSL | TLS for CURL |
| LibBGCode | Bambu Lab G-code format |
| Catch2 | Unit testing (test builds only) |
| admesh | STL mesh repair (bundled in `src/admesh/`) |
| AngelScript | Scripting engine (bundled in `src/angelscript/`) |
| ImGui | In-process overlay UI (bundled in `src/imgui/`) |

---

## 13. Agent Working Tips

### Adding a new infill pattern
1. Create `src/libslic3r/Fill/FillMyPattern.hpp/.cpp`, subclass `FillBase`, implement `fill_surface()`.
2. Add an enum value to `InfillPattern` in `src/libslic3r/PrintConfig.hpp`.
3. Register the factory entry in `src/libslic3r/Fill/Fill.cpp` (`Fill::new_from_type()`).
4. Add a translated label in the enum option definition in `PrintConfig.hpp`.

### Adding a new file import format
1. Implement loader in `src/libslic3r/Format/MyFormat.hpp/.cpp`.
2. Register the file extension and loader in `src/libslic3r/Format/Format.hpp`.
3. Add the extension to the CLI file filter in `src/PrusaSlicer.cpp` (`CLI::run()`).
4. Add a Catch2 test in `tests/libslic3r/test_myformat.cpp`.

### Adding a new print config option
1. Declare it in the appropriate config struct in `src/libslic3r/PrintConfig.hpp` using the macro DSL.
2. Consume it wherever needed via `config.opt_float("my_option")` or the typed accessor.
3. Add a UI control in the relevant settings `Tab` file under `src/slic3r/GUI/`.
4. Update any profile INI files in `resources/profiles/` that need a default value.

### Adding a new PrintStep
1. Add the enum value to `PrintStep` (or `PrintObjectStep`) in `src/libslic3r/Print.hpp`.
2. Implement the step method on `Print` or `PrintObject`.
3. Wire up invalidation: call `this->invalidate_step(posMyStep)` from any config-change handler that affects your step.
4. Insert the step into `Print::process()` in `src/libslic3r/Print.cpp` in the correct order.

### Adding a G-code post-processor
1. Create `src/libslic3r/GCode/MyPostProcessor.hpp/.cpp`.
2. Instantiate and call it from the appropriate place in `GCode::_do_export()` in `src/libslic3r/GCode.cpp`.
3. If it needs config options, declare them in `PrintConfig.hpp` (see above).

### Navigating GCode.cpp
`GCode.cpp` is 638 KB. Use these anchors:
- `GCode::_do_export()` — top-level export loop (iterates layers/objects)
- `GCode::extrude_loop()` — emits a single extrusion loop
- `GCode::travel_to()` — travel move with retraction logic
- `GCode::set_extruder()` — tool-change sequence

### Understanding a layer's geometry
Start in `LayerRegion`: `slices` are raw polygons from the mesh slicer. `fill_surfaces` are those polygons after surface classification (which ones are top, bottom, internal solid, sparse). `perimeters` and `fills` are the final extrusion entities ready for G-code.
