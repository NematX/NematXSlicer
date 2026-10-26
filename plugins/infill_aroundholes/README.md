# NematX rectilinear around holes

Package `nematx-infill-aroundholes`, `INFILL_PATTERN` plugin id `rectilineararoundholes`
(preserves the pattern value used by NematXSlicer 2.7 profiles).
Available in the sparse, top, bottom and solid pattern lists.

Rectilinear lines that, instead of jumping over a hole, follow its contour half a
spacing inside the material with a low-flow path, through the shorter side.

## Files

| Path | Role |
| --- | --- |
| `src/AroundHolesKernel.hpp` | Host-independent algorithm (plain C++). |
| `src/AroundHolesPattern.cpp` | SDK adapter: options, pattern lists, offsets, publication. |
| `tests/kernel_tests.cpp` | Kernel tests, no host library needed. |
| `tests/smoke_export.py` | Real export on a holed plate (1% and 0% bypass flow). |
| `package/` | Package metadata templates. |

## Options (region)

- `fill_rectilinearholes_travel_speed` (mm/s or % of `travel_speed`, default 100%, 0 = travel speed).
- `fill_rectilinearholes_travel_flow_ratio` (% of the infill flow, default 1%; 0% moves without extruding).

## Behaviour and differences with 2.7

- The rectilinear rasterisation is a private copy of the slicer code: host
  changes to its own rectilinear patterns do not affect this plugin.
  Incompatible changes to the public SDK still require adaptation; the ABI
  manifest checks compatibility. Host rectilinear fixes are not imported automatically.
- A bypass is a thin infill path (role of the surface, width × 0.1, its own speed),
  not a travel: v3 forbids travel geometry inside printed content. With 0% flow,
  a zero `EXTRUSION` command moves without E. Preview shows it as infill.
- Fixes: the bypass contour is offset vertex by vertex (Clipper could renumber the
  vertices); the vertex walk has no off-by-one; a bypass never targets a piece that
  was dropped for being too short (the chain is split instead).
- Full-fill volume correction is done by the host post-infill step, bypasses included.

## Verification

```powershell
cmake --build .build --config Debug --target nematx_infill_aroundholes infill_aroundholes_tests
ctest --test-dir .build -C Debug -R infill_aroundholes --output-on-failure
python plugins/infill_aroundholes/tests/smoke_export.py --host C:/path/to/Slic3r_console.exe
```

Current validation: the DLL compiles and links against the Debug host library;
kernel tests pass. Real exports (including 0% E and speed inheritance) and the
comparison with 2.7 remain unvalidated until a rebuilt host executable is available.
