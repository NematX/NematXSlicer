# SuperSlicer REST deployment

## Build and install

Use C++17 and a Release build for deployment. Configure the existing dependency
setup, including Argon2, with `SLIC3R_BUILD_REST_SERVER=ON`. GUI and headless
builds are supported; `SLIC3R_GUI=OFF` selects headless operation.

On Windows, with a Visual Studio multi-configuration generator:

```powershell
cmake -S . -B build -DSLIC3R_BUILD_REST_SERVER=ON -DSLIC3R_GUI=OFF
cmake --build build --config Release --target Slic3r_app_rest
cmake --build build --config Release --target Slic3r_app_console
cmake --build build --config Release --target slic3r_rest_server
cmake --build build --config Release --target slic3r_rest_password
# If STEP support is enabled, also build its installable module:
# cmake --build build --config Release --target OCCTWrapper
cmake --install build --config Release --prefix "C:/Services/SuperSlicer"
```

Build every enabled installable module before installation (`OCCTWrapper` when
enabled). With `SLIC3R_GUI=ON`, also build `Slic3r_app_gui` in Release. CMake's
install command does not build missing targets.
The application launchers load `Slic3r.dll`; keep them beside that DLL and the
GMP/MPFR runtime DLLs. Do not copy an executable alone.

On Linux/macOS with a single-configuration generator, configure with
`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSLIC3R_BUILD_REST_SERVER=ON
-DSLIC3R_GUI=OFF`, then build `Slic3r`, `slic3r_rest_server` and
`slic3r_rest_password` separately with `cmake --build build --target TARGET`.
For an FHS/package install, set `CMAKE_INSTALL_PREFIX` during configuration to
match the platform's resource lookup layout. Preserve the macOS bundle layout.

Windows executable, DLL and resource destinations are relative to the install
prefix: `cmake --install --prefix` relocates them together, including paths with
spaces/non-ASCII characters. Installed resources are read-only; accounts,
profiles and work storage are separate operator-managed files.

Windows: `<app>_rest.exe --config PATH` or
`<app>_console.exe --rest-server --config PATH`. Linux/macOS: `<app>-rest` or
`<app> --rest-server`. On macOS the application binary/link belongs in the
bundle's `Contents/MacOS`, resources in `Contents/Resources`; preserve this layout.
The standalone `slic3r_rest_server` works without the application DLL/GUI but still
requires its core runtimes and resources. Its defaults use the working directory.
The application's resource defaults use its own layout; all explicit JSON/CLI
paths take priority. Modes must be the first argument. No GUI preferences/data
are imported by REST.

## Configuration and accounts

Copy [examples/server.json](examples/server.json). All paths in JSON are UTF-8
and relative to the **configuration file**, not the service working directory.
Use absolute resources for an FHS install or macOS bundle. Profiles/resources must
exist; work storage must be writable. Unknown keys, including nested misspellings,
are rejected. CLI overrides take priority. Never store passwords/tokens in JSON
configuration, command-line arguments, source control or service logs.

Provision an accounts document with `slic3r_rest_password --username NAME`, or
`<app>_console.exe --rest-hash-password --username NAME`. The interactive prompt
does not echo. For automation use `--password-stdin`: exactly one line from a
redirected UTF-8 stream without a BOM, with no prompt or plaintext output; console stdin is refused.
The tool reads bytes without transcoding: configure the producer's encoding,
especially when using Windows PowerShell. Windows interactive input supports
printable ASCII only and refuses unsupported characters instead of silently
dropping them; use redirected UTF-8 input for non-ASCII passwords.
Options can appear in either order, once each. Redirect stdout to `accounts.json`;
on Windows prefer a process API writing UTF-8 without a BOM instead of Windows
PowerShell 5's UTF-16 `>` redirection. Protect the file with ACLs/0600. The output
is a complete `users/username/password_hash` document, not a merge operation.
To add users, securely merge hash entries without replacing existing users.
Account edits require restarting the server. Restrict the service account's
permissions and protect accounts, tokens and backups.

## Vendor and user profiles

Copy `vendor`, `print`, `printer` and `filament` from the data directory of a
configured GUI to the REST profile root. Alternatively, copy the desired `.ini`
files from [slic3r-profiles](https://github.com/slic3r/slic3r-profiles) into
`profiles/vendor/`. Each subdirectory is optional. Vendor bundles are loaded with
the GUI system loader: inheritance is resolved, abstract parents are removed,
and printer model/variant metadata is checked. Flat user INIs are complete
configurations layered over system presets; keep their `inherits` parent names.

All system profiles are listed, without GUI installation filtering. Names are
the full preset names (not aliases); compatibility can be queried before
selection (§5.4 of the contract). Vendor files are loaded in filename order:
duplicate preset names keep the first definition and produce warnings on stderr
after `listening on`. A user preset with a system preset's name is ignored by the
core loader with a log warning. Vendor bundles that the GUI skips because they
have no models/variants and no template metadata are also skipped, with a startup
warning (including `Templates.ini` from the pinned 2.7 package profiles).
Unreadable/malformed INIs and invalid vendor sections still fail startup.
Restart after modifying profiles; never edit the catalog during a live process.

Profile paths and filenames are UTF-8, including on Windows. Preserved unknown
bundle keys remain in custom-setting sidecars; they are not applied as engine
options or presented as substituted values by the GUI system loader.

### Indicative performance

Windows Release measurements on 2026-10-04: Windows 10 Pro, AMD Ryzen 7 7840U
(8 cores/16 threads), 31.3 GiB usable RAM, MSVC 19.44.35228.0, C++17.
Selection/defaults are means over 20 calls of the catalog's default printer;
"every printer" selects each printer with its default print once (mean/max);
load and list are single observations.

Defaults, selections and project loads work on a reduced copy of the catalog:
the presets they name, their parents, and the filaments compatible with one of
the printer's extruders.

| Catalog | FFF profiles | Load | Selection | Defaults | Every printer (mean / max) | Compatibility list |
|---|---:|---:|---:|---:|---:|---:|
| Pinned package `c3fc5fd5948c74c51dd6d49d1521b6059eb82f7d` (37 INIs) | 6,912 | 4.301 s | 7.1 ms | 6.9 ms | 32.6 / 271 ms (642 printers) | 16.8 ms |
| Upstream main `ca25c7ec55dcc6073da61e39692c321cdb6497dc` (23 INIs) | 1,845 | 0.792 s | 4.9 ms | 5.0 ms | 11.4 / 60 ms (338 printers) | 6.2 ms |

The slowest printers have several tools with different settings (for example
5-tool printers with per-tool offsets): filament compatibility is evaluated over
the whole catalog once per distinct tool, about 40 ms each on the pinned package.

Earlier measurements, which copied the whole catalog for every request:

| Catalog | Load | Selection | Defaults | Compatibility list |
|---|---:|---:|---:|---:|
| Pinned package | 23.769 s | 2.533 s | 2.106 s | 38.705 ms |
| Upstream main | 6.683 s | 0.831 s | 0.757 s | 15.445 ms |

On the day of the change, before it, a full catalog copy alone measured 0.73 to
1.72 s (pinned package) and 0.20 to 0.81 s (main) across runs, with selection at
1.34 s and defaults at 1.29 s on the pinned package. Load times varied between
sessions (23.8 s to 4.3 s on the pinned package); they are not controlled
comparisons.

Package inputs were copied from the pinned `resources/profiles` submodule and
processed by the official `download_vendor_bundles.py` (its extra repository list
was empty). The package catalog emitted one expected warning for skipped
`Templates.ini`; main emitted no catalog warnings. No bundle was edited or
flattened. A prior Debug run of main, with whole-catalog copies, measured
7.038 s load, 2.114 s selection, 1.669 s defaults and 99.338 ms list.

Defaults and selections use the import worker pool, preserving network
responsiveness but consuming worker slots and memory. These figures are not
latency/throughput guarantees; measure your own deployment. The earlier benchmark
initialization also logged an unavailable cached Python plugin package and a
disabled `max_overhang_threshold` plugin type mismatch; these were not catalog
warnings and no plugin repair is claimed.
## Operation

Startup prints `listening on ADDRESS:PORT` (IPv6: `[ADDRESS]:PORT`) after binding; port
zero prints the assigned port. Shutdown prints `stopping`, then `stopped` after
workers have joined. Exit 0 means help/normal shutdown, 1 means startup/options
failed. Use Ctrl+C, SIGTERM, or Windows Ctrl+Break. Avoid forced termination:
it can leave work artifacts. Stop cancels active jobs and retires sessions;
tokens, sessions and results are not persisted across restarts. An `engine_data`
directory and empty `uploads`, `exports`, `results` containers may remain in
work storage; temporary uploads, export files and result directories must not.
Run the [smoke script](examples/smoke-test.ps1) against a loopback server before
exposing it remotely. Service examples are templates: adjust executable command
names (`SLIC3R_APP_CMD`), paths, users and file permissions before enabling them.
If the smoke script fails, stop the disposable server to retire any partially
created session before checking its work directory.

### Installed smoke verification

The script tests HTTP routes against an **already running** server. It logs in,
selects the named Basic profiles, uploads a closed STL cube, slices, downloads
G-code, deletes the session and logs out. It does not launch/stop the process,
inspect its work directory, or cover viewer G-code, project export and encoded
setting IDs (`%2F`). Those routes require separate functional/proxy checks.

For a disposable installation check:

1. Install to a prefix containing a space and a non-ASCII character. From another
   directory, launch the installed REST launcher with `--config PATH --port 0`.
   Read the assigned port from `listening on`; do not expose the test server.
2. Place `Basic.ini` in `profiles/vendor/` and supply an account named `Smoke` (or pass
   `-Username NAME`). Set `REST_SMOKE_PASSWORD` in the caller's environment and run
   `smoke-test.ps1 -BaseUrl http://127.0.0.1:PORT -OutputDirectory PATH`.
3. Stop with Ctrl+Break on Windows, verify exit 0 and ordered `stopping`/`stopped`
   lines, and check that work storage has no upload/export files or result
   subdirectories. `engine_data` and empty containers may remain.
4. Repeat with the installed standalone server. Do not run two servers against
   the same work directory. Remove the password environment variable afterwards.

## Remote access and proxies

Keep REST on loopback behind HTTPS or inside a trusted encrypted VPN. Bearer
tokens/passwords over cleartext networks are unsafe. There is no native TLS,
browser CORS contract or remote-administration endpoint. Untrusted models can
consume CPU/memory; cap uploads, connections and worker counts appropriately.
REST never runs configured post-processing commands, but should still run under
an unprivileged isolated account.

[nginx](examples/nginx.conf) and [Caddy](examples/Caddyfile) examples preserve the
original URI (especially `%2F` inside opaque profile IDs), Authorization, streaming
uploads and downloads. Do not add path stripping, decode/re-encode passes or a
trailing URI to proxy_pass. Align proxy/server body-size limits. Nginx buffering
is disabled to avoid disk staging and whole-file delays. Align proxy send/read
idle limits; Caddy's response-header limit concerns time before headers, not
download duration. Test encoded IDs and a large transfer through your actual
proxy: these templates were reviewed, not executed in this Windows validation.

## Timeouts, troubleshooting and limitations

`request_seconds` bounds header/buffered request processing and non-file response
writes. Multipart upload and file downloads use an **idle** deadline, reset only
when bytes advance; a slow active transfer may exceed the configured duration.
A peer whose writes stop advancing is disconnected and its file owner released.
Socket buffering means this measures server-side write progress, not application
acknowledgements from the client. Session expiration pauses during work.

For load errors check resource/profile directories, write permissions, accounts
JSON, UTF-8 encoding and unknown-key diagnostics. Missing GMP/MPFR DLLs prevent
Windows startup before application diagnostics. A 401 means credentials/token
failed; a 404 may deliberately hide another account's session. See the spec for
409 busy/stale-result cases. Logs/progress should never contain credentials.
Large viewer transfers still obey non-file response timeouts.
See the contract's [Known limitations](rest-server.md#known-limitations) and the
[platform checklist](platform-validation.md) for the authoritative remaining scope.

## Developer verification

Windows Debug and Release have been validated. Release core, GUI-enabled
application, launchers, standalone tools, enabled STEP module and REST suites
built separately; all 19 REST/preset CTest selections passed (144.01 s), and
profile suite randomized seeds 2/5 passed. Both installed server forms passed
the public Basic workflow, PATH launch outside the checkout, actual Ctrl+Break
exit 0 and clean work storage under a prefix with spaces/non-ASCII characters.
This is not Linux/macOS, real reverse-proxy, visual GUI or physical terminal
validation. Development-only build wrappers are not distributed tools.

`rest_deployment_tests` loads the checked-in small Basic bundle directly from
`profiles/vendor/`, selects its profiles over real loopback HTTP and slices a
cube. The fixture is not a complete vendor distribution.

Build test targets explicitly. `ctest -R "^rest_" -LE rest_app` excludes commands
requiring application launchers, but includes standalone-command tests. The
`rest_launch_tests` executable is separate from uninitialized-engine transport
tests. Do not interpret an empty CTest selection as success.
The password command checks use CMake's JSON reader (CMake 3.19 or newer).
