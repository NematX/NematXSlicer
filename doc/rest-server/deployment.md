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

## Flat profiles in this release

REST currently reads `profiles/print/*.ini`, `profiles/printer/*.ini` and
`profiles/filament/*.ini`. It does **not** load a vendor bundle in `vendor/` yet.
As a GUI workaround, select the desired vendor presets, save each category as a
named user preset, and copy its full INI from the GUI data directory's `print`,
`printer` or `filament` subdirectory. Remove the `inherits` setting from the
copies after confirming they contain resolved settings; leave GUI originals
untouched. Update `default_print_profile`/`default_filament_profile` references
to the copied names, or select all three categories explicitly through REST.
Do not copy a vendor file containing `[print:...]` sections directly: inheritance
and abstract parents need resolution first. Restart after replacing profiles.

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
2. Supply the converted Basic profiles and an account named `Smoke` (or pass
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
Progress percentages are indicative, jobs have no crash recovery, and large
viewer transfers still obey non-file response timeouts. Linux/macOS service
examples, proxy behavior and console no-echo interaction need platform validation.

## Developer verification

Validation in this development environment uses Debug; it is not a Release,
Linux/macOS or real reverse-proxy validation. Test those configurations before
production use. Development-only build wrappers are not distributed tools.

The helper in `rest_deployment_tests` flattens the checked-in small Basic bundle
using the GUI's `LoadSystem` loader. It is a test fixture, not an installed
converter or a complete vendor catalog. To export it, set
`REST_TEST_FLAT_PROFILES` to a disposable output directory and run
`rest_deployment_tests "*Basic profiles*"`.

Build test targets explicitly. `ctest -R "^rest_" -LE rest_app` excludes commands
requiring application launchers, but includes standalone-command tests. The
`rest_launch_tests` executable is separate from uninitialized-engine transport
tests. Do not interpret an empty CTest selection as success.
The password command checks use CMake's JSON reader (CMake 3.19 or newer).
