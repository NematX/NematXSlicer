# V1 platform validation checklist

These checks have **not** been executed on Linux/macOS or through real proxies
in the Windows validation. Record OS, compiler, dependency versions, commit,
commands and results before checking an item. Use disposable accounts/work
storage and loopback listeners. Do not put credentials in reports.

## Linux

- [ ] GUI-enabled Release build and all REST/preset tests:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSLIC3R_BUILD_REST_SERVER=ON -DSLIC3R_GUI=ON
cmake --build build --target Slic3r
cmake --build build --target slic3r_rest_server
cmake --build build --target slic3r_rest_password
cmake --build build --target rest_server_tests rest_launch_tests rest_deployment_tests rest_profiles_tests rest_models_tests rest_settings_tests rest_engine_tests rest_session_tests rest_auth_tests rest_slice_tests rest_e2e_tests libslic3r_preset_tests
ctest --test-dir build -R '^rest_|^libslic3r_preset' --output-on-failure
```

- [ ] Repeat after configuring `-DSLIC3R_GUI=OFF`; verify no GUI dependency is
  required and the `<app>-rest` link launches REST directly.
- [ ] Configure the intended FHS prefix, build all installable enabled modules
  (including `OCCTWrapper` if enabled), and install:

```sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local -DSLIC3R_GUI=OFF
cmake --build build --target OCCTWrapper
cmake --install build
```

- [ ] Adapt `examples/superslicer-rest.service` paths/user to that installation;
  verify unprivileged service operation, SIGTERM cleanup and restart:

```sh
sudo systemctl daemon-reload
sudo systemctl start superslicer-rest
sudo systemctl stop superslicer-rest
sudo journalctl -u superslicer-rest --no-pager
```

## macOS

- [ ] Run the build/test commands above with the supported dependency setup and
  GUI ON, then OFF. Run the official packaging script and verify the application
  and `<app>-rest` link under `Contents/MacOS`, resources under `Contents/Resources`.
- [ ] From outside the checkout, run the packaged `<app>-rest --config PATH`;
  run the public smoke workflow below, send SIGTERM, verify exit 0 and no owned
  upload/export/result artifacts.
- [ ] Adapt `examples/com.superslicer.rest.plist` executable/configuration paths
  and label; test the user service without administrator privileges:

```sh
launchctl bootstrap gui/$(id -u) "$HOME/Library/LaunchAgents/com.superslicer.rest.plist"
launchctl kickstart gui/$(id -u)/com.superslicer.rest
launchctl bootout gui/$(id -u) "$HOME/Library/LaunchAgents/com.superslicer.rest.plist"
```

## Public smoke workflow

Supply `profiles/vendor/Basic.ini`, a disposable `Smoke` account and server.json
with absolute resource/profile/work/accounts paths. Start on loopback port 0 and
read the assigned port. PowerShell 7 may run the existing script on any platform
(there is no shipped Bash smoke script):

```sh
read -r -s REST_SMOKE_PASSWORD
export REST_SMOKE_PASSWORD
pwsh -NoProfile -File doc/rest-server/examples/smoke-test.ps1 -BaseUrl http://127.0.0.1:PORT -OutputDirectory ./smoke-output
unset REST_SMOKE_PASSWORD
```

- [ ] Login, profile selection, STL upload, slice, download, delete and logout
  pass; graceful stop prints `stopping`, then `stopped`, exits 0 and leaves no
  session artifacts. Repeat with the standalone server and separate work storage.

## nginx and Caddy

- [ ] Adapt examples to disposable certificates and the server's loopback port;
  validate and reload the actual proxy:

```sh
nginx -t -c /absolute/path/nginx.conf
caddy validate --config /absolute/path/Caddyfile
caddy run --config /absolute/path/Caddyfile
```

- [ ] Run the smoke workflow against the HTTPS origin (trusted certificate;
  do not disable TLS validation). Check unauthenticated requests return 401.
- [ ] Select a real listed profile ID containing `/`, URI-encode it once to
  `%2F`, and test the applicable setting/profile route through each proxy.
  Compare with direct loopback behavior; preserve bearer Authorization.
- [ ] Upload a valid model just below the configured byte limit, then one above
  it; verify success/refusal, no partial session mutation and no leaked uploads.
- [ ] Download large G-code slowly (`curl --limit-rate 32k` with an authorization
  header supplied securely); continued progress may exceed the idle deadline.
  Stop reading to prove eventual disconnect/cleanup. No proxy buffering or URI
  decode/re-encode should change the public contract.

## GUI and terminals

- [ ] Start the actual GUI with a system bundle containing a preserved unknown
  option: no substitution dialog and no modified system preset. Direct loader
  tests are not a visual startup check.
- [ ] On Windows and Linux real terminals, run
  `slic3r_rest_password --username Smoke`: password keystrokes are not echoed,
  cancellation emits no accounts document, and the generated hash authenticates.
  Windows unsupported characters must be rejected, not truncated. Verify non-ASCII
  passwords through redirected UTF-8 `--password-stdin` without a BOM.

Restore original build options, prefixes and local service configuration after
validation; retain only non-secret evidence.
