# Specification — SuperSlicer REST Server

**Status:** V1 contract\
**Version:** 1.1

Version 1.1 adds scene editing (§5.16), entity settings/layer ranges (§5.17),
arrangement/orientation (§5.18), plane cuts (§5.19), and facet painting (§5.20);
1.0 routes and response
meanings are unchanged. New response fields are additive.

## 1. Objective

Provide a standalone REST service allowing a remote client to manage
SuperSlicer sessions, select profiles, prepare a platter, start slicing,
and retrieve its results without a graphical interface.

The repository contains a C++ engine (`libslic3r`) and a command-line interface.
This specification defines the REST server to be built; it calls `libslic3r`
directly to retain and control session state within its own process.

## 2. Initial decisions

Confirmed for the initial scope:

- the server runs as a standalone process, without a GUI;
- it is intended to be remotely reachable through a VPN or reverse proxy;
- the API uses user accounts and a Bearer token obtained through `/login`;
- passwords use Argon2id with a per-account salt, and `/login` is limited
  to five requests per minute per username;
- the server calls `libslic3r` directly to manage sessions and slicing;
- HTTP transport uses Boost.Beast/Asio, already available through Boost in the repository;
- JSON data uses the `nlohmann` library already present in the repository;
- the CMake option `SLIC3R_BUILD_REST_SERVER`, disabled by default, adds the
  internal library `slic3r_rest` and a REST launch mode of the application
  executable (§9), without requiring the GUI;
- different sessions may slice simultaneously, with at most one active
  slicing operation per session;
- accounts are stored in a file administered outside the API; a local tool
  for generating Argon2id hashes is shipped with the server (see §9 and deployment.md);
- the first version covers FFF/FDM slicing and G-code generation from a model;
- it exposes profiles/configurations and manages sessions;
- installed profiles may be listed and selected, but not created, modified,
  imported, or deleted through the API in V1;
- extruder profiles do not exist yet: their catalog and selection are empty
  in V1, without preventing slicing;
- no post-processing script is executed in REST mode;
- a session represents a logical workspace containing a profile selection,
  settings, and platter state.

## 3. V1 functional scope

### Included

1. Start and stop the service independently of the graphical application.
2. Retrieve the list of available settings, grouped by category.
3. Create a session and use its identifier in every session-specific route.
4. List available profiles and select the session's profiles.
5. Read one or more selected session settings; modify a single setting
   or submit several settings in one request.
6. Add a model to the session or load a 3MF project.
7. List objects, their parts, instances, and layer ranges, then transform an object,
   instance, or part, or change the instance count.
8. Read and modify settings on objects, parts, and layer ranges.
9. Clear the platter, start slicing, save a 3MF project, and retrieve
   JSON data for the G-code viewer.
10. Delete a session.
11. Authenticate users by account and isolate their sessions.

### Outside the initial scope

- sending files to a printer or controlling a physical print;
- exposing the graphical interface's state or actions;
- providing a web interface;
- creating or administering accounts through the API; accounts are provisioned outside the API by the administrator;
- creating, modifying, importing, or deleting profiles through the API;
- SLA slicing support; V1 is limited to FFF/FDM.

## 4. Operating model

- `GET /list_settings` describes the settings the API can expose. Its categories
  are `preference`, `print`, `filament`, `extruder`, `printer`, and `entity`;
  definitions with entity overrides also list their allowed `scopes`.
- `get_setting` and `get_settings` read the session's effective values;
  the former uses SuperSlicer setting serialization, while the latter returns
  typed JSON values for the requested settings, as does `set_settings`.
- Settings in the `preference` category are also session-local; reading or
  modifying them does not change the process's global preferences or those
  of other sessions.
- A session is an isolated workspace: selected profiles, settings,
  models/project, and scene state. It is addressed by an opaque `session_id`
  and belongs to the account that created it. It may carry an optional display
  name, the equivalent of the GUI project name (§5.3.1).
- Every session-specific route includes `session_id` in its path. This ID
  locates the session context and does not replace the authentication required
  for remote access. Global routes `/login`, `/logout`, `/list_settings`, and
  `/create_session` have no session; only `/login` does not require a token.
- `add_model` adds a model to the scene; `load_project` replaces the existing
  scene with the 3MF project's scene and applies its saved profiles/settings.
  Loading a profile clears local modifications to the settings it replaces.
  Settings absent from a loaded configuration revert to system defaults;
  unknown settings are preserved as custom data without being applied to the
  engine (see §5.5.1).
- Object, part, instance, and layer-range identifiers are unique and stable within a session
  while the entity exists. They are invalidated when the entity is removed
  or the scene is replaced; transformations do not change retained entities' IDs.
  A layer-range ID is attached to its owning object and exact height bounds; changing
  the bounds preserves that ID.
- `slice` starts asynchronous slicing of the session's current state.
  `GET /session/{session_id}/state` returns its `etat` (`ready`, `slicing`,
  `sliced`, or `failed`) and the main and secondary `PrintStatus` messages
  and percentages. Only one slicing operation may be active per session;
  different sessions may slice simultaneously. During slicing, mutation
  operations are refused, except for `cancel_slicing`.
- Sessions are kept in memory only and do not survive a server restart.
  An inactive session expires after 24 hours by default; this duration is
  configurable, and every authorized session request resets it. A session
  cannot expire during slicing, including G-code export and the effective
  stopping period after cancellation. The inactivity period restarts when
  work finishes. An expired session returns `404 session_not_found` as soon as
  its inactivity deadline is reached, even before the next cleanup sweep; a late
  request never extends its lifetime. Sweeping reclaims untouched expired sessions.
  Recovery after a server crash is outside the V1 scope.
- A change affecting slicing invalidates the current G-code and returns the
  session to `ready`. Data from the last successful viewer result is retained
  with `outdated: true`; it remains readable, including in the `ready` state.

## 5. Route and JSON contracts

Routes use the supplied paths without a version prefix:
`/list_settings`, `/session/{session_id}/...`, etc. JSON bodies and responses
are UTF-8 encoded. Field names use `snake_case`.

| Method | Route | Input and output |
| --- | --- | --- |
| `POST` | `/login` | JSON credentials; Bearer token in response |
| `POST` | `/logout` | Revoke the current Bearer token |
| `GET` | `/list_settings` | JSON setting definitions |
| `GET` | `/list_paint_kinds` | JSON support, MMU, and facet-annotation channels |
| `POST` | `/create_session` | Optional JSON name; return `session_id` and `name` |
| `POST` | `/session/{session_id}/set_name` | JSON name to set, or `null` to remove it |
| `GET` | `/session/{session_id}/get_name` | JSON session name |
| `GET` | `/session/{session_id}/list_profiles` | JSON available profiles |
| `POST` | `/session/{session_id}/set_profile` | JSON profiles to select |
| `GET` | `/session/{session_id}/get_setting/{setting_id}` | Serialized setting value |
| `POST` | `/session/{session_id}/get_settings` | Requested IDs in JSON; typed values in response |
| `POST` | `/session/{session_id}/set_setting/{setting_id}/{serialized_value}` | INI-style serialized value in the path |
| `POST` | `/session/{session_id}/set_settings` | JSON object of settings to modify |
| `POST` | `/session/{session_id}/add_model` | Upload an STL, OBJ, or 3MF |
| `POST` | `/session/{session_id}/load_project` | Upload a 3MF project |
| `GET` | `/session/{session_id}/list_objects` | JSON objects, parts, instances, layer ranges, and variable-layer-height presence |
| `POST` | `/session/{session_id}/arrange` | Optional JSON placement settings and object/instance IDs |
| `POST` | `/session/{session_id}/orient` | Optional JSON orientation settings and object/instance IDs |
| `POST` | `/session/{session_id}/cut/{id}` | Cut an object or solid part by a plane |
| `POST` | `/session/{session_id}/paint/{id}` | Paint facet states on an object or solid part |
| `POST` | `/session/{session_id}/clear_paint/{id}` | Clear one or all paint channels |
| `GET` | `/session/{session_id}/get_paint/{part_id}` | Read painted triangles for a solid part |
| `GET` | `/session/{session_id}/get_custom_gcodes` | Read the custom G-code list (color changes, pauses, templates, custom G-code per height) |
| `POST` | `/session/{session_id}/set_custom_gcodes` | Replace the whole custom G-code list |
| `GET` | `/session/{session_id}/get_layer_height_profile/{object_id}` | Read the stored and effective layer-height profile |
| `POST` | `/session/{session_id}/set_layer_height_profile/{object_id}` | Replace, adapt, smooth, or reset an object's layer-height profile |
| `POST` | `/session/{session_id}/transform/{id}` | JSON matrix applied to an object, instance, or volume/part ID |
| `POST` | `/session/{session_id}/set_instance_count/{object_id}` | JSON target instance count |
| `POST` | `/session/{session_id}/set_part_type/{part_id}` | Change a part type |
| `POST` | `/session/{session_id}/rename/{id}` | Rename an object or part |
| `POST` | `/session/{session_id}/delete_entity/{id}` | Delete an object, part, instance, or layer range |
| `POST` | `/session/{session_id}/add_part/{object_id}?type=...` | Upload a part into an object |
| `POST` | `/session/{session_id}/add_shape/{object_id}` | Add a built-in shape part |
| `POST` | `/session/{session_id}/split/{id}` | Split into objects or parts |
| `POST` | `/session/{session_id}/merge` | Merge objects into one multipart object |
| `GET` | `/session/{session_id}/list_entity_settings/{entity_id}` | Explicit setting IDs on an object, part, or layer range |
| `GET` | `/session/{session_id}/get_entity_settings/{entity_id}/{setting_id}` | Typed explicit entity setting |
| `POST` | `/session/{session_id}/set_entity_setting/{entity_id}/{setting_id}/{serialized_value}` | Set one serialized entity override |
| `POST` | `/session/{session_id}/set_entity_settings/{entity_id}` | Atomically set/remove typed entity overrides |
| `POST` | `/session/{session_id}/add_layer_range/{object_id}` | Add a height range with required height/extruder settings |
| `POST` | `/session/{session_id}/set_layer_range/{range_id}` | Change bounds while preserving the range ID |
| `POST` | `/session/{session_id}/slice` | Start asynchronous slicing and return its initial state |
| `GET` | `/session/{session_id}/state` | `etat` and main/secondary progress |
| `POST` | `/session/{session_id}/cancel_slicing` | Request cancellation of active slicing (`202 Accepted`) |
| `GET` | `/session/{session_id}/get_gcode` | Download raw G-code after successful slicing |
| `POST` | `/session/{session_id}/clear_platter` | Clear the platter; JSON response |
| `GET` | `/session/{session_id}/save_project` | Binary 3MF snapshot download; optional `file_name` query |
| `GET` | `/session/{session_id}/get_gcode_for_view` | Paginated viewer JSON (`offset`, `limit`, `result_id`) with an `outdated` flag |
| `DELETE` | `/session/{session_id}/delete` | Destroy the session: `204` with no body; `409 session_busy` during active work |

### 5.1. General data rules

- Identifiers (`session_id`, profile, setting, object, part, and instance IDs)
  are opaque strings. Clients must not reconstruct them from a display name.
- A JSON error response uses the following structure:

  ```json
  {
    "error": {
      "code": "invalid_setting_value",
      "message": "The value does not match the setting type.",
      "request_id": "req_...",
      "details": [{"path": "layer_height", "reason": "expected_number"}]
    }
  }
  ```

  `details` is optional; `code`, `message`, and `request_id` are required.
  An invalid request does not modify the session. HTTP codes: `400` for malformed
  JSON, `401` for incorrect login credentials or a missing, invalid, expired,
  or revoked token, `404` for an unknown session/entity or a session owned by
  another account, `409` for an operation incompatible with the current state
  or a replaced result, `413` for an oversized file, `422` for an invalid setting,
  profile, or transformation parameter, and `429` for exceeding the login limit.
  Authentication is checked before session access; an authenticated account
  cannot distinguish an unknown session from one belonging to another account.
- File bodies: `add_model` and `load_project` receive a file; `save_project`
  returns a 3MF and `get_gcode` returns G-code. These bodies are not JSON.
- `DELETE .../delete` returns `204 No Content` on success.

### 5.2. `GET /list_settings`

Return available setting definitions, not a session's current values.
Each of the five original categories `preference`, `print`, `filament`, `extruder`,
and `printer`, plus the additive `entity` category, is a flat list of definitions.

```json
{
  "preference": [
    {
      "id": "language",
      "type": "string",
      "label": "Language",
      "default": {"value":"en"},
      "read_only": false,
      "can_be_disabled": false
    }
  ],
  "print": [],
  "filament": [
    {"id":"filament_setting_id","type":"float","label":"Filament setting","array":"extruder","default":{"value":200}}
  ],
  "extruder": [
    {"id":"extruder_setting_id","type":"float","label":"Extruder setting","array":"extruder","default":{"value":0.4}}
  ],
  "printer": [],
  "entity": [
    {
      "id": "extruder",
      "type": "int",
      "label": "Extruder",
      "default": {"value": 0},
      "min": 0,
      "choices": [
        {"value": "0", "label": "default"},
        {"value": "1", "label": "1"},
        {"value": "2", "label": "2"},
        {"value": "3", "label": "3"},
        {"value": "4", "label": "4"},
        {"value": "5", "label": "5"},
        {"value": "6", "label": "6"},
        {"value": "7", "label": "7"},
        {"value": "8", "label": "8"},
        {"value": "9", "label": "9"}
      ],
      "scopes": ["object", "part", "layer_range"]
    }
  ]
}
```

Every definition must contain `id`, `type`, `label`, and `default`.
`default` is a value object as described below, or `null` if no default exists.
All settings in the `filament` and `extruder` categories have `"array":"extruder"`.
Optional fields are `description`, `unit`, `min`, `max`, `precision`, `choices`,
`read_only`, `can_be_disabled`, `array`, and `scopes`. `scopes`, when present,
lists the entity kinds that may store an override (`object`, `part`, or
`layer_range`); it is omitted when the setting has no entity scope. `extruder` is
the entity-only setting in V1, with `0` meaning the default extruder and `1`–`9`
the numbered choices. The original five category definitions keep their existing
meaning; `scopes` is their only new metadata. `can_be_disabled` indicates whether
the setting is disableable; omission means `false`. A `choices` entry has the
form `{"value":"...","label":"..."}`. `choices.value` is the serialized
setting value, so integer settings use decimal text (for example `"1"`), while
the setting's `default.value` keeps its declared JSON type. `type` remains the
element type;
`array`, when present, describes the number of elements in the value.
Without `array`, the setting represents one value of its type, which may itself
be structured, such as `point2` or `graph`.

Every setting returned by `get_settings` or supplied to `set_settings` is an
object with a required `value` field. `enabled` and `percent` are siblings of
`value`, never nested inside it.

Types and representations of the `value` field:

| `type` | JSON value |
| --- | --- |
| `bool` | Boolean |
| `int` | Integer |
| `float`, `percent` | Number; `percent` uses percentage points (e.g. `25` = 25%) |
| `string`, `enum` | String; enums expose their allowed values in `choices` |
| `float_or_percent` | Number; the sibling `percent` field is `false` for an absolute value, `true` for percentage points |
| `point2` | Two-number array `[x,y]` |
| `point3` | Three-number array `[x,y,z]` |
| `graph` | `{"type":"linear","points":[[x,y],...]}`; type `linear`, `square`, or `spline` |

For a graph, the V1 API exposes only usable points and the type.
Any points retained by the engine outside this range, and its
`begin_idx`/`end_idx` indices, are not representable; writing a graph therefore
reconstructs a complete range over the supplied points.

For `point2` and `point3`, components follow the indicated order;
the definition's `unit` field specifies their unit when applicable.

- `enabled` is required for a disableable setting and absent otherwise.
  The value remains present, typed, and retained even when `enabled` is `false`:
  disabling a setting does not erase its value.
- `percent` is required for `float_or_percent`, absent for other types,
  including `percent`, whose unit is already known.
- For a scalar setting these flags are booleans. For a setting with `array`,
  `value` is an array and every applicable flag is a boolean array of the same
  length; index i describes `value[i]`.
- A point or graph without `array` remains a structured scalar value:
  the flags do not apply to its coordinates.

Example setting values (illustrative IDs):

```json
{
  "setting_id": {"value":0.8},
  "setting_id2": {"enabled":true,"value":0.8},
  "setting_id3": {"enabled":false,"value":"text"},
  "setting_id4": {"percent":false,"value":0.8},
  "setting_id5": {"percent":[false,false],"enabled":[true,false],"value":[0.8,2]}
}
```

`null` does not replace a disabled setting's value. `get_setting` and
`set_setting` continue to use SuperSlicer serialization.
The `array` field accepts exactly these values:

| `array` value | Value cardinality |
| --- | --- |
| Field absent | One value of type `type` |
| `"7"` | Fixed-size array: seven values, indexed from 0 to 6 |
| `"variable"` | Variable-length array with elements of type `type` |
| `"extruder"` | Array containing exactly one value per extruder of the selected printer |

Thus, a `float` setting with `array: "extruder"` describes one `float` value per
extruder. For this array kind, `default` describes a single element
(e.g. `{"value":0.4}`), used as a common extruder default; its applicable flags
are also scalar. `get_setting` and `set_setting` transport the complete effective
value through SuperSlicer serialization; `get_settings` and `set_settings`
represent it as an object whose `value` contains exactly one value per extruder,
with parallel arrays for applicable flags. For `array: "7"` or
`array: "variable"`, `default` also uses an object with `value` and applicable
flags as arrays of the specified cardinality.

Example definitions:

```json
[
  {"id":"layer_height","type":"float","label":"Layer height","default":{"value":0.2}},
  {"id":"nozzle_diameter","type":"float","label":"Nozzle diameter","array":"extruder","default":{"value":0.4}},
  {"id":"calibration_values","type":"int","label":"Calibration values","array":"7","default":{"value":[0,1,2,3,4,5,6]}},
  {"id":"custom_values","type":"string","label":"Custom values","array":"variable","default":{"value":[]}},
  {"id":"setting_id5","type":"float_or_percent","label":"Disableable values","array":"extruder","can_be_disabled":true,"default":{"value":0.8,"enabled":true,"percent":false}}
]
```

### 5.3. `POST /create_session`

The body is empty or a JSON object whose only optional field is `name`, a
string or `null` (§5.3.1). Return `201 Created`:

```json
{"session_id":"session_opaque_id","name":null}
```

### 5.3.1. Session name: `POST .../set_name` and `GET .../get_name`

A session name is the equivalent of the GUI project name. It is optional,
kept as given for display, and only sanitized when it becomes a download file
name (§5.14). A name is valid UTF-8 of 1 to 255 bytes without control
characters; `null` means no name, and an empty string is refused with
`400 invalid_request`.

- `set_name` takes `{"name":"Bracket"}` or `{"name":null}`; the field is
  required and no other field is accepted. It returns `200 {"name":...}`.
  The name never affects slicing: the revision increases, but status, G-code,
  and viewer result are kept. Setting the current name again changes nothing.
  An invalid body returns `400`, checked before the slicing state; a valid
  request during slicing returns `409 session_busy`.
- `get_name` has no body and returns `{"name":"Bracket"}` or
  `{"name":null}`.
- As in the GUI, `load_project` names the session after the uploaded file
  without its last extension (`support.3mf` gives `support`; no name when the
  result is not a valid name). `add_model`, transformations, instance counts,
  settings, profiles, and `clear_platter` keep the name.

### 5.4. `GET .../list_profiles`

Return profiles as flat lists by type. `id` is opaque; `name` is the full unique
preset name, not its possibly repeated alias. System presets additionally have
`vendor` (vendor display name); user presets omit it. Prints and filaments have a
boolean `compatible`; printers do not.

The profile root follows the GUI data layout: optional `vendor/*.ini` system
bundles and optional flat `print/*.ini`, `printer/*.ini`, `filament/*.ini` user
presets. System inheritance is resolved and abstract `*…*` presets are excluded.
All system presets are visible without GUI installed-printer filtering. Vendor
files are read in filename order; duplicate names retain the first preset and
produce a startup warning. A flat user preset named like a system preset is
ignored by the core loader. Restart after modifying the catalog.

Vendor bundles without printer models/variants and without template metadata
are skipped as in the GUI, with a startup warning naming the file. This includes
`Templates.ini` in the pinned 2.7 profile distribution. Malformed/unreadable INIs
and invalid vendor sections remain fatal startup errors.

```json
{
  "print": [{"id":"print-id", "name":"0.20mm NORMAL .4N", "vendor":"Custom", "compatible":true}],
  "printer": [{"id":"printer-id", "name":"Basic 0.4 nozzle", "vendor":"Custom"}],
  "extruder": [],
  "filaments": [{"id":"filament-id", "name":"Basic PLA @BASIC", "vendor":"Custom", "compatible":true}]
}
```

Catalog lists are independent of the ordered arrays sent to `set_profile`.
Extruder profiles do not exist yet: `extruder` is an empty list in V1.
Settings in the `extruder` category remain available, notably through the
printer profile; they do not require independent extruder profiles to exist.

Optional query parameters: `printer=<id>`, `print=<id>`, and
`compatible_only=0|1`. Without overrides, compatibility is evaluated against the
session's printer and print, not the catalog defaults. Overrides evaluate another
combination without changing the session. `compatible_only=1` filters prints and
filaments; printers are always returned. Print compatibility checks the printer;
filaments also check the print and use the **first extruder**. `set_profile` still
validates every actual filament slot and returns `422` for incompatibility.

Conditions use the session's current configuration, including `set_setting` and
`set_settings` edits, even when a catalog ID remains. Its catalog name and vendor
are retained for identity checks. An explicit printer/print query override uses
that catalog preset instead of the session's edits for the overridden category.

An embedded/modified session profile with no catalog ID is evaluated from the
session configuration with an empty name. Named compatibility lists cannot match
it; conditions still apply. An anonymous printer has no vendor identity, so
conditions are evaluated without restricting it to one vendor.

After authentication, refusals are checked in this order:

| Check | Response |
| --- | --- |
| Method is not GET | `404 route_not_found` |
| Nonempty body; unknown/repeated query key; empty value; compatible_only not 0/1 | `400 invalid_request` |
| Missing session or another account's session | `404 session_not_found` |
| Engine not initialized | `503 engine_not_initialized` |
| Unknown printer/print ID, or non-FFF printer | `422 invalid_profile` |

Otherwise the response is `200` with the lists above.

### 5.5. `POST .../set_profile`

The body contains all four keys. `print` and `printer` are profile IDs;
`extruder` and `filaments` are arrays of IDs ordered by slot, indexed from zero.
For a printer with N extruders, each nonempty array contains exactly N IDs;
the ID at index i applies to extruder i.

```json
{
  "print": "print-id",
  "printer": "printer-id",
  "extruder": [],
  "filaments": ["pla-slot-0", "petg-slot-1"]
}
```

An empty `filaments` array selects the default filament profiles supplied by
the printer profile. In V1, `extruder` must be empty and means “no independent
extruder profile”; no default extruder profile is required. A nonempty `extruder`
array references unavailable profiles and returns `422`. Slicing is possible
with `extruder: []`.

Validate the complete selection before applying it. Loading a profile clears
local modifications to settings in its category, even when selecting the same
profile again. `set_profile` reloads the complete print/printer/filament
configuration; session preferences are retained. The `200 OK` response returns
the effective selection in the same form.

Preparing defaults for `create_session` and applying `set_profile` run on the
import worker pool, not the network thread. Other sessions remain responsive.
Selection captures the session revision before preparing its configuration:
concurrent changes cause `409 session_changed`, and deletion causes `404`;
neither permits a late selection to overwrite newer state. Shutdown joins these
workers before releasing the catalog and sessions.

### 5.5.1. Configuration loading and engine changes

The following target rules apply when loading profiles or a project's
configuration, not to partial updates through `set_settings`:

- recognized settings present in the loaded configuration are applied;
- settings absent from replaced categories are initialized to system defaults,
  without reusing the session's previous modifications;
- settings unknown/unavailable to the engine are not applied. Settings from a
  print or printer profile are preserved in `print_custom_setting`, as a JSON
  object whose `print` and `printer` groups each contain key/value pairs.
  Settings from a filament profile are preserved in `filament_custom_setting`,
  as an array of JSON objects, one per slot. Values remain raw INI strings
  and are never interpreted by the engine.
- Unknown keys and known keys placed in the wrong category are classified
  according to the profile that supplied them. A value explicitly present in
  `print_custom_setting` or `filament_custom_setting` overrides the captured value
  for the same key; the existing `print_custom_variables` and
  `filament_custom_variables` options are not used.
- Recomposing, copying, and serializing a configuration preserves this data.
  For each slot, `filaments: []` tries the corresponding printer default,
  then the printer's other defaults, then the first visible compatible filament;
  no available candidate returns `422`.

A profile embedded in a project need not exist in the installed catalog for
its recognized settings to be loaded into the session. Loading a project
does not create or modify shared installed profiles.

A 3MF project's configuration is flat: the profile supplying an unknown key
is no longer known. Its key name is used as a provenance hint, with N being
the project's extruder count:

- a `filament_*` key whose raw value is an array of N elements is distributed
  into `filament_custom_setting`, one element per slot. A value starting with
  `"` is a string list separated by `;`; otherwise elements are separated by `,`;
- a key starting with `printer_`, `firmware_`, `machine_`, `printhost_`, `host_`,
  `bed_`, `thumbnails_`, `gcode_`, or an extruder prefix (`extruder_`, `retract_`,
  `deretract_`, `nozzle_`, `tool_`) goes into the `printer` group: in V1,
  extruder settings come from the printer profile;
- every other key, including a `filament_*` key of a different size, goes into
  the `print` group.

Unknown keys from object, volume, or layer-range configurations are collected
by the same loader and follow the same rules.

### 5.6. `GET .../get_setting/{setting_id}`

Read a session setting's effective value. No request body.
The response uses the same serialized representation as the `serialized_value`
segment of `set_setting`, allowing the value to be reused directly:

```json
{"setting_id":"layer_height","serialized_value":"0.2"}
```

The setting must apply to the session; an unknown or inapplicable identifier
returns the common error response.

### 5.7. `POST .../get_settings`

The JSON body explicitly requests settings to read. It must contain a nonempty
`setting_ids` array; an empty list never means “all settings” and is refused:

```json
{"setting_ids":["layer_height","perimeters"]}
```

The response is a JSON object mapping each requested `setting_id` to its value
object (`value`, plus `enabled` and `percent` when applicable). It follows the
`/list_settings` and `set_settings` contracts, including cardinality when `array`
is present. Values from profiles and applicable defaults are included;
this is not just the list of explicitly modified settings. Only requested IDs
are returned. An unknown or inapplicable ID returns the common error response.

```json
{
  "layer_height": {"value":0.2},
  "perimeters": {"value":3}
}
```

All `/list_settings` categories, including `preference`, are accessible in the
session context; preferences remain isolated per session.

### 5.8. `POST .../set_setting/{setting_id}/{serialized_value}`

This route modifies one setting. There is no request body. The value is a string
produced by SuperSlicer configuration serialization (the INI value format for
that type), then encoded as a URL path segment. Examples: `0.2`, `25%`, `0.4,0.6`,
or `rectilinear`, depending on the type. Reserved characters are URL-path encoded;
values exceeding practical path size must use `set_settings` or require a
variant with a JSON body.

The JSON response returns the accepted value in serialized form:

```json
{"setting_id":"layer_height","serialized_value":"0.2"}
```

### 5.9. `POST .../set_settings`

The body is a JSON object: each key is a `setting_id` and each entry contains
`value`, plus `enabled` and `percent` when applicable, as specified by the
corresponding `/list_settings` definition. This format does not use the INI
serialization of `set_setting`.

```json
{
  "layer_height": {"value":0.2},
  "perimeters": {"value":3},
  "fill_pattern": {"value":"rectilinear"},
  "nozzle_diameter": {"value":[0.4,0.6]},
  "external_perimeter_speed": {"value": 60, "percent": true}
}
```

The request is partial by setting: omitted IDs remain unchanged. Each supplied
setting contains its `value` and every applicable flag; there is no implicit
update of a single flag. Validate types, cardinalities, and every key before
applying any entry; on error, no change is applied. Return the typed values
actually applied in an `updated` object:

```json
{
  "updated": {
    "layer_height": {"value":0.2},
    "perimeters": {"value":3},
    "fill_pattern": {"value":"rectilinear"},
    "nozzle_diameter": {"value":[0.4,0.6]},
    "external_perimeter_speed": {"value":60,"percent":true}
  }
}
```

### 5.10. `POST .../add_model` and `POST .../load_project`

The file is sent as `multipart/form-data` in a single field named `file`, with
its filename. `add_model` accepts STL, OBJ, or 3MF as a model; `load_project`
accepts a 3MF project, replaces the existing scene, and reloads the slicing
configuration from its saved profiles/settings. Previous local modifications
are cleared; absent slicing settings and categories revert to system defaults
according to §5.5.1. Session preferences are retained. Unknown settings are
preserved as custom data according to §5.5.1. `add_model` does not apply a 3MF's
project configuration or other model-level data such as per-height custom
G-code: only its objects are added. Both routes return newly created IDs:

```json
{"object_ids":["object-id-1"],"object_count":1}
```

Multipart data is processed incrementally; authentication, the session, and its
state are checked from the headers, before any byte is written. The payload is
written into the work directory's `uploads` subdirectory under an exclusively
created random name; the client-supplied name is used only for the extension.
`limits.max_upload_bytes` limits file bytes, defaulting to 1 GiB. Temporary files
are removed after success or rejection, and files left by an abrupt stop are
removed at startup (one server per work directory). Everything following the
closing boundary is an ignored epilogue.

Model parsing runs on a dedicated pool of `limits.import_threads` threads
(default 2) without blocking other requests. `timeouts.request_seconds` is one
deadline for a complete JSON request, and an idle timeout between reads for an
upload. File downloads (`save_project`, `get_gcode`) also use an idle write
timeout, rearmed whenever a write advances; active transfers may last longer
than `request_seconds`. An error sent before upload completion (`401`, `404`, `409`, `413`,
`422`, etc.) is followed by bounded draining of the remaining body (1 MiB or 2 s)
before closing, so the client receives the response. An engine-unreadable file
produces `422 invalid_model`; other internal errors produce `500 internal_error`
with a generic message.

`load_project` accepts only the `.3mf` extension (`422 unsupported_model_format`
before any storage). The project is recomposed as in the interface: settings are
split into print, printer, and filament profiles on a private copy of the catalog presets it names;
a 3MF without configuration resets all settings to system defaults. All scene
data read by the 3MF loader is retained (object settings, modifiers, layer ranges,
per-height G-code, etc.). The scene, configuration, session profile selection,
and session name (§5.3.1) are replaced in one operation; all previous object,
part, and instance IDs are invalidated. In the recorded selection, a profile keeps its catalog ID only if
the project uses the installed profile unchanged; an embedded or modified profile
has no ID. A supplied `post_process` is retained but never executed. Reject the
project with `422 invalid_project`, without changing the session, if it is not
FFF, defines no nozzle, its configuration (checked as loaded, before automatic
index correction), an object, volume, or layer range uses an extruder absent from
the project, engine cross-validation fails, a `*_custom_setting` is malformed,
or the file is unreadable.

### 5.11. `GET .../list_objects`

Return the complete scene. Object, part, and instance IDs are unique in one
session identifier namespace and remain stable while their entities exist.
Each may be used directly in `transform/{id}`. A part corresponds to a model
volume. An instance matrix places it in platter space; a volume matrix places
it in instance space. For a mesh point,
`p_platter = M_instance × M_volume × p_mesh`. An object is a grouping: its exposed
matrix is identity; effective transformations reside on its instances and volumes.

```json
{
  "objects": [
    {
      "id": "object-id",
      "name": "part.stl",
      "cut": false,
      "has_settings": false,
      "variable_layer_height": false,
      "matrix": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "parts": [
        {"id":"part-id", "name":"part.stl", "type":"part", "painted":[], "has_settings":false, "matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}
      ],
      "instances": [
        {
          "id": "instance-id",
          "matrix": [1,0,0,10,0,1,0,20,0,0,1,0,0,0,0,1]
        }
      ],
      "layer_ranges": [
        {"id":"range-id", "min":2, "max":4, "has_settings":true}
      ]
    }
  ]
}
```

`cut` is `true` when the object belongs to a cut-object group whose dependent
objects must be invalidated together before restricted edits. `type` has these
stable values:

| `type` | Meaning |
|---|---|
| `part` | Printable solid part |
| `negative_volume` | Volume subtracted from solid geometry |
| `modifier` | Parameter modifier |
| `support_blocker` / `support_enforcer` | Support-demand blocker or enforcer |
| `seam_position_center` / `seam_position_center_z` / `seam_position_inside` | Seam helper volume |
| `brim_patch` / `brim_negative` | Brim enforcer or blocker |
| `support_column` | Volume that asks for sparse support inside it |
| `no_support_zone` | Volume that forbids support inside it |
| `support_interface_column` | Volume that asks for dense interface support inside it |

`painted` lists the painting channels present on the part. Stable channel names
are `support` for support painting and `mmu` for multi-material painting,
followed by annotation keys such as `builtin:seam`.
`has_settings` reports whether that entity stores any explicit configuration
overrides; it does not include inherited values or filter by the keys exposed by
`list_entity_settings`. A part created by `add_part` or `add_shape` stores the
default `extruder = 0`, so its `has_settings` is `true`. `layer_ranges` is
ordered by height bounds and contains each range's stable ID, exact `min`/`max` bounds
in millimeters, and override-presence flag. The ID remains stable when its bounds
are changed through `set_layer_range`.

`variable_layer_height` is `true` when the object stores a non-empty layer-height
profile. It reports stored data even when a later geometry change makes that
profile inapplicable; use `get_layer_height_profile` to inspect `applied`.

### 5.12. `POST .../transform/{id}`

The route resolves `id` within the session, without an additional parent segment:

- object ID: transform all its instances;
- instance ID: transform only that instance;
- volume/part ID: transform that volume, shared by the object's instances.
  Other volumes are not modified; instances are retained unless bed placement
  is explicitly requested.

An unknown ID returns `404`. An object transformation is atomic across all
its instances.

The body contains `matrix`, a flat array of 16 finite numbers in row-major
order: `[m00,m01,m02,m03,m10,...,m33]`, equivalent to four rows of four values.
The homogeneous matrix acts on column vectors; translation is in millimeters
at indices 3, 7, and 11, and the last row (indices 12 through 15) is `[0,0,0,1]`.
`action` accepts `compose` or `replace`; omission defaults to `compose`.
`compose` applies a relative transformation. For an object or instance,
`replace` directly sets the instance matrix to the received matrix, regardless
of the requested space. Optional `ensure_on_bed` is a boolean, default `false`.
Optional `instance` is a string containing a reference instance ID and is valid
only for volume targets. It defaults to the object's first (original) instance.
The referenced entity must be an instance of that volume's object, even when
the chosen space does not affect the formula. Unknown fields are refused.

The required `space` field preserves the GUI's three rotation/translation
coordinate-space choices:

| `space` | Coordinate space |
| --- | --- |
| `platter` | Global platter space for instances; global axes about the reference instance origin for relative volume operations |
| `object` | The placed object's space, i.e. its instance space |
| `volume` | Target volume/part's local space; for an object or instance target, instance space |

Space conversion must account for parent transformations. With `A` denoting
the received matrix, `I` the instance matrix, `V` the volume matrix, and
`L_ref` the reference instance's linear transform with zero translation,
composition is defined as follows:

- instance in `platter`: `I' = A × I`; in `object` or `volume`: `I' = I × A`;
- volume in `object`: `V' = A × V`; in `volume`: `V' = V × A`;
- volume in `platter`: `V' = L_ref⁻¹ × A × L_ref × V`.

Relative volume operations in `platter` follow the GUI convention: the instance
offset is not part of the basis conversion. A rotation without supplied pivot
translations rotates around the reference instance origin, not the platter
origin. A platter pivot must therefore be expressed relative to that instance
origin when building `A`. Translations behave as with the full-instance formula.
No additional pivot or object recentering is applied.

Volume `replace` sets `V' = A` in `object` and `volume` space. In `platter`, it
sets `V' = I_ref⁻¹ × A` using the complete reference instance transform, including
its offset: absolute replacement places a mesh point at `A × p` in platter
space for that reference instance. Other instances share the resulting `V'`.

For a numerical example without bed placement, let `I_ref = T(10,0,0)`,
`V = T(2,0,0)`, and `A` be a positive 90-degree rotation around Z. For the
mesh point `p = (0,0,0)`, both requests use `space: "platter"`:

| Action | Volume-local point `V' × p` | Platter point `I_ref × V' × p` |
| --- | --- | --- |
| `compose` | `(0,2,0)` | `(10,2,0)` |
| `replace` | `(-10,0,0)` | `(0,0,0)` |

The relative rotation preserves the reference instance origin at `(10,0,0)`;
absolute replacement instead places the mesh origin at `A × p = (0,0,0)`
in platter space. Coordinates and translations in this example are in millimeters.

For an object target, apply the same operation to each instance. `replace`
therefore imposes `I' = A` on all instances and superimposes them. No implicit
pivot is applied: to rotate around point `p`, the client supplies
`T(p) × R × T(−p)` expressed in the chosen space.

Each resulting matrix must be finite, invertible (`|det(I'.linear)| > 1e-9`),
and shear-free: columns `cᵢ` of its linear part must be pairwise orthogonal,
with relative tolerance `|cᵢ · cⱼ| ≤ 1e-9 × ‖cᵢ‖ × ‖cⱼ‖` for `i ≠ j`.
The REST check compares normalized column directions, without using the
`Geometry::Transformation::has_skew()` heuristic, which may reject a mirror
combined with a 3D rotation. Mirrors and non-uniform scales are accepted,
including rotations around arbitrary axes. Validate the composition result,
since two shear-free transformations may introduce shear when composed.
All matrices are checked before application; an error on one instance refuses
the entire object transformation without changing the scene or its IDs.
For a volume, validate both `V'` and every `I_k × V'` in its object before
application. A singular or sheared combination in any instance refuses the
whole operation without changing the revision or any scene entity.

With `ensure_on_bed: true`, each targeted instance is then translated in Z to
bring its lowest point to zero, computed by `ModelObject::get_instance_min_z()`
on its model parts. For a volume target, every instance of its parent object
is placed on the bed after the volume change. Placement is per instance, without XY arrangement.
Without this option, the matrix is applied exactly. Bed placement without
valid model geometry is refused.

| Case | Response |
| --- | --- |
| Invalid authentication | `401` |
| Unknown/foreign session or unknown ID | `404 session_not_found` / `404 entity_not_found` |
| Unknown reference `instance` ID | `404 entity_not_found` |
| Method other than POST | `404 route_not_found` |
| Malformed JSON | `400 invalid_json`, including parser `details` |
| Missing body or JSON content type, missing field, wrong type, unknown field | `400 invalid_request` |
| Matrix without 16 finite numbers, invalid last row, unknown action or space | `422 invalid_transform` |
| Singular/sheared result or instance-volume combination, invalid bed placement | `422 invalid_transform` |
| Reference ID of another type/object, or `instance` supplied for an object/instance target | `422 invalid_transform` |
| Valid request while the session is slicing (body errors are reported first) | `409 session_busy` |
| Concurrent mutation before commit | `409 session_changed` |

The response returns the target ID, action, space, and received matrix:

```json
{"id":"instance-id","action":"compose","space":"platter","matrix":[1,0,0,10,0,1,0,0,0,0,1,0,0,0,0,1]}
```

Effective matrices can subsequently be read through `list_objects`.
The old paths with an object ID followed by a part ID are not part of the contract.

### 5.13. `POST .../set_instance_count/{object_id}`

The body sets the total number of instances, including the original:

```json
{"count":3}
```

`count` is a strictly positive integer, not a boolean, string, or floating-point
number. The body must contain only `count`. The response contains the object
and its ordered instance IDs after the change:

```json
{"object_id":"object-id","count":3,"instance_ids":["i0","i1","i2"]}
```

New instances are superimposed on the original: they copy its transformation
and receive new IDs. No automatic arrangement is performed. The client can then
move them individually through `transform/{id}`. The original instance is first
in the list; decreasing `count` removes instances from the end, without changing
retained instances' IDs or transformations. `count=1` retains the original.

Copies use the original instance's current transformation at index 0, even if
it has moved since creation; they do not copy another instance's placement.
Removed instance IDs become unusable immediately at the scene commit and return
`404 entity_not_found` through `transform/{id}`. Adding/removing instances and
updating their ID mappings form the same revision-checked atomic operation;
a refused commit neither removes old IDs nor adds new ones.

`limits.max_instances_per_object` is a configurable positive integer, default
`1000`, bounding counts requested through this route. Values above this limit
are rejected before scene copying or allocation. This route limit does not
rewrite imported projects that already contain more instances. Every request
must nevertheless satisfy the configured limit: requesting the same over-limit
count returns `422 invalid_instance_count`, not an unchanged-count success.
Reducing such an object to a positive count within the limit is allowed.

A valid request matching the current count returns `200` with the current
ordered IDs without committing: revision, G-code availability, and viewer state
are unchanged. An effective count change returns the session to `ready`, makes
current G-code unavailable, and retains the previous viewer as outdated.

| Case | Response |
| --- | --- |
| Invalid authentication | `401` |
| Unknown/foreign session or unknown entity ID | `404 session_not_found` / `404 entity_not_found` |
| Method other than POST | `404 route_not_found` |
| Malformed JSON | `400 invalid_json`, including parser `details` |
| Missing body/JSON content type/count, wrong count type, extra field | `400 invalid_request` |
| Count below 1 or above the configured limit | `422 invalid_instance_count` |
| Instance or volume ID supplied instead of an object ID | `422 invalid_instance_count` |
| Valid request while the session is slicing, including an otherwise unchanged count (body errors are reported first) | `409 session_busy` |
| Concurrent mutation before commit or ID snapshot capture | `409 session_changed` |

### 5.14. `POST .../slice`, `GET .../state`, `cancel_slicing`, `get_gcode`, `clear_platter`, `save_project`, and `delete`

- `/slice` has no body. It starts slicing in the background and returns
  `202 Accepted` with the same state shape as `GET .../state` below.
- A second `/slice` call while the session is already `slicing` returns
  `409 Conflict`.
- Slicing uses a fixed worker pool configured by the positive integer
  `limits.slice_threads` (default `2`). Additional accepted sessions wait in an
  unbounded queue; queued sessions are already in the `slicing` internal state.
- Slice refusal order is authentication (`401`), method/path (`404`), initialized
  engine (`503`), body validation (`400 invalid_request` for any nonempty body),
  owning session (`404 session_not_found`), then scene/state. An empty scene
  returns `409 empty_scene`; an already active session returns `409 session_busy`.
- `GET /session/{session_id}/state` has no body and returns `200 OK` with
  the session's current state and live main/secondary `PrintStatus` progress:

  ```json
  {
    "etat": "slicing",
    "primary_message": "Slicing objects",
    "primary_percentage": 90,
    "secondary_message": "Generating infill",
    "secondary_percentage": 100
  }
  ```

  `etat` is `ready`, `slicing`, `sliced`, or `failed`. Both message fields are
  strings; both percentages are integers, with `-1` meaning unavailable progress
  and `0` through `100` representing percentage points. Values received outside
  that range are clamped. The primary channel is indicative overall progress and
  the secondary channel is indicative detailed progress. Messages are the
  engine's English, non-translated text. Message templates use their supplied
  arguments, including `%%` as a literal percent sign; an invalid
  template is returned as raw text rather than failing slicing.
- Progress is a current snapshot, not an event stream. Updates from independent
  producers may be interleaved, and clients must not assume monotonic
  percentages, regular update intervals, or ordering between the two channels.
  An empty update with an available percentage retains that channel's previous
  message; an empty update with unavailable progress clears it. Control events
  for scene/support/preview reload, warning refresh, slicing/G-code completion,
  and forced display do not populate either progress channel.
- Outside `slicing`, progress is blank: both messages are empty and both
  percentages are `-1`. The sole exception is `failed`, where
  `primary_message` contains the sanitized English error while the other three
  progress fields retain their unavailable defaults. Both progress messages and
  errors mask native and generic UTF-8 renderings of server work and result paths
  with `<output>`.
- `state` is an engine-independent registry read. Its route has exactly the three
  path segments shown, accepts only `GET`, and requires an empty body. Authentication
  is checked before body validation. An authenticated account receives the same
  `404 session_not_found` for a missing session and another account's session.

  | State request condition | Response |
  |---|---|
  | Nonempty request body, including malformed JSON | `400 invalid_request` |
  | Missing or invalid authentication | `401 unauthorized` |
  | Unknown method/path | `404 route_not_found` |
  | Unknown session or another account's session | `404 session_not_found` |

  Engine initialization does not affect these responses and never produces a
  `503` for `state`.

  TODO(progress rework): indicative progress and its formatting may change with
  the libslic3r progress-model redesign.
- The initial `/slice` response may have unavailable progress, for example:

  ```json
  {
    "etat": "slicing",
    "primary_message": "",
    "primary_percentage": -1,
    "secondary_message": "",
    "secondary_percentage": -1
  }
  ```

  Empty messages and `-1` percentages indicate unavailable initial progress,
  not permanent defaults; subsequent state reads expose live progress when
  available. Storage or scene preparation can fail after work admission and
  return `202` already in `failed`, using the same state shape.
- The sliced input is an immutable scene and configuration snapshot captured
  after work admission. Later mutations cannot commit while that work is active.
- Validation, engine, and storage failures transition the session to `failed`.
  No `post_process` command is executed, including commands retained in imported
  profiles or projects.
- `sliced` is published only after successful slicing, G-code export, and
  preparation of the viewer result. No partial file or result is published
  on failure or cancellation.
- Server shutdown cancels every running and queued slice like `cancel_slicing`,
  waits for slice and import workers to finish, returns their retained session
  contexts to `ready`, and removes unpublished output.
- `POST .../cancel_slicing` has no body and requests that active slicing stop.
  It is the only mutation accepted while `etat: "slicing"`, and returns
  `202 Accepted` immediately with the same state shape as `GET .../state`:
  - queued work that has not started ends at once: the response already reports
    `etat: "ready"` and the engine never runs;
  - started work keeps `etat: "slicing"` until the engine has effectively
    stopped, then reports `ready`; until then new slices and mutations still
    return `409 session_busy`, and a repeated cancellation returns `202` again;
  - an accepted cancellation always ends in `ready`, even if the engine finished
    or failed meanwhile: the computed result is discarded, no error is recorded,
    and the previous viewer result, if any, is retained as outdated;
  - the inactivity period restarts when work has effectively stopped.

  Without active slicing, including after a `sliced` publication that won the
  race, it returns `409 not_slicing`. Refusal order is authentication (`401`),
  method/path (`404 route_not_found`), body (`400 invalid_request` for any
  nonempty body), owning session (`404 session_not_found`), then state
  (`409 not_slicing`). Engine initialization never produces a `503` here.
- `GET .../get_gcode` has no body and streams the `.gcode` file of the current
  result (`Content-Type: text/x.gcode`, `Cache-Control: no-store`,
  `Content-Disposition: attachment`). `get_gcode_for_view` remains the viewer's
  JSON output.
  - A result is current only in `etat: "sliced"` with no invalidation since:
    before any success, during slicing, after an effective change, a failure, or
    a cancellation it returns `409 no_gcode`.
  - A download already started keeps the published file until it completes or
    is abandoned, even if the session is changed, resliced, or deleted meanwhile.
  - The optional `file_name` query and the portable naming rules are those of
    `save_project`, with the `.gcode` extension: the requested name, else the
    session name, else `project.gcode`.
  - Refusal order is authentication (`401`), method/path
    (`404 route_not_found`), body and query (`400 invalid_request`), owning
    session (`404 session_not_found`), then `409 no_gcode`. Engine
    initialization never produces a `503` here. A file that cannot be opened
    for sending returns `500 internal_error`.
- `/clear_platter` has no body and returns `{"cleared_object_count":3}`.
  Session profiles, settings, preferences, and name are retained. Removing the
  objects and invalidating every object, part, and instance ID form one atomic
  commit; the previous viewer result is retained as outdated. Clearing an
  already empty platter returns `{"cleared_object_count":0}` without any
  change: revision, G-code, viewer, and status are untouched, so a `failed`
  status stays `failed` until the next slicing or effective change.
  A non-empty body returns `400 invalid_request`, checked before the slicing
  state; a valid request during slicing returns `409 session_busy`, and a
  concurrent mutation returns `409 session_changed`.
- `GET /session/{session_id}/save_project` has no body and returns a streamed
  binary 3MF file (`Content-Type: model/3mf`, `Cache-Control: no-store`), not JSON.
  - **Contents and snapshot:** One committed scene/configuration/name snapshot is
    exported on the model worker pool. Saving is allowed during slicing and does
    not change revision, status, name, G-code, viewer snapshot, or entity IDs.
    Later mutations or session deletion do not change a captured export.
    The archive contains geometry, object/volume hierarchy, instances, transforms,
    slicing configuration, embedded profile settings, and unknown print/printer/
    filament settings preserved in custom settings. No thumbnail is generated.
    Source paths are reduced to file names; `print_host`, `printhost_apikey`,
    `printhost_cafile`, and `printhost_port` are stripped as in GUI secure exports.
  - **Reload:** `load_project` recovers settings with new entity IDs and normalizes
    single-volume offsets into instance translations without changing world-space
    geometry. Inactive phony option values are not serialized. Session-only
    preferences and opaque profile IDs are not restored. An embedded preset is
    named after the loaded file, followed by its original source in parentheses
    only when that source differs; repeated save/reload cycles keep the same name.
  - **Name:** The optional `file_name` query uses form decoding (`+` means space;
    `%HH` decodes bytes). Empty pairs are ignored; unknown keys, repeated decoded
    keys, malformed escapes, decoded NUL, and empty `file_name` return
    `400 invalid_request`. A portable `.3mf` name is derived from the requested
    name, then session name, then `project.3mf`. Unsupported characters become
    underscores, device names gain an underscore prefix, trailing dots/spaces are
    removed, and an existing `.3MF` suffix retains its case. Names containing only
    dots/spaces fall back rather than fail. `Content-Disposition: attachment`
    supplies an ASCII fallback (one underscore per non-ASCII code point; percent
    signs also become underscores) and RFC5987 `filename*` preserving exact UTF-8.
  - **Errors:** After authentication, refusal order is method/path
    (`404 route_not_found`), engine (`503 engine_not_initialized`), body
    (`400 invalid_request`), query (`400 invalid_request`), owning session/snapshot
    (`404 session_not_found`), then empty scene (`409 empty_scene`). Export/storage
    failures return generic `500 internal_error` without exposing paths.
  - **Temporary files:** `rest-export-` files in `work/exports` are deleted after
    responses complete or disconnect; stale matching files are removed at startup
    without deleting unrelated files.
- Slice failures sanitize the configured `paths.work` location and each
  random result directory as `<output>` when an underlying error contains that
  path. A result-directory preparation failure records the generic literal
  `The server could not prepare the result storage.`
- Each success writes `work/results/rest-result-<random>/output.gcode`. The
  directory is owned until the last session viewer, viewer response, or download
  reference releases it. Startup removes unowned matching directories and keeps
  unrelated files and results still owned across an in-process restart.
  A failed replacement retains the prior outdated viewer and
  its one old result directory without leaving a new result directory.
- `DELETE .../delete` has no body and returns `204 No Content`; the session ID
  and data are invalidated, and retained results are released once no viewer
  page or download in progress still uses them. A session that is slicing,
  including a cancellation still stopping, cannot be deleted: `409 session_busy`.
  Refusal order is authentication (`401`), method/path (`404 route_not_found`),
  body (`400 invalid_request`), owning session (`404 session_not_found`), then
  `409 session_busy`. Inactive sessions are also removed by a periodic sweep.

#### Slice errors

| Condition | Response |
|---|---|
| Nonempty request body | `400 invalid_request` |
| Missing or invalid authentication | `401 unauthorized` |
| Unknown route or method | `404 route_not_found` |
| Unknown session or another account's session | `404 session_not_found` |
| Already queued or slicing | `409 session_busy` |
| Empty scene before work admission | `409 empty_scene` |
| Uninitialized engine | `503 engine_not_initialized` |

#### Result validity

- Any effective scene, profile, or setting change affecting slicing returns
  `etat` to `ready` and makes `get_gcode` unavailable (`409`) until the next
  success. This includes `clear_platter` and `load_project`; a preference with
  no effect on slicing does not make the result stale.
- The last successful viewer result is retained with `outdated: true`.
  It remains readable in `ready`, during a new slicing operation, or after its
  failure. Its metadata describes the old scene, not the modified scene.
- Starting a new slicing operation makes G-code unavailable and marks the old
  viewer result `outdated: true`. Success atomically replaces the results;
  cancellation or failure retains only the old stale viewer result.
  Without a prior success, the viewer returns `409`.
- Registry erase or inactive expiration releases retained results once external
  viewer/download references are gone. A session cannot expire while slicing.

### 5.15. `GET .../get_gcode_for_view`

Return versioned JSON with metadata, layers, and one page of moves from the
session's last successful slicing result, even if stale. The request accepts
`offset` (zero-based, default `0`), `limit` (default `10000`, between `1` and
`100000`), and `result_id`, as decimal query parameters; any other key, a
repeated key, a non-numeric or out-of-range value, or `offset>0` without
`result_id` returns `400 invalid_request`. The route returns
`409 no_viewer_result` if no successful result has been retained. Metadata and
layers are included in every page; only `moves` is paginated. An `offset` at or
beyond `total_moves` returns an empty `moves` list with `next_offset: null`.
The route only reads the session (no `503`). Refusal order is authentication,
method/path, body, query, owning session, then `409`. An internal failure while
building a page returns `500 internal_error`.

Every response contains an opaque `result_id` and an `outdated` boolean.
The first request at `offset=0` may omit `result_id` to retrieve the latest
retained result. Subsequent pages (`offset>0`) must supply that ID.
If the result has been replaced, return `409` with code `result_replaced`,
without supplying a page from another slicing operation. Invalidation changes
`outdated` but not the retained result's ID, moves, metadata, or layers;
a new success produces a new ID and `outdated: false`.

Units are explicit in field names. Numbers are the shortest decimal that
represents the engine value exactly (`0.2`, not `0.20000000298023224`).
`object_id` is a zero-based index into `metadata.objects`, or `null` for a move
outside an object; it is unrelated to the opaque REST object IDs of the scene.
Each `metadata.objects` entry is an object instance as labelled in the G-code by
the engine, and its `name` is that label (for example `part_stl id:0 copy 0`).
`layer_id` is zero-based and indexes `layers`. As in the GUI preview, only
layers containing an extrusion are listed, each at the height (`z_mm`) of its
first extrusion; a move between layers (start G-code, layer change) belongs to
the next listed layer, or to the last one after the end. When no move extrudes,
`layers` is empty and `layer_id` is `null`. `statistics.estimated_time_s.stealth`
is `null` when the stealth time estimate is disabled. Move types use
the names `noop`, `retract`, `unretract`, `seam`, `tool_change`, `color_change`,
`pause_print`, `custom_gcode`, `travel`, `wipe`, and `extrude`. Roles use the
names `perimeter`, `external_perimeter`, `overhang_perimeter`,
`internal_infill`, `internal_bridge_infill`, `solid_infill`, `top_solid_infill`,
`ironing`, `bridge_infill`, `thin_wall`, `gap_fill`, `skirt`,
`support_material`, `support_material_interface`, `wipe_tower`, `milling`,
`custom`, and `travel`. `metadata.filename` is a file name, never a server path.

**Sparse moves.** To reduce the response size, moves are delta-encoded inside
a page:

- the first move of every page carries every field, so each page can be decoded
  on its own;
- every later move always carries `points`, and carries any other field only
  when its value differs from the same field of the previous move of the page;
  an omitted field keeps the previous move's value;
- `null` is a value: a field changing to `null` (for example `object_id` leaving
  an object) is sent explicitly as `null`, never omitted;
- equality is exact equality of the serialized JSON value.

The example below shows a first move, which is therefore complete.

Each `move` contains `points`, an ordered list of `[x,y,z]` coordinates in
millimeters describing its entire 3D path from start to end. Intermediate
points are included when the path is not a single straight segment.
`position_mm` is the final point and must equal the last element of `points`
when nonempty. Moves without a spatial path use `"points": []`.

Speed is provided by `feedrate_start_mm_s` and `feedrate_end_mm_s`, in millimeters
per second at the start and end of the move, respectively. When they differ,
the viewer can display a speed gradient along `points`: a higher end speed
indicates acceleration and a lower end speed deceleration. Equal values
represent constant speed for this representation.

In V1, the back end provides only one speed per segment. That speed is duplicated
in `feedrate_start_mm_s` and `feedrate_end_mm_s`; no additional acceleration
estimate is invented. When the back end provides multiple speeds, a movement
with multiple phases (acceleration, cruise, deceleration) will be split into
multiple moves to represent them. Several moves may therefore share a
`gcode_id`; pagination counts returned moves, not G-code lines.

`time_s` is the cumulative estimated time at the end of the move, in seconds
since the start of G-code, not the segment's duration. Every `points` path is
complete even when its move is the first on a page.

```json
{
  "schema_version": 1,
  "result_id": "result-opaque-id",
  "outdated": false,
  "metadata": {
    "filename": "output.gcode",
    "is_binary": false,
    "bed_shape_mm": [[0,0],[200,0],[200,200],[0,200]],
    "max_print_height_mm": 200,
    "z_offset_mm": 0,
    "extruder_count": 1,
    "extruders": [
      {"id":0,"extruder_color":"#ffffff","filament_color":"#ff0000","filament_diameter_mm":1.75,"filament_density_g_cm3":1.24}
    ],
    "objects": [{"id":0,"name":"part_stl id:0 copy 0"}],
    "statistics": {
      "estimated_time_s": {"normal":120,"stealth":150},
      "filament_volume_mm3_by_extruder": [1200]
    }
  },
  "layers": [{"id":0,"z_mm":0.2}],
  "moves": [
    {
      "gcode_id": 1,
      "type": "extrude",
      "role": "perimeter",
      "extruder_id": 0,
      "color_change_id": 0,
      "object_id": 0,
      "position_mm": [12.4,12.5,0.2],
      "extrusion_delta_mm": 0.05,
      "feedrate_start_mm_s": 40,
      "feedrate_end_mm_s": 40,
      "width_mm": 0.45,
      "height_mm": 0.2,
      "mm3_per_mm": 0.09,
      "fan_speed_percent": 100,
      "temperature_c": 210,
      "time_s": 12.05,
      "layer_id": 0,
      "internal_only": false,
      "points": [[12.3,12.5,0.2],[12.4,12.5,0.2]]
    }
  ],
  "pagination": {
    "offset": 0,
    "limit": 10000,
    "total_moves": 1,
    "next_offset": null
  }
}
```

`role` is a stable string identifying the extrusion role, or `null` for moves
without a role. `color_change_id` is the color-print index in use for the move (`0` before the first color change).
Arrays may be empty. `next_offset` is `null` when the final page is reached.

### 5.16. Scene editing (1.1)

All scene edits use `POST`, validate the body before checking whether slicing is
active, and publish the scene plus entity-ID changes in one revision-checked
commit. Effective geometry edits invalidate current G-code. An unchanged type or
name returns `200` without a commit. Object names affect slicing; part names do not.

- `set_part_type/{part_id}` accepts `{"type":"modifier"}`. Writable values are
  `part`, `negative_volume`, `modifier`, `support_blocker`, `support_enforcer`,
  `seam_position_center`, `seam_position_center_z`, `seam_position_inside`,
  `brim_patch`, `brim_negative`, `support_column`, `no_support_zone`, and `support_interface_column`.
  The last solid part cannot change type;
  cut and embossed-object restrictions apply. Response: `{"id":...,"type":...}`.
- `rename/{id}` accepts `{"name":"Bracket"}` for an object or part. A name is
  1–255 UTF-8 bytes without control characters, as a session name (§5.3.1;
  otherwise `400 invalid_request`), and contains none of `<>:/\|?*"`
  (`422 invalid_name`). Embossed parts and instances cannot be renamed.
  Response: `{"id":...,"name":...}`.
- `delete_entity/{id}` accepts an empty body or `{"invalidate_cut":true}`.
  Deleting the last solid part or last instance is refused. Cut objects and their
  solid/negative parts require explicit cut invalidation. Response lists every
  removed ID in `deleted`; a retained sole part keeps its REST ID.
- `add_part/{object_id}?type=...` uploads STL, OBJ, or 3MF. Loadable types are
  solid, negative, modifier, support blocker/enforcer, brim patch/negative, support column,
  no support zone, and support interface column.
  Response: `{"part_id":...,"type":...}`.
- `add_shape/{object_id}` requires `type` and `shape`; optional `size`, `position`,
  and reference `instance` control dimensions and platter placement. Shapes are
  selected by the following table. Response includes
  `part_id`, `type`, and `shape`.

  | `type` | Accepted `shape` values |
  |---|---|
  | `part`, `negative_volume`, `modifier`, `support_blocker`, `support_enforcer`, `seam_position_inside` | `box`, `cylinder`, `sphere`, `slab` |
  | `brim_patch`, `brim_negative` | `square`, `circle` |
  | `support_column`, `no_support_zone`, `support_interface_column` | `box`, `cylinder` |
  | `seam_position_center` | `small_sphere`, `sphere` |
  | `seam_position_center_z` | `small_cylinder` |

  Defaults are fixed because the REST server has no camera. `size`, when
  accepted, overrides the bounding-box dimensions in millimeters.

  | `shape` | Default size |
  |---|---|
  | `box` | 100 × 100 × 100 mm |
  | `cylinder` | diameter 50 mm × height 100 mm |
  | `sphere` | diameter 100 mm |
  | `slab` | 1.5 × instance width and depth, 0.5 × instance height; placed at the instance bottom |
  | `square` | 100 × 100 × 0.1 mm |
  | `circle` | diameter 10 mm × height 0.1 mm |
  | `small_sphere` | diameter 20 mm |
  | `small_cylinder` | diameter 20 mm × height 100 mm |
- `split/{id}` accepts `{"to":"objects"}` or `{"to":"parts"}`. Object splitting
  replaces the source with solid-shell objects; modifiers are discarded. Part
  splitting replaces the source ID with new IDs. Responses contain `removed` and
  `object_ids` or `part_ids`.
- `merge` accepts `{"ids":["object-1","object-2"],"name":"Merged"}`. IDs must
  identify distinct, non-cut, single-instance objects. It preserves part IDs and
  platter geometry while replacing source object and instance IDs.

| Condition | Response |
|---|---|
| Malformed JSON; missing, extra, or wrongly typed field; name of invalid length or with control characters | `400 invalid_json` / `400 invalid_request` |
| Unknown or disallowed type or target | `422 invalid_part_type` |
| Illegal filename characters, embossed part, or instance rename | `422 invalid_name` |
| Last solid part or last instance deletion | `422 invalid_delete` |
| Cut-dependent operation without permission | `422 cut_object` |
| Unknown/disallowed shape or invalid dimensions/position | `422 invalid_shape` |
| Unreadable or empty uploaded model | `422 invalid_model` |
| Unsplittable entity | `422 not_splittable` |
| Invalid merge selection | `422 invalid_merge` |
| Active slicing / concurrent commit | `409 session_busy` / `409 session_changed` |

### 5.17. Entity settings and layer ranges (1.1)

Entity settings are partial overrides stored on an object, a printable solid
part, a parameter modifier, or a layer-height range. Reads expose only values
explicitly stored on that entity; inherited values are not returned. A setting
definition's optional `scopes` lists which entity kinds may store it.

| Scope | Accepted settings |
|---|---|
| `object` | `PrintObjectConfig` and `PrintRegionConfig` settings, including FFF plugin options declared for `Object`, `Layer`, or `Region` containers, plus `extruder` |
| `part` | Region settings, including FFF plugin options declared for `Layer` or `Region`, plus `extruder`; support-category settings are excluded |
| `layer_range` | Region settings, including FFF plugin options declared for `Layer` or `Region`, plus required `layer_height` and `extruder`; support-category settings are excluded |

Definitions with category `none` are omitted, except for the special `extruder`
and `layer_height` settings. `extruder` is also listed in the `entity` catalog
category. Its integer value is `0` for the default tool or `1` through `N` for
the printer's numbered extruders. Entity-scoped vector values use the session's
selected printer to validate their extruder cardinality. Categories such as
`extruders` and `wipe` are not filtered on single-extruder printers.

An object, solid part (`type: "part"`), modifier (`type: "modifier"`), and
layer range can be queried with:

- `GET .../list_entity_settings/{entity_id}` returns
  `{"id":"entity-id","setting_ids":["extruder","fill_density"]}`. The
  setting IDs are sorted and include only explicit, supported overrides.
- `GET .../get_entity_settings/{entity_id}/{setting_id}` returns a typed value
  in the same `value`/`enabled`/`percent` format used by `/get_settings`, for
  example `{"id":"entity-id","setting_id":"fill_density","value":{"value":40}}`.
  A permitted but absent override returns `404 setting_not_set`; values are
  never inherited in this response.
- `POST .../set_entity_setting/{entity_id}/{setting_id}/{serialized_value}`
  accepts the same URL-encoded SuperSlicer serialization as `set_setting`, with
  no body. It returns the accepted `serialized_value`.
- `POST .../set_entity_settings/{entity_id}` accepts a nonempty JSON object.
  Each setting value uses the typed value-object contract from §5.2; `null`
  removes that entity's override. The response includes typed values actually
  applied in `updated` and removed keys in sorted `removed`.

The two entity-setting GET routes may return `409 session_changed` if the
session revision changes while the scene snapshot and entity ID are being read;
clients can retry the read.

Validate the complete update before assigning any override. A key outside the
entity's scope, invalid type/value/range/cardinality, read-only setting, or
extruder outside `0..N` returns `422 invalid_setting` without a partial change.
On a layer range, `layer_height` and `extruder` cannot be removed, and the
stored layer height must remain within the selected extruder's minimum/maximum
nozzle limits. Cross-setting consistency is checked by the slicer when slicing,
as in the GUI. A no-op update returns `200` without increasing the session
revision; a real change invalidates current G-code and publishes one new revision.

Layer ranges are entities whose ID is keyed by the owning object ID and exact
`[min,max]` bounds. Their IDs appear in `list_objects` and remain stable while
they exist; changing bounds moves the ID to the new key. Ranges are persisted in
3MF projects. Each range has required `layer_height` and `extruder` overrides.

- `POST .../add_layer_range/{object_id}` accepts
  `{"min":5,"max":8,"layer_height":0.3,"extruder":0}`; `extruder` is
  optional and defaults to `0`. It returns the new `id`, `min`, and `max`.
- `POST .../set_layer_range/{range_id}` accepts only numeric `min` and `max`.
  Bounds must be finite, satisfy `0 <= min < max`, and not overlap another
  range on the same object. Touching bounds are allowed. The range's settings
  follow the new bounds and its ID is retained. Repeating the current bounds is
  a no-op.
- `POST .../delete_entity/{range_id}` removes a range and returns its ID in
  `deleted`; `invalidate_cut` has no effect on a range.

Invalid bounds, overlap, or a layer height outside nozzle limits return
`422 invalid_layer_range`. The `add_layer_range` target must be an object and
`set_layer_range` target must be a layer-range ID. Instances and non-printable
volume types do not accept entity settings (`422 invalid_entity_settings`).
Layer ranges cannot be transformed or renamed, and other scene-edit routes
return their corresponding `422` target error. As with other reads, GET routes
reject a request body with `400 invalid_request`; valid writes during slicing
return `409 session_busy`, and a concurrent revision change returns
`409 session_changed`.

### 5.18. Arrange and orient (1.1)

Both placement routes accept an empty body or a JSON object containing only the
listed fields. Distances are in millimeters. `ids`, when present, is an array of
object or instance IDs; an object selects all its instances. Missing or empty
`ids` selects the whole platter for arrangement and every printable instance
for orientation. Explicitly selected instances are used even if marked
non-printable. Unknown IDs return `404 entity_not_found`; part and layer-range
IDs return `422 invalid_arrange` or `422 invalid_orient`.

- `POST .../arrange` accepts optional `distance`, `distance_from_bed`,
  `rotations`, `alignment`, `geometry`, and `strategy`. The distance defaults to
  `duplicate_distance` plus the preferred object spacing, or 6 mm
  when `duplicate_distance` is zero. In By object, the preferred spacing includes
  nozzle clearance; it is not a mandatory minimum. An explicit `distance` may be
  smaller and must be finite and non-negative.
  `distance_from_bed` must be finite and non-negative. Both distances must also
  fit safely in the engine's scaled-coordinate representation; a finite value
  outside that representable range returns `422 invalid_arrange`.
  `alignment` is one of `center`, `rear_left`, `front_left`, `front_right`,
  `rear_right`, or `random`; `geometry` is `convex`, `balanced`, or `advanced`;
  `strategy` is `auto` or `pull_to_center`. Alignment affects XL printers only.
  Rotations default to `false`.
- Brim and skirt widths from the session configuration are kept clear of the
  bed edge: each placed item stays at least the larger of `distance_from_bed`
  and the skirt plus brim extent from the edge. When the skirt is measured from
  the brim (`skirt_distance_from_brim`, the default), the engine counts the
  brim twice in that margin, even without a skirt, as in the GUI. `distance` is
  the gap between object outlines, as in the GUI; neighbouring brims may merge.
  A wipe tower is not taken into account. Items that do not fit the physical
  bed may be placed on virtual beds; virtual beds are outside the printable
  area. An item too large for a bed is left in place with a `null` bed index.
- The response is `{"instances":[{"id":"instance-id","bed":0}]}`. Bed 0 is
  the physical bed, positive values identify virtual beds, and `null` means the
  instance could not be placed. Instance matrices are read back through
  `list_objects`.
- `POST .../orient` accepts optional `min_area` (default `true`) and
  `overhang_angle` (default 60; finite and between 45 and 90 degrees). It
  minimizes support area when `min_area` is true and volume otherwise. It
  optimizes each selected instance from its own transformed mesh and lays each
  oriented instance on the bed (its own lowest point at Z = 0); other instances
  of the same object are not moved. It does not arrange the instances afterwards.
  The response is `{"instances":[{"id":"instance-id","matrix":[16 numbers]}]}`
  with row-major matrices.
- A real placement change commits one new session revision and invalidates
  current G-code. If every transform is unchanged, the route still returns 200
  but does not commit. Both routes return `409 session_busy` during slicing and
  `409 session_changed` if another mutation wins while the job is running.
  Malformed or unknown fields return `400`; unsupported values return
  `422 invalid_arrange` or `422 invalid_orient`; an uninitialized engine returns
  `503 engine_not_initialized`.

### 5.19. `POST .../cut/{id}` — plane cut (1.1)

This route cuts either an object or one printable solid part. It uses the same
plane orientation as the GUI: the side in the direction of `normal` is the
`upper` side. The body is a closed JSON object:

```json
{
  "plane": {"point": [100, 100, 10], "normal": [0, 0, 1]},
  "space": "platter",
  "instance": "instance-id",
  "keep_upper": true,
  "keep_lower": true,
  "keep_as_parts": false,
  "flip_upper": false,
  "flip_lower": false,
  "place_on_cut_upper": false,
  "place_on_cut_lower": false,
  "invalidate_cut": false,
  "arrange": false
}
```

`plane` is required. `point` and `normal` contain three finite millimeter
coordinates; `normal` must be non-zero and is normalized. `keep_upper` and
`keep_lower` default to `true`, and at least one must remain true. `space`
defaults to `platter` and accepts:

| `space` | Coordinates used for `point` and `normal` |
|---|---|
| `platter` | Global platter coordinates |
| `object` | The placed reference instance's object coordinates |
| `volume` | The target part's local coordinates; for an object target, the reference instance's coordinates |

For a reference instance transform `I` and a target part transform `V`, the
plane is converted to platter coordinates with `M = identity` for `platter`,
`M = I` for `object`, and `M = I × V` for `volume` on a part (`M = I` for
`volume` on an object):

```
point_platter  = M × point
normal_platter = normalize((M.linear()⁻¹)ᵀ × normal)
```

The linear transform must be invertible. The resulting engine cut frame is a
translation by `point_platter - offset(I)` followed by the smallest rotation
that maps local +Z to `normal_platter`. This inverse-transpose conversion keeps
the plane correct under non-uniform scaling and mirroring. `instance` is
optional and defaults to the object's first instance; when supplied it must be
an instance of the target object (for a part target, of the part's object). An unknown ID returns
`404 entity_not_found`, while an instance of another object returns
`422 invalid_cut`.

Before changing the scene, the route checks every vertex of each targeted solid
part. At least one vertex must be more than `1e-6` above and one more than
`1e-6` below the plane in the cut frame. A plane that does not cross the
geometry returns `422 invalid_cut` and leaves the session unchanged.

For an object target, `keep_as_parts` keeps both halves in one object as two
parts named with `_A` (upper) and `_B` (lower). It requires both keep flags and
cannot be combined with `flip_*` or `place_on_cut_*`. Without it, the route
creates one object per retained side, in upper-then-lower response order.
`flip_upper`, `flip_lower`, `place_on_cut_upper`, and `place_on_cut_lower` have
the same meaning as the GUI cut controls and are refused with `keep_as_parts`.
The source object and all of its object, part, instance, and layer-range IDs are
removed; generated entities receive new IDs. The cut group's identity and
checksum follow the GUI rules, and other objects in that group are synchronized.
Each generated object is sorted and placed on the bed as in the GUI operation.

For a part target, the entity must have type `part`. Only `keep_upper` and
`keep_lower` are meaningful; `keep_as_parts`, all `flip_*` and
`place_on_cut_*` options, and `arrange` return `422 invalid_cut`. Only the
selected part is replaced by new part IDs; other parts, their transforms,
instances, modifiers, settings, height ranges, and object identity remain
unchanged. An object cut clears painting data on every part of the source
object; a part cut clears painting data only on the selected part.
Keeping only one side leaves the parent object at its existing position; a
client can reposition it with `transform` and `ensure_on_bed: true`.
Cutting a part of an object already in a cut group requires
`invalidate_cut: true`; otherwise the route returns `422 cut_object`. With
invalidation, the group metadata is invalidated before the replacement. The
`invalidate_cut` field is accepted for any target but has no other effect.

The response identifies removed entities and the newly created sides:

```json
{
  "removed": ["old-object-id", "old-part-id", "old-instance-id"],
  "objects": [
    {"id": "upper-object-id", "side": "upper"},
    {"id": "lower-object-id", "side": "lower"}
  ]
}
```

An object kept as parts uses `"side":"parts"` and one `objects` entry. A part
cut returns `parts` instead of `objects`, with one entry per retained side.
`arrange: true` is valid only for an object cut. It cuts first and then uses
the default `/arrange` settings on the new object instances. The complete
operation runs on the import pool and the response additionally contains
`"beds":{"instance-id":0|k|null}`. Without arrangement, the operation is
synchronous and invokes the scene-edit observer immediately before the commit.
Both paths reject stale concurrent commits with `409 session_changed` and
active slicing with `409 session_busy`; a session deleted while an arranged
operation is running returns `404 session_not_found`.

Successful cuts invalidate current G-code, mark the viewer outdated, and leave
the session ready. Other failures use the common error format: malformed JSON,
unknown fields, missing fields, and wrong types return `400 invalid_request`;
unknown sessions or entities return `404`; unsupported target/options, a
non-solid target, a degenerate normal, a non-crossing plane, or an invalid
combination return `422 invalid_cut`; and a restricted cut-group part returns
`422 cut_object`.

### 5.20. Facet painting (1.1)

Painting stores per-triangle states on solid model parts. The global
`GET /list_paint_kinds` route returns the fixed `support` and `mmu` channels,
followed by the generic facet-annotation definitions registered by the engine.
Generic definitions include `builtin:seam` and active plugin channels such as
`perimeter.post_process.fuzzy_skin.painting`. Each entry has an `id`, `label`,
and `states`; support and generic channels use `enforce` and `block`, while MMU
uses `extruder` and reports `max_extruder: 15`.

`POST /session/{session_id}/paint/{id}` accepts a closed JSON body. The target
is either a solid `part` ID or an object ID; an object applies each resolved
point to its nearest solid model part. The optional `instance` selects the
reference instance used to place request coordinates and defaults to the first
instance. It must belong to the target object (for a part target, to the part's
object). A target part must be of type `part`; modifier and support volumes
cannot be painted.

```json
{
  "kind": "builtin:seam",
  "state": "enforce",
  "instance": "instance-id",
  "space": "platter",
  "tool": "circle",
  "radius": 2.0,
  "view_direction": [0, 0, -1],
  "strokes": [
    [[10, 10, 20], [12, 10, 20]],
    [{"origin": [10, 10, 100], "direction": [0, 0, -1]}]
  ],
  "split_triangles": true,
  "overhangs_only": 0,
  "smart_fill_angle": 30,
  "angle": 0,
  "tolerance": 0.05
}
```

`space` is `platter` (the default), `object`, or `volume`. Positions are
transformed with the full reference/volume matrix for the selected space;
directions use its linear part and are normalized, not transformed as surface
normals. A stroke is a non-empty array of either surface points `[x,y,z]` or
ray objects with exactly `origin` and `direction`; all points in one stroke
must use the same form. Surface points are accepted when their closest target
surface is within `tolerance` millimeters. A ray uses the first target hit.
There may be at most 10,000 points in one request.

`state` is `enforce`, `block`, or `erase` for support and generic channels.
For MMU it is `{"extruder": k}`, where `0` erases and `1..min(N,15)` selects
an extruder from the session's configured nozzle count. MMU painting requires
at least two extruders. `radius` is in the range `0.05..2000` millimeters.
`smart_fill_angle`, `overhangs_only`, and `angle` are degrees in `0..90`.
`tolerance` must be finite and positive. `view_direction` overrides the
surface-facing direction for circle brushes and replaces the ray direction for
ray points; without it, a ray's own direction is used. The sphere ignores that
option.

The REST extension accepts every tool for every channel: `circle`, `sphere`,
`triangle`, `smart_fill`, `bucket_fill`, and `by_angle`. Circle/sphere strokes
paint one cursor or capsules between consecutive points on the same part.
Triangle and bucket fill operate on each point as a seed; smart fill follows
the GUI's surface-angle and overhang rules. `by_angle` has no strokes and
paints every original face whose normal is within `angle` degrees of the
downward vertical (platter `-Z`). There is no clipping plane in the REST
request.

Painting is deferred to the import worker pool because target AABB trees may be
large. The request is fully resolved before any selector is changed, and a
point error leaves every channel unchanged. On success the response is
`{"changed":["part-id", ...]}` in target-part order. An unchanged request
returns an empty `changed` array and does not advance the session revision.
Successful changes invalidate current G-code, retain an outdated viewer, and
return the session to `ready`. A stale revision returns `409 session_changed`;
an active slice returns `409 session_busy`.

Point resolution failures use `422 invalid_paint` with index details:

```json
{
  "error": {
    "code": "invalid_paint",
    "message": "Paint point 0/1 could not be resolved: off_surface",
    "request_id": "req_...",
    "details": [{"stroke": 0, "point": 1, "reason": "off_surface"}]
  }
}
```

`POST /session/{session_id}/clear_paint/{id}` accepts only
`{"kind":"support"}`, `{"kind":"mmu"}`, a generic channel ID, or
`{"kind":"all"}`. It is synchronous and targets a solid part or all solid
parts of an object. It invokes the scene-edit observer immediately before a
real commit, returns the same `changed` response, and does not commit when no
channel was present. `all` also removes stored generic annotation keys that
are not currently active plugins.

`GET /session/{session_id}/get_paint/{part_id}?kind=...` reads one immutable
session snapshot and is allowed during slicing. The required `kind` and
optional `instance` and `space` query parameters are strict; unknown or
repeated parameters return `400 invalid_request`. The target must be a solid
part, and the body must be empty. `platter` is the platter space of the
reference instance, `object` is object space (`V` applied to a vertex), and
`volume` is the part's local space; `platter` is the default. The response
contains strict triangulated painted facets in the requested output space:

```json
{
  "kind": "support",
  "states": [
    {"state":"enforce", "vertices":[[x,y,z]], "triangles":[[0,1,2]]}
  ]
}
```

MMU states use `{"extruder": k}`. A channel with no painted facets returns
`"states": []`. The painting data is part of the normal 3MF project snapshot,
so `save_project` and `load_project` preserve support, MMU, seam, and active
plugin annotation channels.

### 5.21. Custom G-code per height (1.1)

The scene stores one list of height-based actions (`Model::custom_gcode_per_print_z`),
the data behind the GUI layer slider. It is saved in and restored from the 3MF
project and is applied when slicing. `clear_platter` erases it, as in the GUI;
`add_model` does not touch it.

`GET /session/{session_id}/get_custom_gcodes` takes no body or query, reads one
immutable session snapshot, and is allowed during slicing:

```json
{
  "mode": "single_extruder",
  "current_mode": "single_extruder",
  "items": [
    {"z": 2.4, "type": "color_change", "extruder": 1, "color": "#FF0000"},
    {"z": 5.0, "type": "pause", "message": "Insert nuts"},
    {"z": 6.0, "type": "template"},
    {"z": 7.2, "type": "custom", "gcode": "M117 Hello"},
    {"z": 9.0, "type": "tool_change", "extruder": 2, "color": "#00FF00"}
  ],
  "warnings": []
}
```

`POST /session/{session_id}/set_custom_gcodes` takes a closed
`{"items": [...]}` body holding the complete list: it replaces the stored list
atomically, and `[]` empties it. The response has the shape of
`get_custom_gcodes`. Each item is a closed object whose fields depend on `type`
(any other field is `400 invalid_request`):

| `type` | Fields |
|---|---|
| `color_change` | `z`, optional `extruder` (defaults to the unique extruder, or 1), optional `color` |
| `tool_change` | `z`, required `extruder`, optional `color` |
| `pause` | `z`, non-empty `message` |
| `template` | `z` |
| `custom` | `z`, non-empty `gcode` |

`z` is a number in millimeters (the print height of the layer at which the
action is applied). `extruder` is a 1-based integer in `[1, N]`, `N` being the
session extruder count. `color` is `#RRGGBB` (any case, stored upper case); when
absent it is the color of the target extruder, as in the GUI: `extruder_colour`
of that extruder when not empty, otherwise `filament_colour`. If neither is a valid
color, the item is stored without color and `color` is omitted from the JSON. An
unknown `type` is `422`, reported after the 400 checks of `type` and `z`, which
every item needs.

**Modes.** The mode is computed at the time of the call with the slicing engine's
own rule (`ToolOrdering::assign_custom_gcodes`) and stored with the list: a `Print`
is applied on a copy of the scene and the session configuration, without slicing.
It is `single_extruder` when the session has one extruder or the scene is empty;
`multi_as_single` when exactly one extruder prints the whole model; otherwise
`multi_extruder`. The engine counts every print region of each object, including
its fallback region (which keeps the default extruder), plus modifiers, height
ranges and MMU painting; so `multi_as_single` is reached only when every printing
region uses the same extruder, in practice the default one, and a part or object
assigned to another extruder gives `multi_extruder`. `tool_change` is accepted only in
`multi_as_single`; in `single_extruder`, `extruder` must be 1. Items without a
color or extruder choice use the unique extruder of `multi_as_single` (1
otherwise); pauses, templates and custom items carry it for information only.
`current_mode` is the mode recomputed from the current scene and configuration on
every read. `mode` is `undefined` for the list of a fresh session and for an empty
list stored by an older project.

**Validation** (`422 invalid_custom_gcode`, with
`details: [{"item": i, "reason": r}]` for an item error). `reason` is
`unknown_type`, `invalid_z` (not finite, not above 0, or rounding to 0 on the layer
grid),
`empty_scene`, `above_objects` (`z` above the top of the tallest object once
both are rounded to the engine's layer grid of 0.0001 mm, so a `z` returned by
the API is always accepted again on the same scene), `mode_tool_change`,
`extruder_range`, `invalid_color`, `empty_message`, `empty_gcode`, or
`duplicate_z` (two items fall at the same rounded `z`; `item` is the later
one). `z` is bounded by the scene height and must stay below 10000 mm once rounded; a scene
top of 10000 mm or more refuses every item. By object accepts height G-codes too:
their heights split the plate's work into bands, and each eligible event is emitted once
at the first printing-plan layer at or above its height. An empty list is always
accepted. The stored list is sorted by `z`.

**Warnings.** Accepted values that need attention are reported by both routes in
`warnings`, each `{"item": i, "reason": r}` or a bare `{"reason": r}`:
`empty_printer_gcode` (an advisory check: for `color_change` and `pause`, the
`color_change_gcode` or `pause_print_gcode` option when set, otherwise the
legacy `gcode_flavor` default; for `template`, the raw `template_custom_gcode`;
an empty value is accepted). This warning still uses the legacy writer's
defaults, not `gcode_firmware_plugin`, so it can disagree with the selected
firmware's actual fallback. It does not suppress an item or change the output.
Other warnings are
`above_objects` (the scene changed and now ends below the item), and
`mode_mismatch` (the list holds a color or tool change
and its stored mode and the current mode differ on whether the print is
`multi_extruder`: this is exactly when the engine ignores the color and tool
changes of the list).

`set_custom_gcodes` is synchronous. A list equal to the stored one (an
empty list is equal to any empty list, whatever its mode) is not a new revision. A real
change invalidates current G-code, retains an outdated viewer, and returns the
session to `ready`; it invokes the scene-edit observer immediately before the commit.
An active slice returns `409 session_busy` and a concurrent session change
`409 session_changed`.

The engine applies an item to the first layer whose `print_z` is at or above its
`z`. As in the GUI's shared engine, **all color changes, pauses, templates and
custom items mapped to a layer are retained**, in increasing source `z` order.
A `tool_change` is processed separately and applies in addition to these items.
Layers are only known when slicing, so the API does not report this mapping.
A non-empty `pause_print_gcode` supplies the whole pause script; use
`M117 {pause_message}` followed by a pause command to display the supplied message.
With an empty script, the selected `gcode_firmware_plugin` supplies its fallback.
For `gcode.firmware.marlin2`, that is `M0` followed by the optional pause message;
the color-change fallback is `M600`.

### 5.22. Variable layer height (1.1)

Each object stores a flat sequence of `[z, height]` pairs in millimeters,
relative to the bed-level base of the printed part. Heights are piecewise linear
between points; a step is represented by two points at the same Z. Z values must
be non-decreasing, and no Z may appear more than twice. The first Z is that base
(`0`), and the last Z is the object top. The first object layer remains fixed at
the effective first layer height (`first_layer_height` resolved and rounded to
`z_step`); a point is exempt from printer height bounds only when a following
point exists at or below that first-layer Z. An anchor whose segment crosses the
first-layer boundary is subject to the bounds. `save_project` and `load_project`
round-trip profiles written by SuperSlicer, including two-point profiles. A
Bambu-format 3MF is loaded by the fallback importer, which still drops two-point
profiles; the object then stores no profile (`profile` is empty and `applied` is
false) and layers fall back to the layer-range profile. 3MF stores profile
values with 6 decimals, so a reloaded top stays within the 1e-3 mm tolerance. A
non-empty profile takes precedence over the `layer_height` values in the
object's height ranges; the other range settings remain in effect. Resetting the
profile returns layer-height selection to those ranges.

`GET /session/{session_id}/get_layer_height_profile/{object_id}` takes no body
or query and reads one scene snapshot, including while slicing:

```json
{
  "profile": [[0.0, 0.2], [5.0, 0.2], [5.0, 0.1], [20.0, 0.1]],
  "applied": true,
  "valid": true,
  "object_height": 20.0,
  "min_layer_height": 0.07,
  "max_layer_height": 0.3,
  "layers": [0.2, 0.4, 0.6]
}
```

`profile` is the stored sequence. REST validates the stored profile when it is
non-empty; otherwise it validates the effective profile from the object's layer
ranges. The profile top must be within 1e-3 mm (the engine's 10 × EPSILON) of the
object top, the engine's retention tolerance. Explicit input accepts a final Z
within `1e-3` mm and normalizes it to the exact object top before the other
profile checks. REST also checks finite values, a zero first Z, ordering and
repeated-Z steps, and printer height bounds; the engine's retention check alone
does not enforce those rules. `valid` reports these REST checks, and valid responses omit `errors`.
`applied` is true exactly when the engine's effective profile equals the stored
non-empty profile, so it can be true even when REST reports `valid: false` (for
example, a stored point outside printer bounds). If a stored profile fails REST
validation, GET still returns HTTP 200 with `valid: false`, an `errors` array, and
no `layers`; the route never asks the layer generator to consume invalid data.
A profile that is empty uses the layer-range profile instead. `object_height` and
the minimum/maximum bounds are computed from the object's slicing parameters. An
object thinner than its first layer is reported as `valid: false` by GET, because
the fixed first layer ends above the object top. `layers` contains the top Z of
each effective object layer, relative to the bed-level base; it reflects the
fixed first layer, the applicable profile, or the layer ranges.

`POST /session/{session_id}/set_layer_height_profile/{object_id}` takes a closed
JSON object with exactly one operation:

| Body | Effect |
|---|---|
| `{"profile": [[z, height], ...]}` | Store an explicit profile |
| `{"adaptive": {"quality": 0.5, "min_height": 0.07, "max_height": 0.3}}` | Generate an adaptive profile; the optional bounds default to the object's printer bounds |
| `{"smooth": {"radius": 5, "keep_min": false}}` | Smooth the effective profile; `keep_min` defaults to `false` |
| `{"reset": true}` | Clear the stored profile |

Explicit profiles require at least two points, finite values, a first `z` of
zero, non-decreasing Z values (no Z may occur more than twice), and a last `z`
within `1e-3` mm of the object height. An accepted final Z is normalized to the
exact object top before ordering and repeated-Z checks. Heights must be within
printer bounds with a `1e-6` mm tolerance, except when a point has a following
point at or below the effective first layer height. The latter point controls
layers above the boundary and remains subject to printer bounds; an anchor whose
segment crosses the boundary is not exempt.

`quality` is in `[0,1]`. Optional adaptive bounds must be ordered and within
printer limits; each is rounded to `z_step`, then clamped to the printer limits
before the engine generates the profile. The returned profile
shows the normalized result. Requested bounds apply to adaptive points except
the fixed-first-layer anchors and final gap point; the engine clamps the final
gap point to printer layer-height limits. Smoothing `radius` is an integer in
`[1,10]`. An object without a solid part cannot be edited.

Point failures return `422 invalid_layer_height_profile` with
`details: [{"point": i, "reason": r}]`; reasons include `first_z`, `z_order`,
`height_range`, and `last_z`. Global profile failures carry only a reason, with
no point index. The stable global reasons are `invalid_length` (a stored profile
with an odd number of values) and `too_few_points`; other global failures come
from profiles derived from layer ranges and from profiles generated by
`adaptive` or `smooth`. An explicit profile with fewer than two points returns
the same status and code with reason `too_few_points` in the message and no
`details`, because no point index identifies that failure. Smoothing returns 422
whenever
`get_layer_height_profile` reports `valid: false`, with the same error details.

Malformed JSON, unknown or missing fields, wrong types, and requests with zero
or multiple operations return `400`. Unknown objects return `404`; a part or
instance target returns `422`. Setting a profile is synchronous: an unchanged
flat profile does not create a new revision. A changed profile is committed on a
private scene copy, invalidates current G-code, marks the viewer outdated, and
returns the session to `ready`. The scene-edit observer runs immediately before
commit. Active slicing returns `409 session_busy`; a competing scene change
returns `409 session_changed`.

Changing object height does not clear the stored profile. If the stored profile
ends more than 1e-3 mm (the engine's 10 × EPSILON) from the current object top,
GET keeps `profile` visible and reports `valid: false` with the endpoint error
and no `layers`; `reset` or a new valid profile can repair it. Layer-height
slicing parameters use the first instance (see Known limitations). The REST API
does not expose the layer-height brush; clients can send an explicit profile
instead. SLA layer-height editing is not exposed.

## 6. Security and remote access

Remote access is a functional requirement, not an anonymous deployment mode.
The specification therefore requires:

- individual user accounts; `POST /login` receives
  `{"username":"...","password":"..."}` and returns a token, for example
  `{"access_token":"...","token_type":"Bearer","expires_in":86400}`;
- only salted Argon2id hashes are retained; the submitted password is used only
  to verify the hash during the request, then forgotten, and is never logged;
- `/login` is limited to five requests per minute per username;
  an excess request returns `429 Too Many Requests`;
- Bearer tokens expire after 24 hours by default; this duration is configurable
  and `expires_in` is expressed in seconds;
- every route except `/login` requires `Authorization: Bearer <access_token>`;
- a missing, invalid, expired, or revoked token returns `401`; an unknown
  or foreign-owned session returns `404` after authentication;
- `POST /logout` revokes the presented token and returns `204 No Content`;
- only its owning account may access a session; `session_id` does not replace
  authentication;
- remote exposure through a VPN or reverse proxy with encrypted transport;
- the SuperSlicer process listens on a local/private interface and is not
  directly exposed to the Internet;
- no authentication data in logs;
- no post-processing script is executed in REST mode, regardless of provenance
  (installed profile, API setting, or 3MF project). `post_process` may be retained
  as configuration but remains inert: the REST pipeline never executes these scripts;
- validation of received files and configurable size/storage limits;
  the default limit is 1 GiB per uploaded file and may be changed in the server
  configuration.

Accounts are created by the administrator outside the API. The shipped local
Argon2id tool (§9 and [deployment.md](deployment.md)) emits an accounts document
including the salt and verification parameters. No plaintext password is
persisted or logged.

## 7. Acceptance criteria

V1 is acceptable if:

1. The server starts in its own process from the application executable's REST
   mode (§9) and works without launching or building the GUI.
2. `/list_settings` returns settings grouped by category; `/create_session`
   returns a `session_id` usable for session routes.
3. A session can list and select profiles, read one or more selected settings,
   and modify one or more settings. An empty extruder catalog does not prevent
   slicing. JSON values follow the `value`/`enabled`/`percent` object contracts,
   including parallel arrays and retention of disabled values. Loading a profile
   clears the relevant local modifications; the target rules in §5.5.1 are
    verified by the profile and settings integration tests below.
4. A session can add a model, load a 3MF project, and return the object/part/instance
   hierarchy with unique identifiers.
5. `transform/{id}` accepts an object, instance, or volume ID and preserves the
   GUI's global, instance, and volume rotation/translation spaces. New instances
   are superimposed and may subsequently be moved individually without changing
   retained IDs.
6. `slice` starts an asynchronous operation; `state` exposes `etat` and the
   `PrintStatus` messages/percentages, and `cancel_slicing` can interrupt active slicing.
7. The client can clear the platter and download the saved 3MF project, current
   raw G-code, and viewer JSON. An invalidating mutation blocks raw G-code but
   retains the viewer with `outdated: true`. `result_id` prevents mixing pages
   from two results. In V1 the engine-provided speed is duplicated in both fields.
8. Deleting a session makes its ID unusable and destroys its associated state.
9. Unauthenticated remote connections are refused and remote transport is
   encrypted. Authentication errors return `401`; accessing another account's
   session returns `404`, as for an unknown session.
10. No post-processing script is executed in REST mode, including scripts from
    imported projects. The local Argon2id tool is created and its output actually
    authenticates provisioned accounts.
11. A session occupied by slicing or export does not expire; the inactivity
    period restarts after success, failure, or effective stop. No crash recovery
    is required in V1.

### Acceptance evidence

The following are exact `TEST_CASE` names in the named executables. Tests use
real loopback HTTP where appropriate; TLS termination remains an operator and
platform validation responsibility, not an in-process TLS implementation.

| Criterion | Executable and test |
|---|---|
| 1 | `rest_launch_tests`: `REST launch help options and resource-default precedence`; installed launcher checks described in deployment.md |
| 2 | `rest_server_tests`: `REST server creates authenticated sessions through the global routes`; `rest_engine_tests`: `settings catalog exposes finalized definitions` |
| 3 | `rest_profiles_tests`: `REST profile IDs are opaque and selections are isolated`, `REST vendor bundles ignored by the GUI warn without preventing startup`, `Profile compatibility reads edited session values but overrides use catalog presets`; `rest_settings_tests`: `REST sessions read and update typed settings atomically` |
| 4 | `rest_models_tests`: `REST add_model imports STL OBJ and 3MF`, `REST list_objects exposes stable IDs across uploads and project replacement` |
| 5 | `rest_models_tests`: `REST transforms instances and objects atomically with explicit bases and bed placement`, `REST volume transforms use reference axes and retain shared-instance geometry`, `REST instance counts duplicate the current original and preserve retained IDs` |
| 6 | `rest_slice_tests`: `REST real slice publishes readable sanitized progress outside the session lock`, `REST cancel_slicing stops a running engine and allows a new slice` |
| 7 | `rest_e2e_tests`: `E1 complete public REST workflow`; `rest_slice_tests`: `REST viewer keeps the last result across invalidations and refuses replaced IDs`, `REST open G-code transfer retains its file through deletion replacement and disconnect` |
| 8 | `rest_slice_tests`: `REST delete removes only the owned session and its unreferenced result` |
| 9 | `rest_e2e_tests`: `E2 accounts authentication and ownership`; actual encrypted proxy checks: platform-validation.md |
| 10 | `rest_e2e_tests`: `E4 post processing is inert from all configuration sources`; `rest_launch_tests`: `REST launch account writer produces a document accepted by HTTP` |
| 11 | `rest_e2e_tests`: `E5 expiration shutdown and restart`; `rest_slice_tests`: `REST slice keeps registry work busy and restarts TTL at completion` |
| 1.1 scene editing | `rest_models_tests`: `REST list_objects reports part types paintings and cut objects`, `REST set_part_type changes types atomically and keeps unchanged types uncommitted`, `REST rename validates names and invalidates G-code only for objects`, `REST delete_entity removes entities and keeps the retained part ID`, `REST add_part checks the target before upload and imports a part`, `REST add_shape creates default and sized shapes`, `REST split and merge replace scene entities in one commit`, `REST scene edits are refused while slicing and on concurrent changes`, `REST negative volumes and support enforcers change the sliced result` |
| 1.1 entity settings and ranges | `rest_models_tests`: `Entity setting scopes follow the GUI lists and plugin containers`, `Entity settings write atomically and remove overrides`, `Layer ranges validate bounds overlaps and nozzle height limits`, `REST list_settings publishes entity setting scopes`, `REST entity settings read and write objects parts and layer ranges`, `REST layer ranges are entities with IDs that follow their bounds`, `REST entity setting edits are refused while slicing and on concurrent changes`, `REST entity settings change the sliced result`, `REST entity settings and layer ranges survive save_project and load_project` |
| 1.1 arrangement and orientation | `rest_models_tests`: `Arrange defaults follow the GUI distance rules`, `Arrange separates instances and keeps unselected ones fixed`, `Arrange reports virtual beds and unplaced instances`, `Orient rotates each instance from its own mesh`; `rest_placement_tests`: `REST arrange validates parameters and targets`, `REST arrange moves instances onto the bed and reports their beds`, `REST orient validates parameters and rotates instances onto the bed`, `REST arrange and orient are refused while slicing and on concurrent changes` |
| 1.1 plane cut | `rest_models_tests`: `Part cuts keep the cut geometry in place for every instance`, `Object cuts split the geometry as the GUI does`, `Cut planes are equivalent in platter object and volume spaces`, `Part cuts replace only the cut part`, `Cut validation rejects degenerate requests`, `REST cut validates the request and the target`, `REST cut validates reference instances`, `REST cut enforces cut-group invalidation`, `REST cut replaces objects and parts with new IDs`, `REST cut can arrange the new objects`, visible `REST cut results slice`, hidden and not executed `REST cut results with two objects slice` (`[.rest_multi_object_engine]`), `REST cut is refused while slicing and on concurrent changes` |
| 1.1 facet painting | `rest_models_tests`: `Brush strokes paint the expected area`, `Fill tools follow the GUI semantics`, `Points rays and spaces select the same paint`, `Object targets paint the nearest part`, `MMU and generic channels store their states`, `Paint validation leaves the scene unchanged`, `REST paint kinds are listed`, `REST paint validates the request and the target`, `REST paint updates parts and reports changes`, `REST get_paint returns painted triangles`, `REST paint persists in projects`, `REST paint is refused while slicing and on concurrent changes`, `REST painted parts slice`; `REST support painting reaches the support engine` is hidden and not executed (`[.rest_support_engine]`) |
| 1.1 custom G-code per height | `rest_models_tests`: `Custom G-code mode follows the engine rule`, `Custom G-code entries are validated and sorted`, `Custom G-code JSON reports modes and warnings`, `REST custom G-codes validate requests`, `REST custom G-codes replace the whole list`, `REST custom G-codes persist in projects`, `REST custom G-codes are refused while slicing and on concurrent changes`, `REST custom G-codes reach the G-code`, `REST custom G-code tool changes reach the G-code` |
| 1.1 variable layer height | `rest_models_tests`: `Explicit layer height profiles are validated`, `Adaptive and smoothed profiles follow the GUI tools`, `Stale layer height profiles are reported`, `REST layer height profiles validate requests`, `REST layer height profiles replace and reset`, `REST layer height profiles persist in projects`, `REST layer height profiles persist non-round object heights`, `REST range profiles accept near-top endpoints and omit point indices`, `REST adaptive bounds constrain profiles within printer limits`, `REST adaptive profiles round-trip a fixed first layer above the maximum`, `REST invalid stored layer height profiles are reported safely`, `REST smoothing rejects stored profiles ignored by the engine`, `REST layer height profiles are refused while slicing and on concurrent changes`, `REST layer height profiles reach the G-code` |

## 8. Decisions

1. **Accounts:** administrator-managed `paths.accounts`, with a `users` object
   keyed by username, for example `{"users":{"Alice":{"password_hash":"<encoded Argon2id hash>"}}}`
   (§6 and deployment.md). No API account administration.
2. **Headless build:** resolved in §9. The REST mode builds with or without the
   GUI; without the GUI, no GUI executable is produced.
3. **Engine concurrency:** separate session engines and configurable slice workers
   (§5.14); isolation and independent cancellation are tested by `E3 simultaneous
   engines cancel independently` in `rest_e2e_tests`.
4. **Coordinate-space details:** fixed for objects/instances and volumes (§5.12).
   Relative volume operations use the GUI's linear
   reference transform; absolute platter replacement uses the full instance
   transform. `instance` selects the reference, defaulting to the original,
   and volume bed placement affects every parent instance.
5. **Unknown-setting preservation:** format, provenance, and write priority are
   fixed in §5.5.1; no additional decision is required for the V1 contract.

### Known limitations

- Progress percentages/messages are indicative engine output, not a monotonic
  completion estimate; the engine progress rework is outside V1.
- No crash recovery or persistent sessions/results/tokens. Graceful shutdown
  cleans owned artifacts; a later startup removes stale work files.
- Accounts and profile catalogs are read only at startup; edits require restart.
- Built-in shape defaults are fixed because the REST server has no camera.
- Splitting into objects discards modifiers, matching the GUI operation.
- Merging is limited to objects that each have exactly one instance.
- Objects whose instances have different Z extents use the first instance's
  height for layer-height profiles and slicing parameters.
- Entity-setting combinations are checked by the slicer during slicing, not by
  the REST override writer. `extruders`/`wipe` scope is not filtered for a
  single-extruder printer, so clients may see settings the current printer does
  not use.
- Support enforcer and blocker parts are stored and sliced, but this build does
  not generate support from them yet: support generation is being migrated to
  the plugin pipeline.
- Defaults/selection copy only the presets they can read, but filament
  compatibility is still evaluated over the whole catalog once per distinct
  printer tool, so multi-tool printers in large catalogs cost more. These run on
  the import pool rather than blocking network polling; worker capacity and
  memory still limit concurrency. See deployment.md for measured observations.
- Viewer `metadata.objects` contains engine instance labels (for example
  `cube_stl id:0 copy 0`), not a separately reconstructed object-name list.
  Engine speed is duplicated in the two viewer speed fields in V1.
- Arrangement and orientation jobs use the import pool and are not cancellable;
  graceful shutdown waits for the active operation to finish. Arrangement
  ignores the wipe tower, and virtual beds are outside the printable area.
- Plane cuts currently do not create connectors, grooves, dowels, or contour
  selections. An object cut clears painting data on every source part; a solid
  part cut clears it only on the selected part, as required by the engine.
- Support facet painting is stored and exposed by REST, but this build does not
  yet migrate painted support facets into support generation, so it has no
  slicing effect. REST painting does not expose the GUI clipping plane.
- Custom G-code retains all color changes, pauses, templates and custom items
  that fall in the same layer, in source height order, as in the GUI's shared
  engine; a tool change applies in addition. Color and
  tool changes of a list are ignored by the engine when its stored mode and the
  current mode differ on the multi-extruder axis (`mode_mismatch`). Assigning any part or
  object to a non-default extruder makes the current mode `multi_extruder`, because
  the engine also counts each object's fallback region (default extruder). The API does
  not reproduce the GUI's other conflict icons (color of an unused extruder,
  redundant tool change).
- Debug builds of the engine assert on custom G-code heights of 10000 mm or more
  (`scale_to_layer_coord`) and on items at or above 1000 mm when slicing
  (`ToolOrdering::assign_custom_gcodes`). These are engine limits, distinct from
  the REST bound (the scene height); REST refuses 10000 mm or more. Slicing a layer
  range that reaches 10000 mm still hits the Debug assert (the mode computation
  clamps such ranges on its private copy).
- On multi-extruder sessions, `get_custom_gcodes` and `set_custom_gcodes` apply a
  `Print` to compute the mode, so session configurations that make `Print::apply`
  hit a Debug assert also abort these routes, as they do slice and arrange.
- Variable layer-height profiles are stored when an object is scaled, but the
  engine ignores a profile whose top no longer matches the object height. REST
  exposes no brush operation, and slicing parameters use the first instance of
  each object.
- The current slicing engine may crash or assert on multi-object scenes or on
  objects whose lowest point is above the bed; these plugin-pipeline limits are
  unrelated to plane cuts.
- Native TLS, browser CORS and remote administration are not provided. Operators
  must supply encrypted remote transport and validate their actual proxies.
- Linux/macOS packaging/services, actual proxies, visual GUI startup with future
  vendor keys and interactive terminal no-echo remain external validation items.
  See [platform-validation.md](platform-validation.md). A frozen contract does
  not claim these checks have been performed.

## 9. Executables and launch modes

The REST server is not a separate application: it is a launch mode of the
SuperSlicer executable, built when `SLIC3R_BUILD_REST_SERVER` is enabled, with
or without the GUI (`SLIC3R_GUI`). In the names below, `<app>` is the
application command name (`SLIC3R_APP_CMD`).

- **Windows.** The application is a library loaded by small launchers placed
  side by side: `<app>.exe` (GUI, only built with the GUI), `<app>_console.exe`
  (command line), and `<app>_rest.exe`, a console launcher that starts the REST
  server. Without the GUI, only the console and REST launchers exist.
- **Linux and macOS.** There is a single executable (inside the application
  bundle on macOS). `<app> --rest-server [options]` starts the REST server; a
  `<app>-rest` symbolic link next to it, created at build and install like the
  G-code viewer link, starts it the same way. Without the GUI, the executable
  runs either the command line or the REST server.
- **Options.** The REST mode accepts only its own options, after `--rest-server`
  or directly with the dedicated launcher/link: `--config PATH`,
  `--bind-address VALUE`, `--port VALUE` (`0` selects a free port),
  `--resources-dir PATH`, `--profiles-dir PATH`, `--work-dir PATH`,
  `--accounts-file PATH`, and `--help`. Command-line slicing actions and GUI
  options cannot be combined with it. The process runs until `SIGINT`/`SIGTERM`
  (Ctrl+C), or Windows `SIGBREAK` (Ctrl+Break), then cancels running slices and stops cleanly.
  `--rest-server` and `--rest-hash-password` must be the first argument after the
  program name. All arguments and JSON path strings are UTF-8 on every platform.
  Exit codes are `0` for help or normal shutdown and `1` for invalid options,
  configuration or startup errors.
- **Application resource defaults.** When neither JSON `paths.resources` nor
  `--resources-dir` supplies resources, application modes use the application's
  resource directory: `resources` beside the Windows launcher, `../Resources`
  in a macOS bundle, or the FHS/`../resources` layout on Linux. No paths are
  borrowed from GUI user data: installed REST profiles (`print`, `printer`,
  `filament` directories), work storage and accounts are supplied by REST
  configuration/options or retain the standalone current-directory defaults.
- **Accounts tool.** The Argon2id provisioning tool (§6) is also a launch mode of
  the same executable: `--rest-hash-password --username NAME` (Windows: through
  `<app>_console.exe`) reads the password without echo and prints the accounts
  JSON, as the standalone tool does.
  On Windows use `<app>_console.exe --rest-hash-password --username NAME` or
  `slic3r_rest_password.exe --username NAME`, not `<app>_rest.exe`, which already
  inserts `--rest-server`. REST/account console modes do not probe OpenGL/Mesa.
- **Standalone executables.** The build also provides two standalone targets,
  `slic3r_rest_server` (REST mode only) and `slic3r_rest_password` (accounts
  tool only), with the same options and behavior, for deployments that do not
  need the application executable. The application modes and the standalone
  executables share one implementation.
  Standalone default paths remain relative to the current working directory.

### Deployment behavior

See [deployment.md](deployment.md) for configuration, accounts, flat profiles,
service/proxy examples and the installed smoke workflow. JSON configuration rejects
unknown keys at every level. File response writes use `request_seconds` as an idle
deadline, rearmed only when bytes advance; total download duration is not bounded
by that value. Buffered JSON responses retain a whole-write deadline.

On startup stdout reports `listening on ADDRESS:PORT` after binding; IPv6 literals
are bracketed (`listening on [::1]:8080`). Normal
shutdown reports `stopping`, then `stopped` after joining workers. Windows also
accepts `SIGBREAK` (Ctrl+Break). Lifecycle lines contain no credentials.

The account tool accepts `--password-stdin` in either option order, with each option
appearing once. It reads one redirected UTF-8 line without transcoding and refuses console stdin, preventing
an echoed interactive password. No prompt/plaintext is emitted in this mode; the
same accounts document and exit codes apply. Without the flag, no-echo interactive
input remains unchanged on POSIX. Windows interactive input supports printable
ASCII only and rejects unsupported characters without emitting an account;
non-ASCII passwords must use UTF-8 `--password-stdin`. A username beginning with
`--` is rejected rather than consuming another option as a name.

### Launch verification targets

`rest_launch_tests` runs in its own process because launches finalize the engine
singleton, while `rest_server_tests` also verifies the uninitialized-engine state.
Build both test targets for launch and transport coverage. The four executable
application checks have the CTest label `rest_app` and require `Slic3r` plus the
Windows REST/console launchers (or the application binary and REST link on Unix).
For standalone-only builds, exclude them with
`ctest --test-dir build -C Debug -R "^rest_" -LE rest_app --output-on-failure`.
This still requires building the selected REST test targets and both standalone
executables; CTest does not build missing executables.
