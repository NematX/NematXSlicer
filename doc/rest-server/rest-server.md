# Specification — SuperSlicer REST Server

**Status:** proposed JSON contract, pending validation\
**Version:** 0.68

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
  internal library `slic3r_rest` and the standalone executable `slic3r_rest_server`;
- different sessions may slice simultaneously, with at most one active
  slicing operation per session;
- accounts are stored in a file administered outside the API; a local tool
  for generating Argon2id hashes must be created;
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
7. List objects, their parts, and their instances, then transform an object,
   instance, or part, or change the instance count.
8. Clear the platter, start slicing, save a 3MF project, and retrieve
   JSON data for the G-code viewer.
9. Delete a session.
10. Authenticate users by account and isolate their sessions.

### Outside the initial scope

- sending files to a printer or controlling a physical print;
- exposing the graphical interface's state or actions;
- providing a web interface;
- creating or administering accounts through the API; accounts are provisioned outside the API by the administrator;
- creating, modifying, importing, or deleting profiles through the API;
- SLA slicing support; V1 is limited to FFF/FDM.

## 4. Operating model

- `GET /list_settings` describes the settings the API can expose. The requested
  categories are `preference`, `print`, `filament`, `extruder`, and `printer`.
- `get_setting` and `get_settings` read the session's effective values;
  the former uses SuperSlicer setting serialization, while the latter returns
  typed JSON values for the requested settings, as does `set_settings`.
- Settings in the `preference` category are also session-local; reading or
  modifying them does not change the process's global preferences or those
  of other sessions.
- A session is an isolated workspace: selected profiles, settings,
  models/project, and scene state. It is addressed by an opaque `session_id`
  and belongs to the account that created it.
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
- Object, part, and instance identifiers are unique and stable within a session
  while the entity exists. They are invalidated when the entity is removed
  or the scene is replaced; transformations do not change retained entities' IDs.
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
  work finishes. Recovery after a server crash is outside the V1 scope.
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
| `POST` | `/create_session` | Return `session_id` |
| `GET` | `/session/{session_id}/list_profiles` | JSON available profiles |
| `POST` | `/session/{session_id}/set_profile` | JSON profiles to select |
| `GET` | `/session/{session_id}/get_setting/{setting_id}` | Serialized setting value |
| `POST` | `/session/{session_id}/get_settings` | Requested IDs in JSON; typed values in response |
| `POST` | `/session/{session_id}/set_setting/{setting_id}/{serialized_value}` | INI-style serialized value in the path |
| `POST` | `/session/{session_id}/set_settings` | JSON object of settings to modify |
| `POST` | `/session/{session_id}/add_model` | Upload an STL, OBJ, or 3MF |
| `POST` | `/session/{session_id}/load_project` | Upload a 3MF project |
| `GET` | `/session/{session_id}/list_objects` | JSON objects, parts, and instances |
| `POST` | `/session/{session_id}/transform/{id}` | JSON matrix applied to an object, instance, or volume/part ID |
| `POST` | `/session/{session_id}/set_instance_count/{object_id}` | JSON target instance count |
| `POST` | `/session/{session_id}/slice` | Start asynchronous slicing and return its initial state |
| `GET` | `/session/{session_id}/state` | `etat` and main/secondary progress |
| `POST` | `/session/{session_id}/cancel_slicing` | Request cancellation of active slicing (`202 Accepted`) |
| `GET` | `/session/{session_id}/get_gcode` | Download raw G-code after successful slicing |
| `POST` | `/session/{session_id}/clear_platter` | Clear the platter; JSON response |
| `GET` | `/session/{session_id}/save_project` | Binary 3MF download |
| `GET` | `/session/{session_id}/get_gcode_for_view` | Paginated viewer JSON (`offset`, `limit`, `result_id`) with an `outdated` flag |
| `DELETE` | `/session/{session_id}/delete` | Destroy the session; no response body |

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
Each of the five categories `preference`, `print`, `filament`, `extruder`,
and `printer` is a flat list of definitions.

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
  "printer": []
}
```

Every definition must contain `id`, `type`, `label`, and `default`.
`default` is a value object as described below, or `null` if no default exists.
All settings in the `filament` and `extruder` categories have `"array":"extruder"`.
Optional fields are `description`, `unit`, `min`, `max`, `precision`, `choices`,
`read_only`, `can_be_disabled`, and `array`. `can_be_disabled` indicates whether
the setting is disableable; omission means `false`. A `choices` entry has the
form `{"value":"...","label":"..."}`. `type` remains the element type;
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

No request body. Return `201 Created`:

```json
{"session_id":"session_opaque_id"}
```

### 5.4. `GET .../list_profiles`

Return installed profiles as flat lists by type. Each profile must contain an
opaque `id` and a display `name`; only `vendor` and `technology` may additionally
be present, both optional.

```json
{
  "print": [{"id":"print-id", "name":"0.20 mm"}],
  "printer": [{"id":"printer-id", "name":"My printer"}],
  "extruder": [],
  "filaments": [{"id":"filament-id", "name":"PLA"}]
}
```

Catalog lists are independent of the ordered arrays sent to `set_profile`.
Extruder profiles do not exist yet: `extruder` is an empty list in V1.
Settings in the `extruder` category remain available, notably through the
printer profile; they do not require independent extruder profiles to exist.

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
preserved as custom data according to §5.5.1. `add_model` does not apply the
project configuration of a 3MF imported as a model. Both routes return newly
created IDs:

```json
{"object_ids":["object-id-1"],"object_count":1}
```

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
      "matrix": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "parts": [
        {"id":"part-id", "name":"part.stl", "matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}
      ],
      "instances": [
        {
          "id": "instance-id",
          "matrix": [1,0,0,10,0,1,0,20,0,0,1,0,0,0,0,1]
        }
      ]
    }
  ]
}
```

### 5.12. `POST .../transform/{id}`

The route resolves `id` within the session, without an additional parent segment:

- object ID: transform all its instances;
- instance ID: transform only that instance;
- volume/part ID: transform that volume, shared by the object's instances.
  Other volumes are not modified.

An unknown ID returns `404`. An object transformation is atomic across all
its instances.

The body contains `matrix`, a flat array of 16 finite numbers in row-major
order: `[m00,m01,m02,m03,m10,...,m33]`, equivalent to four rows of four values.
The homogeneous matrix acts on column vectors; translation is in millimeters
at indices 3, 7, and 11, and the last row (indices 12 through 15) is `[0,0,0,1]`.
`action` accepts `compose` or `replace`; omission defaults to `compose`.
`compose` applies a relative transformation; `replace` sets an absolute
transformation expressed in the chosen space.

The required `space` field preserves the GUI's three rotation/translation
coordinate-space choices:

| `space` | Coordinate space |
| --- | --- |
| `platter` | Global platter space |
| `object` | The placed object's space, i.e. its instance space |
| `volume` | Target volume/part's local space; for an object or instance target, instance space |

Space conversion must account for parent transformations. With `A` denoting
the received matrix, `I` the instance matrix, and `V` the volume matrix,
composition is defined as follows when the instance context is determined:

- instance in `platter`: `I' = A × I`; in `object`: `I' = I × A`;
- volume in `object`: `V' = A × V`; in `volume`: `V' = V × A`;
- volume in `platter`: `V' = I⁻¹ × A × I × V`.

Parent and local spaces therefore do not use the same multiplication order.
Pivots and the reference instance choice for a volume shared by multiple
instances must align with GUI operations; remaining details are listed in §8
before implementation. `replace` also requires this space conversion, not a
blind copy of the received matrix into the engine's local matrix.

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

`count` is a positive integer. The response contains the object and its instance
IDs after the change:

```json
{"object_id":"object-id","count":3,"instance_ids":["i0","i1","i2"]}
```

New instances are superimposed on the original: they copy its transformation
and receive new IDs. No automatic arrangement is performed. The client can then
move them individually through `transform/{id}`. The original instance is first
in the list; decreasing `count` removes instances from the end, without changing
retained instances' IDs or transformations. `count=1` retains the original.

### 5.14. `POST .../slice`, `GET .../state`, `cancel_slicing`, `get_gcode`, `clear_platter`, `save_project`, and `delete`

- `/slice` has no body. It starts slicing in the background and returns
  `202 Accepted` with a JSON state snapshot matching `GET .../state`
  (normally `etat: "slicing"`).
- A second `/slice` call while the session is already `slicing` returns
  `409 Conflict`.
- `GET .../state` returns JSON containing the two `PrintStatus` channels and
  the overall session state:

  ```json
  {
    "etat": "slicing",
    "primary_message": "Slicing objects",
    "primary_percentage": 90,
    "secondary_message": "Generating infill",
    "secondary_percentage": 100
  }
  ```

  `primary_message` and `secondary_message` are strings. Both percentages are
  integers; `-1` means progress is unavailable. `etat` is `ready`, `slicing`,
  `sliced`, or `failed`. On failure, `primary_message` provides the error message.
- `sliced` is published only after successful slicing, G-code export, and
  preparation of the viewer result. No partial file or result is published
  on failure or cancellation.
- `POST .../cancel_slicing` has no body and requests that active slicing stop.
  It returns `202 Accepted` immediately; `GET .../state` reports `ready` once
  work has effectively stopped. It returns `409 Conflict` if no slicing
  operation is active. This is the only mutation accepted while `etat: "slicing"`.
- `GET .../get_gcode` has no body and downloads the `.gcode` file after successful
  slicing (`etat: "sliced"`). It returns `409` if no current result is available.
  `get_gcode_for_view` remains the viewer's JSON output.
- `/clear_platter` has no body and returns `{"cleared_object_count":3}`.
  Session profiles and settings are retained.
- `/save_project` returns a binary 3MF file, not a JSON object.
- `/delete` returns `204 No Content`; the session ID and data are invalidated.

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
- Session deletion or inactive expiration also destroys retained results.
  A session cannot expire while slicing.

### 5.15. `GET .../get_gcode_for_view`

Return versioned JSON with metadata, layers, and one page of moves from the
session's last successful slicing result, even if stale. The request accepts
`offset` (zero-based, default `0`), `limit` (default `10000`, between `1` and
`100000`), and `result_id`. The route returns `409` if no successful result
has been retained. Metadata and layers are included in every page;
only `moves` is paginated.

Every response contains an opaque `result_id` and an `outdated` boolean.
The first request at `offset=0` may omit `result_id` to retrieve the latest
retained result. Subsequent pages (`offset>0`) must supply that ID.
If the result has been replaced, return `409` with code `result_replaced`,
without supplying a page from another slicing operation. Invalidation changes
`outdated` but not the retained result's ID, moves, metadata, or layers;
a new success produces a new ID and `outdated: false`.

Units are explicit in field names. `object_id` is a zero-based index into
`metadata.objects`, or `null` for a move outside an object. `layer_id` is
zero-based. Move types use the names `noop`, `retract`, `unretract`, `seam`,
`tool_change`, `color_change`, `pause_print`, `custom_gcode`, `travel`, `wipe`,
and `extrude`.

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
    "objects": [{"id":0,"name":"part.stl"}],
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
      "color_change_id": null,
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
without a role. `color_change_id` is null outside color changes.
Arrays may be empty. `next_offset` is `null` when the final page is reached.

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

Accounts are created by the administrator outside the API. A local tool for
generating salted Argon2id hashes must be created and shipped with the server.
Its output must be directly usable in the accounts file, including the salt
and verification parameters; no plaintext password is persisted or logged.
This tool is not a functionality already available.

## 7. Acceptance criteria

V1 is acceptable if:

1. The server starts as a standalone process and works without launching the GUI.
2. `/list_settings` returns settings grouped by category; `/create_session`
   returns a `session_id` usable for session routes.
3. A session can list and select profiles, read one or more selected settings,
   and modify one or more settings. An empty extruder catalog does not prevent
   slicing. JSON values follow the `value`/`enabled`/`percent` object contracts,
   including parallel arrays and retention of disabled values. Loading a profile
   clears the relevant local modifications; the target rules in §5.5.1 are
   verified after integration of the corresponding engine changes.
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

## 8. Decisions required before freezing the contract

1. **Accounts:** determine the path and JSON schema of the administrator-managed
   file used by the server.
2. **Headless build:** confirm whether the server builds only in a dedicated
   `SLIC3R_GUI=OFF` configuration or also in the GUI configuration.
3. **Engine concurrency:** before enabling parallel slicing, validate and, if
   necessary, correct isolation of global `libslic3r` state between sessions.
4. **Coordinate-space details:** specify the reference instance for a shared
   volume transformed in global space and the GUI pivots and `replace` behavior
   in each space. The single route and three spaces are decided; their detailed
   mapping to GUI operations must be verified before the transformation phases.
5. **Unknown-setting preservation:** format, provenance, and write priority are
   fixed in §5.5.1; no additional decision is required for the V1 contract.
