# Plugin-defined option lists

Some enum settings list values that are owned by plugins instead of by the core
configuration. The infill pattern lists are the reference example: the
`infill.generator.default` plugin defines `fill_pattern`, `top_fill_pattern`,
`bottom_fill_pattern`, `solid_fill_pattern` and `bridge_fill_pattern`, and each
pattern plugin declares the lists it belongs to.

The C ABI is declared in `Api/plugin/c/slic3r_config_def.h`
(`orchestrator_create_option_def`, `orchestrator_modify_option_def`).

## Plugin ids as enum values

Set `contains_plugin_id` on a plugin-created enum definition when every value is
a plugin id:

```cpp
raw_config_option_def def = raw_config_option_def_init();
def.opt_key = "solid_fill_pattern";
def.type = RAW_CO_ENUM;
def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
def.container_type = RAW_CONTAINER_TYPE_REGION;
def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
def.printer_technology = RAW_PT_FFF;
def.contains_plugin_id = 1;
def.default_serialized_value = "ensuring";
def.enum_def.value_label_pairs = { entries, entry_count };
orchestrator_create_option_def(orchestrator, &def);
```

Such a list may only contain plugin ids. When plugin option definitions are
finalized, the host removes values whose plugin is missing or inactive. A list
left empty reports a non-fatal plugin error and stays empty; it does not abort
startup. Never mix host-only tokens into a `contains_plugin_id` list.

The option serializes the plugin id, so presets stay portable across plugin
sets. In memory the value is an index into the resolved list, which the host
rebuilds during finalization.

## Deferred patches

`orchestrator_modify_option_def()` records a modification that is applied after
every plugin has initialized. Patches are independent of initialization order
and may target a key that another plugin creates later; a patch whose target is
never defined is ignored.

| Patch kind | Effect |
|---|---|
| `ADD_ENUM_VALUES` | append (or prepend with `prepend = 1`) value/label pairs; existing values keep their first position |
| `REMOVE_ENUM_VALUES` | remove the listed values |
| `SET_DEFAULT` | replace the default with `default_serialized_value` |
| `REPLICATE_ENUM` | replace the list with a snapshot of `source_option_key` |

Pattern plugins use `ADD_ENUM_VALUES` to join the lists they support:

```cpp
key_value_string_pair_t entry = { id_impl(), name_impl() };
key_value_string_pair_array_t entries = { &entry, 1 };

raw_option_def_patch patch = raw_option_def_patch_init();
patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
patch.opt_key = "solid_fill_pattern";
patch.enum_values = entries;
orchestrator_modify_option_def(orchestrator, &patch);
```

Unknown target keys are kept until finalization, so a pattern never needs to
know whether a list already exists.

## Resolution order

Patches are applied by `Orchestrator::finalize_plugin_option_defs()` after all
plugins initialized and before the configuration definitions are finalized.
`REPLICATE_ENUM` is applied before every `ADD_ENUM_VALUES` and
`REMOVE_ENUM_VALUES` patch, so a contribution recorded before the target list
existed is not erased by a replication patch. The result therefore does not
depend on plugin initialization order. `REPLICATE_ENUM` creates a dependency
edge to its source, and the host resolves the resulting tree depth-first so
sources are finalized first.

Dependency cycles are broken softly: an edge to an option that is still being
resolved is ignored and the option keeps the values it already has. Cycles are
therefore deterministic, but they are not detected as errors.

## GUI rules

Pattern lists publish an activation reason for each target where the setting is
defined. Because a plugin-id enum stores ids, use `STRING_EQUALS` instead of an
integer enum comparison. The activation group of a rule derives from its target
setting, so no group is declared:

```cpp
GuiRule::activate({ "solid_fill_pattern" })
    .when(has_solid)
    .scope(RAW_PRESET_TYPE_FFF_PRINT)
    .add_to(orchestrator);
```
