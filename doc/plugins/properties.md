# Using Plugin Properties

> Base snapshot commit: `b1692c6920`, with the host-policy extraction described below.
>
> Per-header ABI: `slic3r_extrusion_property.h` 4.0, `slic3r_def.h` 1.5.
> Public policy records: `slic3r_fan_policy.h` 2.0,
> `slic3r_printing_layer_time.h` 1.0; append-only API IDs in `slic3r_plugin_types.h` 4.2.

The commit above identifies the source-tree state against which this guide and
its examples were checked. It is not the commit that adds this document: a
commit cannot contain its own final hash. Update both the snapshot hash and the
per-header ABI requirements whenever this guide is revised for a later plugin API.

Plugin properties are small typed metadata records attached to objects already
owned by the slicer. They let one plugin annotate an object and a later plugin
consume that annotation without adding a field to the native object or coupling
the two plugins to a host implementation class.

This guide uses the C++ API as the primary interface. The final section maps the
same concepts to the C ABI and Python views.

## The Three Parts Of A Property

Every property has:

1. A **payload type**, which describes the bytes stored on an object.
2. A **numeric property type**, which identifies what those bytes mean.
3. A **typed key**, `PluginPropertyKey<Payload>`, which keeps the payload type,
   numeric type and registering orchestrator together.

Property payloads are copied as raw bytes, and newly created payloads are
zero-initialized. They must be trivially copyable and must not own memory. Do
not put `std::string`, `std::vector`, smart pointers, virtual methods or other
resource-owning C++ objects inside a payload.

There are two kinds of key:

- **Built-in keys** have a stable ID published by the host. Their C++ payloads
  expose a ready-to-use static key such as `EPropertySpeed::key`.
- **Dynamic keys** are registered by a plugin under a stable namespaced name.
  Their compact numeric ID belongs to one orchestrator and may differ in another
  run. Keep the key in the plugin instance and never serialize `key.type()`.

The same `PluginPropertyKey<Payload>` can address a generic data-tree property
or an extrusion property. The owner passed to the key selects the storage.

## Choosing The Property Owner

| Owner | C++ access | Lookup behavior |
| --- | --- | --- |
| `Surface`, `Layer`, `LayerRegion`, `LayerIsland`, `LayerRegionIsland`, `Object` | `view.properties()` | Direct metadata on that object; no inheritance |
| `PrintingPlan`, `PrintingGroup`, `PrintingLayerGroup`, `PrintingToolGroup` | `scope.properties()` | Direct metadata on that final-plan scope; no inheritance |
| `ExtrusionEntity` and mutable extrusion views | `entity.get(key)` | Direct property on that node only |
| An extrusion tree being visited | `current_property(key)` | Effective property inherited from the nearest node toward the root |

Use the narrowest owner whose lifetime matches the information. For example, a
classification of one surface belongs on that `Surface`; a value applying to
all descendants of one extrusion subtree belongs on its parent extrusion node;
and a duration for a final ordered layer belongs on its `PrintingLayerGroup`.

## Using A Built-in Property

Built-in data-tree payloads are declared in
[`DataTreeProperties.hpp`](../../src/libslic3r/Api/plugin/cpp/properties/DataTreeProperties.hpp).
Built-in extrusion payloads are declared in
[`ExtrusionProperties.hpp`](../../src/libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp).

The static `key` is the preferred API for every operation:

```cpp
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

using namespace slic3r_api;

void annotate_support_layer(const Layer &layer)
{
    PluginProperties properties = layer.properties();

    LayerSupportProperty &support =
        properties.get_or_add(LayerSupportProperty::key);
    support.interface_id = 2;

    if (const LayerSupportProperty *stored =
            properties.get(LayerSupportProperty::key)) {
        use_interface_id(stored->interface_id);
    }

    properties.remove(LayerSupportProperty::key);
}
```

The key-first syntax is equivalent and is useful when a helper already owns the
key:

```cpp
const LayerSupportProperty *support =
    LayerSupportProperty::key.get(layer.properties());
```

For an extrusion, pass the same kind of key directly to the entity view:

```cpp
const EPropertySpeed *speed = entity.get(EPropertySpeed::key);

EPropertySpeed &editable_speed =
    mutable_entity.get_or_add(EPropertySpeed::key);
editable_speed.speed(35.f).acceleration(800.f);
```

`get()` and `get_mutable()` return `nullptr` when the direct property is absent.
`get_or_add()` returns the existing payload or creates a zero-initialized one.
`remove()` returns whether a property was actually removed.

## Public Layer Timing And Fan References

Two dynamic properties carry pipeline metadata without borrowing host objects:

| SDK helper | Owner | Stable property name |
| --- | --- | --- |
| `PrintingLayerTimeProperty.hpp` | `PrintingLayerGroup::properties()` | `slic3r.printing_layer.time` |
| `FanPolicy.hpp` (`FanPolicy::Context`) | `PrintingToolGroup::properties()` | `slic3r.printing_tool.fan_context` |

Their minimal C headers define the layouts independently of Config, orchestrator
and tree APIs. Register C++ keys with `printing_layer_time_property_key(orch)`
and `FanPolicy::context_property_key(orch)` before parallel processing. C callers
use `orchestrator_register_property()` with the published name, `sizeof` and
alignment. Numeric IDs are local to that orchestrator, not preset data.

Timing is the **whole layer's duration**, shared by every tool visit, not a
per-tool duration. `ESTIMATED` is a replaceable geometric fallback. `RESOLVED`
means a producer ran its timing model after speed/slowdown changes; it does not
mean measured machine time or provide intra-movement timing. Both origins
require finite, nonnegative seconds. A producer must republish after modifying
speeds. `FanCooling` keeps valid RESOLVED records and recomputes
other durations in one ordered estimation pass. No post-slowdown producer is
implemented by these contracts.

Fan references are distinct from extrusion requests. `configured_percent` keeps
normalized filament input (`-1` disabled, historical `1` mapped to `0`), while
`percent` is the current reference (`-1` absent, `0` a real stop). Fallback rules
are constants in the policy implementation, not published per-tool data. Early
family fallbacks are already applied in BASE. Filament normalization must never be
applied to a regional or user request of 1%.

The value-only SDK stages consume copies and enforce this order:

```cpp
const FanPolicy::Settings settings = FanPolicy::read_settings(config, extruder_id);
const FanPolicy::Context base = FanPolicy::make_base(settings);
const FanPolicy::Context timed = FanPolicy::apply_layer_time(base, settings, duration_seconds);
const FanPolicy::Context closed = FanPolicy::close_first_layers(timed, settings, layer_index);
```

`BASE -> TIME -> CLOSED` preserves role exceptions, time-threshold rounding,
the D/F first-layer program and late fallback closure. `struct_size` must match
the C layout: zero-initialized storage is not a published context. Contexts are
computed even when all extrusion requests are protected. Dynamic bounds apply
only to modifiable requests, never to Override or another plugin's mode.
`FanPolicy::transform_request()` preserves protected pairs, updates a modifiable
pair without changing its active mode, and assigns the caller's mode when absent.
Its inputs follow `can_modify()`'s prevalidated-mode contract below.

`slic3r_fan_policy.h` 2.0 publishes results only: references, phase and dynamic
bounds. It is incompatible with the former 1.0 layout. Settings and layer ordinal
remain explicit inputs; the timing header remains 1.0. `read_settings()` validates
configuration once; callers assembling `Settings` directly must call `validate()`.
Validate a context read from a published property with `context.validate(phase)`
before processing it. Numerical stages only check size/phase and the separate
duration argument; they do not rescan references they just computed. Property
timing and estimator output are validated when entering the FanCooling cache,
not again when publishing that unchanged cache.

The host runs independent plugins at the published layer-edit priorities:

| Plugin ID suffix (`layer_extrusion_edit.fan.`) | Priority | Result |
| --- | --- | --- |
| `default` | 20 | Fill only effective absences and publish BASE |
| `cooling` | 22 | Transform eligible requests using total duration; publish TIME |
| `first_layers` | 24 | Apply D/F, close late fallbacks; publish CLOSED |
| `dynamic` | 26 | Increase geometric overhang requests within CLOSED bounds |

The standard retains the `default_fan_plugin` exclusive group. Other stages have
no provider-identity dependency. Each consumes and fully validates the preceding
property before processing a tool. Without optional cooling, the first-layer
applicator accepts BASE as an identity TIME stage, without estimating duration.
Dynamic processing requires CLOSED even when its graph is disabled. Setup only
caches durations; publication happens in the owning layer's run.

Every later policy preserves Override (including zero) and other plugins' modes;
Automatic zero is a real request but may be transformed. No intermediate fan
compaction is performed. The standard materializes inherited collection requests
on eligible deposition leaves, leaving direct service requests unchanged and
not copying a collection request onto travel, empty or service leaves. A printable
leaf whose E is replaced by a raw `EXTRUSION` command (such as a zero-flow infill
connector) is not a deposition leaf: it keeps the fan state of its neighbours. Reprocessing
restores the saved preprocessing tree, rather than undoing prior transformations.

With a disabled dynamic graph, a configured `overhangs_fan_speed` is honored
(unlike the earlier combined provider's unconditional perimeter fallback).
Missing overhang requests wait for the closed perimeter-family reference;
upstream Automatic overhangs retain bridge-role ramp exceptions. Dynamic cooling
never lowers an existing request above its own ceiling. A disabled default fan
uses ramp(100) as the dynamic ceiling, not a manufactured stop. The advanced E/D/G
program, slowdown solver and temporal processing remain separate future work.

Each extracted plugin owns its existing option definitions and invalidates
`STEP_LAYER_EXTRUSION_EDIT`. Shipped defaults enable all four; migration copies
the old standard's activation to missing successors, retaining explicit choices.
Public priority constants are additive in `slic3r_step_layer_extrusion_edit.h` 1.1;
fan2.0 and timing1.0 record layouts are unchanged.

Python exposes the same ctypes records through `slic3r_fan_policy`:
`FanPolicyContext`, `PrintingLayerTimeProperty`, `fan_policy_context_key(api)`,
`printing_layer_time_property_key(api)`, `validate_fan_policy_context(record, phase)`
and `validate_printing_layer_time(record)`. Registration requires the owning API;
successful validation returns `None`. Numerical kernels are in the C++ SDK archive.

## Fan Requests: Absence Is A Mode, Not A Percentage

`EPropertySpeed` contains only speed, acceleration and pressure advance. Fan
requests have their own built-in key, `EPropertyFan::key`, and C payload:

```cpp
struct c_extrusion_property_fan {
    float percent;
    orchestrator_runtime_id mode;
};
```

The standard exclusive modes are `FAN_REQUEST_MODE_UNSPECIFIED` (0),
`FAN_REQUEST_MODE_AUTOMATIC` (1), and `FAN_REQUEST_MODE_OVERRIDE` (2).
There are no flags or combinable bits. A zero-filled payload is neutral.
`clear()` restores `{0, Unspecified}`; `set(percent, mode)` updates the pair.
Active percentages must be finite and in [0,100], including explicit stop at 0.

Inheritance applies to the **whole pair**. A local Unspecified request inherits
the parent even if a local payload exists. `resolved_with(parent)` resolves a
pair. A generic tree visitor can skip neutral payloads with a predicate:

```cpp
const EPropertyFan *fan = current_property(EPropertyFan::key,
    [](const EPropertyFan &request) { return request.is_set(); });
```

This returns null if no active request exists. Raw `current_property(key)`
still returns the nearest payload, not its field-specific semantic resolution.

Only add a default when the **effective** request is absent/Unspecified.
Automatic0, Override0 and every registered custom mode are already set.
For transforming policies, `request.can_modify(my_mode)` implements the shared
conflict rule: Unspecified and Automatic may be modified by any policy; Override
is protected against every policy (even one passing Override as `my_mode`); a
plugin's own mode may only be modified by that plugin. Validate the effective
request and the policy's registered mode before arbitration. `set()` remains a
direct producer operation, not a policy-arbitration operation.

Plugins can register extra exclusive modes with the existing typed registry:

```cpp
slic3r_api::RuntimeIdDomain modes(FAN_REQUEST_MODE_ID_TYPE);
const fan_request_mode special = modes.register_name(orch, "com.example.cooling.special");
mutable_entity.get_or_add(EPropertyFan::key).set(40.f, special);
```

Register before parallel processing and retain the ID per orchestrator/plugin
instance, never in presets or serialized models. The host reserves the three
standard names published by `FAN_REQUEST_MODE_NAME_*`. The generic domain helper
provides `register_name(orch, name)`, `name(orch, id)`, `is_valid(orch, id)` and
`validate(orch, id)` for any typed runtime-ID namespace. It borrows a constant
domain name, which must outlive the helper, and retains no orchestrator.
`EPropertyFan::is_valid(orch)` directly checks active values and registration,
including raw C writes; `validate(orch)` throws if that check fails. Unspecified
ignores its percentage, even when raw writes do not use the canonical zero.
Python exposes `RuntimeIdDomain` from `slic3r_runtime_id`, passing `api` instead
of `orch` to each operation. Fan helpers use `is_valid(api)` and `validate(api)`;
successful validation returns `None`.

Registration defines **identity, not behavior**. Modes arbitrate between policies,
not firmware encodings. The host validates registration in the export's
orchestrator once at the host's `STEP_GCODE` prevalidation boundary. Firmware
then encodes the percentage of **every active request**, without conversion to
Automatic or knowledge of the originating policy. Unregistered modes remain
host validation errors. Firmware checks only active percentages and needs no
orchestrator for fan encoding. Direct SDK callers must supply prevalidated modes.

Firmware applies minimum and per-tool offset once: absent emits nothing; zero
stays zero; positive values encode as `clamp(max(percent, minimum)+offset,0,100)`, as in the legacy slicer.
Each firmware alternative defines `fan_printer_min_speed` in the shared
`gcode_firmware_plugin` exclusive group. SDK helper
`GCodeGeneration::fan_minimum_option_definition()` in `gcode/FanConfig.hpp`
provides identical metadata, including the `Slic3r` translation domain, for
built-in and external providers. Do not depend on Marlin2 being active or
redefine this physical setting in a layer policy.
An empty leaf with a direct active fan request emits once without motion.
Inherited requests do not turn empty heater/script leaves into fan events.
Leaving the event does not emit a lexical ancestor or undo the physical output.

This is an incompatible Speed layout change. Rebuild packages against API 4.0;
old packages must be rejected by their sidecar and exported version table.

## Defining A Dynamic Property

A dynamic payload is an ordinary trivial struct. Register its stable name in
every plugin instance that produces or consumes it, and retain the returned key
as a member:

```cpp
#include <cstdint>
#include <type_traits>

#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

struct SurfaceQualityHint
{
    uint32_t source_id;
    float score;
};

static_assert(std::is_trivially_copyable<SurfaceQualityHint>::value,
              "A plugin property payload must be trivially copyable.");

class QualityPluginState
{
public:
    explicit QualityPluginState(orchestrator_handle *orchestrator) :
        m_quality_hint(
            slic3r_api::PluginPropertyKey<SurfaceQualityHint>::register_dynamic(
                orchestrator,
                "com.example.quality.surface_hint"))
    {
    }

private:
    slic3r_api::PluginPropertyKey<SurfaceQualityHint> m_quality_hint;
};
```

The name is the contract between plugins. Registering the same name, payload
size and alignment on one orchestrator returns the same runtime ID. Registering
the same name with an incompatible layout is rejected.

Once registered, built-in and dynamic properties use exactly the same syntax:

```cpp
SurfaceQualityHint &hint =
    surface.mutable_properties().get_or_add(m_quality_hint);
hint.source_id = source_id;
hint.score = score;

const SurfaceQualityHint *stored = m_quality_hint.get(surface.properties());
```

Default member initializers and constructors are not executed when the host
creates a payload. Initialize every field whose valid default is not all-zero
after detecting that the property is new.

## Direct And Inherited Extrusion Properties

`entity.get(key)` reads only the property physically stored on that entity. It
does not inspect parents. This makes direct lookup predictable and inexpensive.

During a tree traversal, use `current_property(key)` to resolve the effective
value. It searches from the current node toward the root and returns the nearest
definition:

```cpp
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"

class SpeedReader : public slic3r_api::ExtrusionTreeConstVisitor<>
{
protected:
    void visit_leaf(slic3r_api::ExtrusionEntity leaf) override
    {
        const slic3r_api::EPropertySpeed *speed =
            current_property(slic3r_api::EPropertySpeed::key);
        if (speed != nullptr)
            consume_speed(leaf, speed->speed_mm_per_s);
    }
};
```

A child property overrides a parent property of the same type for that subtree.
The returned pointer is borrowed from whichever entity owns the winning
property. Do not retain it after mutating the tree.

## Storing Variable-size Extrusion Data

Property payloads stay small and fixed-size. An extrusion property that needs a
string or larger binary value stores an `extrusion_data_id` in its payload and
keeps the bytes in the extrusion entity's auxiliary storage.

Use `store_property_string()` or `store_property_value()` when the data is owned
by a field in a property. The host then replaces the previous buffer safely and
releases it when the property is removed:

```cpp
struct LabelProperty
{
    extrusion_data_id label_id;
    uint32_t category;
};

const bool already_present = mutable_entity.has(m_label_key);
LabelProperty &label = mutable_entity.get_or_add(m_label_key);
if (!already_present)
    label.label_id = EXTRUSION_DATA_ID_INVALID;

if (mutable_entity.store_property_string(
        m_label_key.type(), &label.label_id, text) ==
    EXTRUSION_DATA_ID_INVALID) {
    throw std::runtime_error("The extrusion label could not be stored.");
}
```

The initialization to `EXTRUSION_DATA_ID_INVALID` matters because a newly
created payload is zeroed, while the invalid data ID is not zero.

Use `store_string()` or `store_value()` only for standalone data whose lifetime
is managed separately with `free_data()`. Auxiliary data belongs to the same
entity that stores it. An inherited payload pointer alone does not identify the
parent entity that owns its referenced data.

Generic data-tree and PrintingPlan properties do not have property-owned
variable buffers. Use `PrintRecords` for variable, print-wide `DynamicConfig`
data instead of putting an owning pointer in a property.

## Lifetime, Mutation And Threading

- `PluginPropertyKey` is a small non-owning value. A dynamic key borrows its
  orchestrator and must not outlive that plugin runtime.
- Pointers and references returned by property access are borrowed. Treat them
  as short-lived and reacquire them after modifying, moving or replacing the
  owner.
- Some const source data-tree views expose mutable plugin metadata through
  `properties()` when their step contract permits annotation. Finalized
  `ConstPrintingPlan` scopes expose `ConstPluginProperties` instead; mutation
  requires a mutable plan-scope view from an editing step.
- Property containers are not synchronized. Parallel workers may modify
  distinct owners. Concurrent mutation of the same owner requires plugin-side
  synchronization.
- Copying or moving a host object follows that object's documented property
  behavior. Do not use a property pointer as an ownership or identity token.

## Property, Config, PrintRecord Or Auxiliary Data?

| Need | Preferred mechanism |
| --- | --- |
| Small fixed metadata attached to one data-tree object or plan scope | `PluginProperties` |
| Small fixed process metadata attached to an extrusion subtree | Extrusion property |
| Text or binary data owned by an extrusion property | `extrusion_data_id` with `store_property_*()` |
| Typed option values, serialization or script arguments | `Config` / `DynamicConfig` |
| Variable print-wide tables or shared records | `PrintRecords` |

Avoid using a property as a general object store. Its strength is a compact,
copyable contract that follows the object carrying it.

## C And Python Equivalents

| Operation | C++ | C | Python |
| --- | --- | --- | --- |
| Register a dynamic property | `PluginPropertyKey<T>::register_dynamic()` | `orchestrator_register_property()` | Custom extrusion helper: `api.register_extrusion_property_type()` |
| Read data-tree metadata | `properties.get(key)` | `plugin_property_data()` | `properties.get(PayloadClass)` |
| Create data-tree metadata | `properties.get_or_add(key)` | `plugin_property_get_or_add_data_mutable()` | `properties.get_or_add(PayloadClass)` |
| Read a direct extrusion property | `entity.get(key)` | `extrusion_property_data()` | `entity.property(PayloadClass)` |
| Create an extrusion property | `entity.get_or_add(key)` | `extrusion_property_get_or_add_data_mutable()` | `entity.property(PayloadClass)` on a mutable view |
| Remove a property | `owner.remove(key)` | `plugin_property_remove()` / `extrusion_property_remove()` | `remove()` / `remove_property()` |

Python identifies a property through the numeric `property_type` attached to a
`ctypes` payload class; it does not currently expose the C++
`PluginPropertyKey` object. Read-only extrusion views return payload copies,
while mutable views return live `ctypes` views over host memory. The current
high-level custom registration helper targets extrusion properties; generic
data-tree registration may require the lower-level host call.

## Review Checklist

Before publishing a property contract, verify that:

- the payload is trivially copyable and has no owning members;
- its dynamic name is stable and namespaced;
- its key is stored per orchestrator and its numeric ID is never serialized;
- every non-zero default or sentinel is initialized after creation;
- direct versus inherited lookup is intentional;
- borrowed pointers do not survive owner mutations;
- parallel writers cannot reach the same owner unsynchronized;
- variable data uses the appropriate auxiliary storage instead of an in-payload
  pointer.

## Instance-owned auxiliary layers

`c_layer_instance_property` (built-in id 16, `slic3r.layer.instance`) stores
`uint32_t object_instance_idx` and `uint32_t reserved` (zero). On a real object's
auxiliary layer it restricts object-local geometry to that instance, indexed as
`Object::instance_shift(i)`. Without the property the layer is a common local
pattern for all instances. The global auxiliary object stays in plateau space
and does not use this placement marker. SDK views expose raw local geometry.
`LayerInstanceProperty` is available in native C++, SDK C++ and Python;
`layer_instance_projections(object, layer)` validates the owner and returns the
scaled instance placements in SDK C++. Invalid owner indices throw rather than
silently dropping geometry. This addition requires DEF 1.3 / DATA_TREE 5.1.
