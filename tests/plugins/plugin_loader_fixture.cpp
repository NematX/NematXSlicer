///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* This library supplies controlled compatibility and registration failures to loader tests.
   The host is the slicer application. Its package loader opens a native shared
   library and looks up exported functions, functions available by name outside
   the library. Build definitions select which deliberately invalid library this
   source represents; it is not a plugin used for normal slicing.

   The loader queries slic3r_plugin_abi_version() before register_plugin(). The
   former describes required versions of the C ABI, the binary calling interface
   between library and application. Build definitions select incompatible versions,
   missing functions or invalid table sizes. The incompatible table is never
   passed to normal application execution.

   When available, register_plugin() supplies the selected registration behavior,
   including a deliberate exception. Loader tests open these libraries through
   normal package discovery and inspect application-owned diagnostic reports.

   Loader call order:
     slic3r_plugin_abi_version
     register_plugin, only after compatibility checks */

#include <stdexcept>
#include <cstddef>

#ifdef SLIC3R_TEST_PLUGIN_LEGACY_ABI
// Deliberately retain the old signature without including its new declaration.
#include <cstdint>
struct orchestrator_handle;
#ifdef _WIN32
#define SLIC3R_PLUGIN_API __declspec(dllexport)
#else
#define SLIC3R_PLUGIN_API
#endif
#ifndef SLIC3R_TEST_PLUGIN_MISSING_REGISTER
// Ignore the borrowed application; configured throwing cases report an unexpected registration attempt without taking ownership.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *);
#endif
extern "C" SLIC3R_PLUGIN_API uint32_t slic3r_plugin_abi_version(void) { return 51u; }
#else
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
static_assert(SLIC3R_PLUGIN_API_PLUGIN_TYPES == 0, "Vtable version must be first.");

#ifndef SLIC3R_TEST_PLUGIN_MISSING_ABI
// Fill the caller-owned header-version array or return its required count; no plugin instances are registered, and exceptions cannot cross C.
extern "C" SLIC3R_PLUGIN_API int32_t slic3r_plugin_abi_version(slic3r_major_minor_version *requirements);
#endif
#ifndef SLIC3R_TEST_PLUGIN_MISSING_REGISTER
// Ignore the borrowed application; configured throwing cases report an unexpected registration attempt without taking ownership.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *);
#endif

#ifdef SLIC3R_TEST_PLUGIN_ABI3
struct Abi3PluginVtable {
    const char *(*get_id)(void *);
    const char *(*get_name)(void *);
    const char *(*get_description)(void *);
    const char *(*get_exclusive_group)(void *);
    const char *(*get_exclusive_group_label)(void *);
    const char *(*get_exclusive_group_tooltip)(void *);
    slicing_step_t (*get_step)(void *);
    const_strings_t (*get_dependencies)(void *);
    int32_t (*get_priority)(void *);
    plugin_used_config_keys_fn used_config_keys;
    plugin_defined_config_keys_fn defined_config_keys;
    void (*initialize)(void *, storage_handle *);
    plugin_setup_fn setup;
    plugin_setup_run_fn setup_run;
    plugin_run_fn run;
    plugin_clear_fn clear;
};
static_assert(offsetof(Abi3PluginVtable, get_id) == 0, "ABI3 has no size prefix.");
#endif

#if defined(SLIC3R_TEST_PLUGIN_BAD_API_MAJOR) || defined(SLIC3R_TEST_PLUGIN_BAD_API_MINOR) || \
    defined(SLIC3R_TEST_PLUGIN_SHORT_API_TABLE) || defined(SLIC3R_TEST_PLUGIN_LONG_API_TABLE) || \
    defined(SLIC3R_TEST_PLUGIN_UNKNOWN_API) || defined(SLIC3R_TEST_PLUGIN_ZERO_API_TABLE) || \
    defined(SLIC3R_TEST_PLUGIN_MISSING_VTABLE) || defined(SLIC3R_TEST_PLUGIN_INVALID_API_SIZE) || \
    defined(SLIC3R_TEST_PLUGIN_CHANGING_API_SIZE) || defined(SLIC3R_TEST_PLUGIN_ABI3) || \
    defined(SLIC3R_TEST_PLUGIN_OLD_SPEED_FAN)
extern "C" SLIC3R_PLUGIN_API int32_t slic3r_plugin_abi_version(
    slic3r_major_minor_version *requirements)
{
    uint32_t count = SLIC3R_PLUGIN_API_COUNT;
#ifdef SLIC3R_TEST_PLUGIN_SHORT_API_TABLE
    count = 1;
#endif
#if defined(SLIC3R_TEST_PLUGIN_LONG_API_TABLE) || defined(SLIC3R_TEST_PLUGIN_UNKNOWN_API)
    count = SLIC3R_PLUGIN_API_COUNT + 1;
#endif
#ifdef SLIC3R_TEST_PLUGIN_INVALID_API_SIZE
    return 0;
#endif
    if (requirements == nullptr)
        return int32_t(count);

    for (uint32_t idx = 0; idx < count; ++idx) {
        requirements[idx].major = 0;
        requirements[idx].minor = 0;
    }

#if !defined(SLIC3R_TEST_PLUGIN_ZERO_API_TABLE) && !defined(SLIC3R_TEST_PLUGIN_MISSING_VTABLE)
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].major = SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR;
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].minor = SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR;
#endif
#ifdef SLIC3R_TEST_PLUGIN_MISSING_VTABLE
    requirements[SLIC3R_PLUGIN_API_PLUGIN].major = SLIC3R_PLUGIN_API_PLUGIN_MAJOR;
#endif
#ifdef SLIC3R_TEST_PLUGIN_UNKNOWN_API
    requirements[SLIC3R_PLUGIN_API_COUNT].major = 1;
#endif
#ifdef SLIC3R_TEST_PLUGIN_BAD_API_MAJOR
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].major = SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR + 1;
#endif
#ifdef SLIC3R_TEST_PLUGIN_ABI3
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].major = 3;
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].minor = 0;
#endif
#ifdef SLIC3R_TEST_PLUGIN_BAD_API_MINOR
    requirements[SLIC3R_PLUGIN_API_PLUGIN_TYPES].minor = SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR + 1;
#endif
#ifdef SLIC3R_TEST_PLUGIN_OLD_SPEED_FAN
    // Actual former Speed layout, not current-version-plus-one.
    requirements[SLIC3R_PLUGIN_API_EXTRUSION_PROPERTY].major = 3;
    requirements[SLIC3R_PLUGIN_API_EXTRUSION_PROPERTY].minor = 0;
#endif
#ifdef SLIC3R_TEST_PLUGIN_CHANGING_API_SIZE
    return int32_t(count - 1);
#endif
    return int32_t(count);
}
#elif !defined(SLIC3R_TEST_PLUGIN_MISSING_ABI)
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
#endif
#endif // SLIC3R_TEST_PLUGIN_LEGACY_ABI

#ifndef SLIC3R_TEST_PLUGIN_MISSING_REGISTER
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *)
{
#ifdef SLIC3R_TEST_PLUGIN_THROW_REGISTER
    throw std::runtime_error("Registration fixture failure.");
#endif
}
#endif
