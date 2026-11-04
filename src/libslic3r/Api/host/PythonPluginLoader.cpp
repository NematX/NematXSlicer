///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* This library loads Python plugins and exposes their objects to the slicer.
   The host is the slicer application. This native library lets it call Python
   objects as plugins through a C ABI, the binary calling interface shared with C.

   The exported register_plugin() calls load_python_plugins(). It loads bundled
   scripts and scans neighboring installed package directories for description.ini,
   a compatible version.ini and a Python entry file, without a native plugin library.
   PythonPackageLoadScope identifies the package directory while
   load_python_plugin_from_path() loads its script under a unique module name.

   A script's register_plugin(api) returns Python objects. register_python_plugin_object()
   creates a PythonPlugin adapter holding its Python reference and C-callable
   functions. s_python_plugins owns it before registration can expose its address,
   including failure paths. Null or None entries in a returned sequence are skipped.
   Activation later decides whether a registered object runs.

   An exclusive group is a set of alternatives for one slicing stage. The adapter
   registers explicit selector descriptions, settings choosing those alternatives,
   through register_python_group_definitions() after introducing each object.
   initialize_bridge() and execution functions acquire Python's interpreter lock.
   Initialization and cleanup return stored diagnostic ids on failure.
   send_python_log() uses orchestrator_send_log() after resolving application
   functions. The function comments describe errors during loading and execution.

   Call graph:
     register_plugin
       load_python_plugins
         load_python_plugin_from_path
           call_python_register
             register_python_plugin_result
               register_python_plugin_object
                 register_python_group_definitions
     initialize_bridge
       call_python_void */

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <dlfcn.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#else
#include <unistd.h>
#endif
#endif

#if defined(_MSC_VER) && defined(_DEBUG)
#define SLIC3R_RESTORE_DEBUG_MACRO
#undef _DEBUG
#endif
#include <Python.h>
#if defined(SLIC3R_RESTORE_DEBUG_MACRO)
#define _DEBUG
#undef SLIC3R_RESTORE_DEBUG_MACRO
#endif

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"

namespace {

using OrchestratorRegisterPluginFn = void (*)(orchestrator_handle *, plugin_instance);
using OrchestratorRegisterPluginFromPackageFn = void (*)(orchestrator_handle *, plugin_instance, const char *);
using OrchestratorIsPluginRegisteredFn = int32_t (*)(orchestrator_handle *, const char *);
using OrchestratorSendLogFn = void (*)(orchestrator_handle *, raw_plugin_log_level, const char *);
using OrchestratorRegisterExclusiveGroupFn = int32_t (*)(orchestrator_handle *, slicing_step_t, const raw_exclusive_group_def *);
using OrchestratorBeginPackageLoadFn = void (*)(orchestrator_handle *, const char *);
using OrchestratorReportPackageErrorFn = void (*)(orchestrator_handle *, const char *,
                                                  raw_plugin_package_load_error_code, const char *);
using OrchestratorFinishPackageLoadFn = void (*)(orchestrator_handle *, const char *);

using OrchestratorSetErrorMessageFn = int32_t (*)(orchestrator_handle *orchestrator, const char *message);

struct OrchestratorFunctions
{
    OrchestratorRegisterPluginFn register_plugin = nullptr;
    OrchestratorRegisterPluginFromPackageFn register_package_plugin = nullptr;
    OrchestratorIsPluginRegisteredFn is_plugin_registered = nullptr;
    OrchestratorSendLogFn send_log = nullptr;
    // Python's group-description method calls this same application function.
    OrchestratorRegisterExclusiveGroupFn register_exclusive_group = nullptr;
    OrchestratorBeginPackageLoadFn begin_package_load = nullptr;
    OrchestratorReportPackageErrorFn report_package_error = nullptr;
    OrchestratorFinishPackageLoadFn finish_package_load = nullptr;
    // Host hook used to turn Python failures into retrievable diagnostic ids.
    OrchestratorSetErrorMessageFn set_error_message = nullptr;
};

OrchestratorSendLogFn s_send_log = nullptr;
orchestrator_handle *s_log_orchestrator = nullptr;

// Load bundled and compatible installed Python packages through borrowed application functions, keeping registered adapters alive.
// Missing loader location logs and returns; runtime/API failures throw, while package failures become diagnostics.
void load_python_plugins(orchestrator_handle *orchestrator);
// Find application functions from the borrowed library path and keep loaded libraries alive; missing functions log and return false.
// Slicing data is unchanged, and allocations may throw.
bool resolve_registration_functions(const boost::filesystem::path &host_library_path, OrchestratorFunctions &functions);
// Import a bundled module by its filename stem and register its returned objects; import failures become package diagnostics.
// Inputs stay borrowed, and the temporary module reference is released after registration.
void load_python_plugin(const boost::filesystem::path &plugin_path, PyObject *api,
    const OrchestratorFunctions &functions, orchestrator_handle *orchestrator, const boost::filesystem::path &package_root);
// Load an external package by absolute path under a unique module name so packages sharing plugin.py do not reuse cached modules.
void load_python_plugin_from_path(const boost::filesystem::path &plugin_path, PyObject *api,
    const OrchestratorFunctions &functions, orchestrator_handle *orchestrator, const boost::filesystem::path &package_root);
// Append the borrowed path to Python's search list while holding its interpreter lock; log full errors without retaining caller data.
void append_python_path(const boost::filesystem::path &path);
// Call an optional Python method; missing attributes succeed, and invalid or failing methods return false.
// Copy exception text to the optional borrowed failure output instead of logging it; release temporary references without taking ownership.
bool call_python_void(PyObject *object, const char *method_name, PyObject *args, const std::string &plugin_id,
    const char *fallback_method_name = nullptr, std::string *failure = nullptr);

// Send borrowed UTF-8 text to application logging when available, otherwise stderr; message storage is not retained.
// Package state is unchanged, and the C logging function contains logging failures.
void send_python_log(raw_plugin_log_level severity, const std::string &message);

void send_python_log(raw_plugin_log_level severity, const std::string &message)
{
    if (s_send_log != nullptr) s_send_log(s_log_orchestrator, severity, message.c_str());
    else std::fprintf(stderr, "Python loader: %s\n", message.c_str());
}

class PythonPackageLoadScope
{
public:
    // Own the package path and open its loading record through borrowed application functions.
    PythonPackageLoadScope(const OrchestratorFunctions &functions, orchestrator_handle *host, std::string root)
        : m_functions(functions), m_host(host), m_root(std::move(root))
    {
        m_functions.begin_package_load(m_host, m_root.c_str());
    }
    // Finish the package's loading record before releasing the path. The borrowed application functions must remain alive.
    ~PythonPackageLoadScope()
    {
        m_functions.finish_package_load(m_host, m_root.c_str());
    }
    PythonPackageLoadScope(const PythonPackageLoadScope &) = delete;
    PythonPackageLoadScope &operator=(const PythonPackageLoadScope &) = delete;
private:
    const OrchestratorFunctions &m_functions;
    orchestrator_handle *m_host;
    std::string m_root;
};
// Call a borrowed module's register_plugin(api) and register returned objects; missing, invalid or failed entry functions become diagnostics.
// Inputs stay borrowed, and temporary Python references are released on the handled paths.
void call_python_register(PyObject *module, PyObject *api, const OrchestratorFunctions &functions,
    orchestrator_handle *orchestrator, const boost::filesystem::path &plugin_path, const boost::filesystem::path &package_root);
// Register returned objects and group descriptions through the borrowed API and package directory; the loader keeps adapters alive.
// Temporary sequence references are released on failure; errors reach package diagnostics without changing slicing data.
void register_python_plugin_result(PyObject *result, const OrchestratorFunctions &functions,
    orchestrator_handle *orchestrator, const boost::filesystem::path &package_root, PyObject *api);
// Own an adapter for the borrowed Python object before registration exposes it, then publish its group descriptions.
// Adapters survive failed attempts until shutdown; slicing data is unchanged, and registration errors propagate.
void register_python_plugin_object(PyObject *plugin_object, const OrchestratorFunctions &functions,
    orchestrator_handle *orchestrator, const boost::filesystem::path &package_root, PyObject *api);

std::vector<std::unique_ptr<class PythonPlugin>> s_python_plugins;

// Consume the active Python exception and format its complete traceback. The
// returned string is safe to retain after the GIL is released.
std::string take_python_error();
// Read a signed 32-bit group priority, using 100 when absent; invalid values throw an error naming the plugin.
// The caller retains the object and holds Python's interpreter lock; temporary references are released.
int32_t group_priority_attribute(PyObject *object, const std::string &plugin_id);
// Publish description dictionaries through the Python API's generated C structure after registering their object, with the interpreter lock held.
// Inputs remain caller-owned; temporary references are released, and Python failures become C++ errors.
void register_python_group_definitions(PyObject *api, PyObject *plugin, const char *id, slicing_step_t step);

// Register one failure message through the host hook and return its id,
// or PLUGIN_ERROR_GENERAL when no hook is available.
int32_t store_python_error_message(OrchestratorSetErrorMessageFn set_error_message,
                                   orchestrator_handle *orchestrator,
                                   const char *message) noexcept
{
    if (set_error_message == nullptr || orchestrator == nullptr)
        return PLUGIN_ERROR_GENERAL;
    return set_error_message(orchestrator, message);
}

// Format the active Python exception and register it as a diagnostic id. Falls
// back to the supplied generic text when the exception carries none.
int32_t take_python_error_message(OrchestratorSetErrorMessageFn set_error_message,
                                orchestrator_handle *orchestrator,
                                const char *fallback) noexcept;

// Publish a package-specific Python failure when loading an external script.
void report_python_package_error(const OrchestratorFunctions &functions,
                                 orchestrator_handle *orchestrator,
                                 const boost::filesystem::path &package_root,
                                 raw_plugin_package_load_error_code code,
                                 const std::string &detail);

struct PythonUsedConfigKey
{
    std::string key;
    raw_config_option_type type = RAW_CO_NONE;
    raw_container_type container_type = RAW_CONTAINER_TYPE_NONE;
    raw_option_preset_type option_preset_type = RAW_PRESET_TYPE_NONE;
};

bool ensure_python_initialized_for_plugins()
{
    if (Py_IsInitialized())
        return true;

    Py_Initialize();
    if (!Py_IsInitialized())
        return false;

    // Py_Initialize() leaves the startup thread holding the Python GIL. Plugin
    // callbacks run later from slicing/background threads and use
    // PyGILState_Ensure() before entering Python. If the startup thread keeps
    // the initial GIL, those callbacks block forever. Release it once here; all
    // later Python entry points explicitly acquire and release the GIL.
    PyEval_SaveThread();
    return true;
}

boost::filesystem::path current_module_path()
{
#ifdef _WIN32
    HMODULE module = NULL;
    const DWORD flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
    if (GetModuleHandleExW(flags, reinterpret_cast<LPCWSTR>(&current_module_path), &module) == 0 || module == NULL)
        return {};

    std::vector<wchar_t> buffer(MAX_PATH);
    DWORD length = 0;
    for (;;) {
        length = GetModuleFileNameW(module, buffer.data(), DWORD(buffer.size()));
        if (length == 0)
            return {};
        if (length < buffer.size() - 1)
            break;
        buffer.resize(buffer.size() * 2);
    }
    return boost::filesystem::path(std::wstring(buffer.data(), length));
#else
    Dl_info info = {};
    if (dladdr(reinterpret_cast<void *>(&current_module_path), &info) == 0 || info.dli_fname == nullptr)
        return {};
    return boost::filesystem::path(info.dli_fname);
#endif
}

boost::filesystem::path current_executable_path()
{
#ifdef _WIN32
    std::vector<wchar_t> buffer(MAX_PATH);
    DWORD length = 0;
    for (;;) {
        length = GetModuleFileNameW(NULL, buffer.data(), DWORD(buffer.size()));
        if (length == 0)
            return {};
        if (length < buffer.size() - 1)
            break;
        buffer.resize(buffer.size() * 2);
    }
    return boost::filesystem::path(std::wstring(buffer.data(), length));
#else
#ifdef __APPLE__
    uint32_t length = 0;
    _NSGetExecutablePath(nullptr, &length);
    std::vector<char> buffer(length);
    if (_NSGetExecutablePath(buffer.data(), &length) != 0)
        return {};
    return boost::filesystem::canonical(boost::filesystem::path(buffer.data()));
#else
    std::vector<char> buffer(1024);
    for (;;) {
        const ssize_t length = readlink("/proc/self/exe", buffer.data(), buffer.size());
        if (length < 0)
            return {};
        if (size_t(length) < buffer.size())
            return boost::filesystem::path(std::string(buffer.data(), size_t(length)));
        buffer.resize(buffer.size() * 2);
    }
#endif
#endif
}

boost::filesystem::path host_library_path()
{
    const boost::filesystem::path executable = current_executable_path();
    if (executable.empty())
        return {};

#ifdef _WIN32
    /*
    The installed Python runtime lives under the user data directory, not next
    to the application binaries. The host API is exported by Slic3r.dll beside
    the executable, so it cannot be located relative to the plugin package.

    Plugin tests deliberately omit Slic3r.dll and export the API from their
    executable instead. Returning the executable is therefore the required
    fallback rather than an error.
    */
    const boost::filesystem::path host_library = executable.parent_path() / "Slic3r.dll";
    if (boost::filesystem::is_regular_file(host_library))
        return host_library;
#endif

    return executable;
}

std::string take_python_error()
{
    PyObject *exception_type = nullptr;
    PyObject *exception_value = nullptr;
    PyObject *exception_traceback = nullptr;
    PyErr_Fetch(&exception_type, &exception_value, &exception_traceback);
    PyErr_NormalizeException(&exception_type, &exception_value, &exception_traceback);

    std::string result;
    PyObject *traceback_module = PyImport_ImportModule("traceback");
    PyObject *format_exception = traceback_module == nullptr ? nullptr :
        PyObject_GetAttrString(traceback_module, "format_exception");
    PyObject *formatted = format_exception == nullptr ? nullptr :
        PyObject_CallFunctionObjArgs(format_exception,
                                     exception_type == nullptr ? Py_None : exception_type,
                                     exception_value == nullptr ? Py_None : exception_value,
                                     exception_traceback == nullptr ? Py_None : exception_traceback,
                                     nullptr);
    PyObject *separator = formatted == nullptr ? nullptr : PyUnicode_FromString("");
    PyObject *joined = separator == nullptr ? nullptr : PyUnicode_Join(separator, formatted);
    if (joined != nullptr) {
        const char *utf8 = PyUnicode_AsUTF8(joined);
        if (utf8 != nullptr)
            result = utf8;
    }

    // Formatting may itself fail when the Python runtime is only partially
    // initialized. Fall back to the exception value rather than losing the
    // original reason entirely.
    if (result.empty() && exception_value != nullptr) {
        PyErr_Clear();
        PyObject *value_text = PyObject_Str(exception_value);
        if (value_text != nullptr) {
            const char *utf8 = PyUnicode_AsUTF8(value_text);
            if (utf8 != nullptr)
                result = utf8;
        }
        Py_XDECREF(value_text);
    }
    if (result.empty())
        result = "Unknown Python error.";

    Py_XDECREF(joined);
    Py_XDECREF(separator);
    Py_XDECREF(formatted);
    Py_XDECREF(format_exception);
    Py_XDECREF(traceback_module);
    Py_XDECREF(exception_traceback);
    Py_XDECREF(exception_value);
    Py_XDECREF(exception_type);
    PyErr_Clear();
    return result;
}

int32_t take_python_error_message(OrchestratorSetErrorMessageFn set_error_message,
                                orchestrator_handle *orchestrator,
                                const char *fallback) noexcept
{
    PyObject *exception_type = nullptr;
    PyObject *exception_value = nullptr;
    PyObject *exception_traceback = nullptr;
    PyErr_Fetch(&exception_type, &exception_value, &exception_traceback);
    PyErr_NormalizeException(&exception_type, &exception_value, &exception_traceback);
    PyErr_Clear();
    PyObject *text = exception_value == nullptr ? nullptr : PyObject_Str(exception_value);
    const char *utf8 = text == nullptr ? nullptr : PyUnicode_AsUTF8(text);
    const char *message = utf8 != nullptr && *utf8 != '\0' ? utf8 : fallback;
    const int32_t id = store_python_error_message(set_error_message, orchestrator, message);
    Py_XDECREF(text);
    Py_XDECREF(exception_traceback);
    Py_XDECREF(exception_value);
    Py_XDECREF(exception_type);
    PyErr_Clear();
    return id;
}

void report_python_package_error(const OrchestratorFunctions &functions,
                                 orchestrator_handle *orchestrator,
                                 const boost::filesystem::path &package_root,
                                 raw_plugin_package_load_error_code code,
                                 const std::string &detail)
{
    if (package_root.empty() || functions.report_package_error == nullptr)
        return;
    functions.report_package_error(orchestrator, package_root.string().c_str(), code, detail.c_str());
}

PyObject *python_path_from_boost(const boost::filesystem::path &path)
{
#ifdef _WIN32
    const std::wstring native = path.wstring();
    return PyUnicode_FromWideChar(native.c_str(), Py_ssize_t(native.size()));
#else
    const std::string native = path.string();
    return PyUnicode_DecodeFSDefault(native.c_str());
#endif
}

void append_python_path(const boost::filesystem::path &path)
{
    PyObject *sys_path = PySys_GetObject("path");
    if (sys_path == nullptr || !PyList_Check(sys_path))
        return;

    PyObject *py_path = python_path_from_boost(path);
    if (py_path == nullptr) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot encode Python search path: " + take_python_error());
        return;
    }

    const int contains = PySequence_Contains(sys_path, py_path);
    if (contains == 0 && PyList_Append(sys_path, py_path) != 0)
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot append Python search path: " + take_python_error());
    else if (contains < 0)
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot inspect Python search path: " + take_python_error());

    Py_DECREF(py_path);
}

std::string py_object_to_string(PyObject *object)
{
    if (object == nullptr)
        return {};

    PyObject *bytes = PyUnicode_AsUTF8String(object);
    if (bytes == nullptr) {
        PyErr_Clear();
        return {};
    }

    const char *text = PyBytes_AsString(bytes);
    std::string out = text != nullptr ? text : "";
    Py_DECREF(bytes);
    return out;
}

std::string string_attribute(PyObject *object, const char *name, const char *fallback = "")
{
    PyObject *attr = PyObject_GetAttrString(object, name);
    if (attr == nullptr) {
        PyErr_Clear();
        return fallback != nullptr ? fallback : "";
    }

    std::string out = py_object_to_string(attr);
    Py_DECREF(attr);
    return out.empty() && fallback != nullptr ? fallback : out;
}

int32_t group_priority_attribute(PyObject *object, const std::string &plugin_id)
{
    PyObject *attribute = PyObject_GetAttrString(object, "exclusive_group_priority");
    if (attribute == nullptr) {
        if (PyErr_ExceptionMatches(PyExc_AttributeError)) { PyErr_Clear(); return 100; }
        throw std::runtime_error("Cannot read group preference for Python plugin '" + plugin_id + "': " + take_python_error());
    }
    const bool integer = PyLong_Check(attribute) != 0;
    const long long value = integer ? PyLong_AsLongLong(attribute) : 0;
    const bool failed = PyErr_Occurred() != nullptr;
    Py_DECREF(attribute);
    if (!integer || failed || value < (std::numeric_limits<int32_t>::min)() || value > (std::numeric_limits<int32_t>::max)()) {
        PyErr_Clear();
        throw std::invalid_argument("Python plugin '" + plugin_id + "' exclusive_group_priority must be a signed 32-bit integer.");
    }
    return static_cast<int32_t>(value);
}

int int_attribute(PyObject *object, const char *name, int fallback)
{
    PyObject *attr = PyObject_GetAttrString(object, name);
    if (attr == nullptr) {
        PyErr_Clear();
        return fallback;
    }

    const long value = PyLong_AsLong(attr);
    if (PyErr_Occurred()) {
        PyErr_Clear();
        Py_DECREF(attr);
        return fallback;
    }

    Py_DECREF(attr);
    return int(value);
}

bool py_object_to_long(PyObject *object, long &out)
{
    if (object == nullptr)
        return false;

    const long value = PyLong_AsLong(object);
    if (PyErr_Occurred()) {
        PyErr_Clear();
        return false;
    }

    out = value;
    return true;
}

std::vector<std::string> string_list_attribute(PyObject *object, const char *name)
{
    std::vector<std::string> out;
    PyObject *attr = PyObject_GetAttrString(object, name);
    if (attr == nullptr) {
        PyErr_Clear();
        return out;
    }

    PyObject *iterator = PyObject_GetIter(attr);
    Py_DECREF(attr);
    if (iterator == nullptr) {
        PyErr_Clear();
        return out;
    }

    for (;;) {
        PyObject *item = PyIter_Next(iterator);
        if (item == nullptr)
            break;
        out.emplace_back(py_object_to_string(item));
        Py_DECREF(item);
    }
    if (PyErr_Occurred())
        PyErr_Clear();

    Py_DECREF(iterator);
    return out;
}

PythonUsedConfigKey used_config_key_from_sequence(PyObject *item)
{
    PythonUsedConfigKey out;
    const Py_ssize_t size = PySequence_Size(item);
    if (size < 2) {
        PyErr_Clear();
        return out;
    }

    PyObject *key = PySequence_GetItem(item, 0);
    PyObject *type = PySequence_GetItem(item, 1);
    PyObject *container = size > 2 ? PySequence_GetItem(item, 2) : nullptr;
    PyObject *preset = size > 3 ? PySequence_GetItem(item, 3) : nullptr;

    out.key = py_object_to_string(key);
    long value = 0;
    if (py_object_to_long(type, value))
        out.type = raw_config_option_type(value);
    if (py_object_to_long(container, value))
        out.container_type = raw_container_type(value);
    if (py_object_to_long(preset, value))
        out.option_preset_type = raw_option_preset_type(value);

    Py_XDECREF(key);
    Py_XDECREF(type);
    Py_XDECREF(container);
    Py_XDECREF(preset);
    return out;
}

PythonUsedConfigKey used_config_key_from_dict(PyObject *item)
{
    PythonUsedConfigKey out;
    PyObject *key = PyDict_GetItemString(item, "key");
    PyObject *type = PyDict_GetItemString(item, "type");
    PyObject *container = PyDict_GetItemString(item, "container_type");
    PyObject *preset = PyDict_GetItemString(item, "option_preset_type");

    out.key = py_object_to_string(key);
    long value = 0;
    if (py_object_to_long(type, value))
        out.type = raw_config_option_type(value);
    if (py_object_to_long(container, value))
        out.container_type = raw_container_type(value);
    if (py_object_to_long(preset, value))
        out.option_preset_type = raw_option_preset_type(value);
    return out;
}

std::vector<PythonUsedConfigKey> used_config_key_list_attribute(PyObject *object, const char *name)
{
    std::vector<PythonUsedConfigKey> out;
    PyObject *attr = PyObject_GetAttrString(object, name);
    if (attr == nullptr) {
        PyErr_Clear();
        return out;
    }

    PyObject *iterator = PyObject_GetIter(attr);
    Py_DECREF(attr);
    if (iterator == nullptr) {
        PyErr_Clear();
        return out;
    }

    for (;;) {
        PyObject *item = PyIter_Next(iterator);
        if (item == nullptr)
            break;

        if (PyDict_Check(item)) {
            out.emplace_back(used_config_key_from_dict(item));
        } else if (PySequence_Check(item) && !PyUnicode_Check(item)) {
            out.emplace_back(used_config_key_from_sequence(item));
        } else {
            // Old string-only declarations are not enough for the typed ABI.
            // Keep the key so the host can report the offending plugin/key.
            PythonUsedConfigKey key;
            key.key = py_object_to_string(item);
            out.emplace_back(std::move(key));
        }
        Py_DECREF(item);
    }
    if (PyErr_Occurred())
        PyErr_Clear();

    Py_DECREF(iterator);
    return out;
}

PyObject *pointer_to_python_uint(const void *ptr)
{
    return PyLong_FromUnsignedLongLong(static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(ptr)));
}

bool call_python_void(PyObject *object, const char *method_name, PyObject *args, const std::string &plugin_id,
                      const char *fallback_method_name, std::string *failure)
{
    PyObject *method = PyObject_GetAttrString(object, method_name);
    if (method == nullptr && fallback_method_name != nullptr) {
        // PluginBase supplies protected bridge methods. The fallback preserves
        // older duck-typed Python plugins that implemented only public hooks.
        PyErr_Clear();
        method = PyObject_GetAttrString(object, fallback_method_name);
    }
    if (method == nullptr) {
        PyErr_Clear();
        return true;
    }

    if (!PyCallable_Check(method)) {
        if (failure != nullptr) *failure = std::string(method_name) + " attribute is not callable";
        send_python_log(RAW_PLUGIN_LOG_WARNING, "Python plugin '" + plugin_id + "' attribute " + method_name + " is not callable.");
        Py_DECREF(method);
        return false;
    }

    PyObject *result = PyObject_CallObject(method, args);
    if (result == nullptr) {
        if (failure != nullptr) *failure = take_python_error();
        else send_python_log(RAW_PLUGIN_LOG_ERROR, "Python plugin '" + plugin_id + "' failed in " + method_name + "(): " + take_python_error());
        Py_DECREF(method);
        return false;
    }

    Py_DECREF(result);
    Py_DECREF(method);
    return true;
}

/*
Report whether the Python plugin exposes a callable clear hook. A declared but
non-callable attribute is a plugin error, so it throws instead of silently
disabling per-Print cleanup.
*/
bool python_plugin_has_clear(PyObject *plugin, const std::string &plugin_id)
{
    PyObject *method = PyObject_GetAttrString(plugin, "clear");
    if (method == nullptr) {
        PyErr_Clear();
        return false;
    }

    const bool callable = PyCallable_Check(method) != 0;
    Py_DECREF(method);
    if (!callable)
        throw std::runtime_error("Python plugin '" + plugin_id + "' clear attribute is not callable.");
    return true;
}

class PythonPlugin
{
public:
    // Copy descriptions while Python's interpreter lock is held, retaining the object after successful construction.
    // Slicing data is unchanged. Invalid attributes and allocation failures throw before registration exposes the object.
    PythonPlugin(PyObject *plugin,
                 const OrchestratorSetErrorMessageFn set_error_message,
                 orchestrator_handle *orchestrator)
        : m_plugin(plugin)
        , m_set_error_message(set_error_message)
        , m_orchestrator(orchestrator)
        , m_id(string_attribute(plugin, "plugin_id", "python.unnamed"))
        , m_name(string_attribute(plugin, "name", m_id.c_str()))
        , m_description(string_attribute(plugin, "description", ""))
        , m_exclusive_group(string_attribute(plugin, "exclusive_group", ""))
        , m_step(slicing_step_t(int_attribute(plugin, "step", STEP_POST_SLICING)))
        , m_priority(int_attribute(plugin, "priority", 0))
        , m_exclusive_group_priority(group_priority_attribute(plugin, m_id))
        , m_dependencies(string_list_attribute(plugin, "dependencies"))
        , m_used_config_keys(used_config_key_list_attribute(plugin, "used_config_keys"))
        , m_defined_config_keys(string_list_attribute(plugin, "defined_config_keys"))
        , m_has_clear(python_plugin_has_clear(plugin, m_id))
    {
        for (const std::string &dependency : m_dependencies)
            m_dependency_ptrs.push_back(dependency.c_str());
        for (const PythonUsedConfigKey &key : m_used_config_keys) {
            raw_used_config_key view = {};
            view.key = key.key.c_str();
            view.type = key.type;
            view.container_type = key.container_type;
            view.option_preset_type = key.option_preset_type;
            m_used_config_key_views.push_back(view);
        }
        for (const std::string &key : m_defined_config_keys)
            m_defined_config_key_ptrs.push_back(key.c_str());
        Py_INCREF(m_plugin);
    }

    ~PythonPlugin()
    {
        PyGILState_STATE gil_state = PyGILState_Ensure();
        Py_DECREF(m_plugin);
        PyGILState_Release(gil_state);
    }

    plugin_instance c_instance()
    {
        plugin_instance instance = {};
        instance.ctx = this;
        m_instance_vtable = vtable();
        // A null callback must remain distinguishable from a Python method that participates in cleanup.
        m_instance_vtable.clear = m_has_clear ? &PythonPlugin::clear_bridge : nullptr;
        instance.vt = &m_instance_vtable;
        return instance;
    }

private:
    static const char *get_id_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_id.c_str();
    }

    static const char *get_name_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_name.c_str();
    }

    static const char *get_description_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_description.c_str();
    }

    static const char *get_exclusive_group_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_exclusive_group.c_str();
    }

    static slicing_step_t get_step_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_step;
    }

    static const_strings_t get_dependencies_bridge(void *plugin_ctx)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        const_strings_t out = {};
        out.items = plugin->m_dependency_ptrs.empty() ? nullptr : plugin->m_dependency_ptrs.data();
        out.size = uint32_t(plugin->m_dependency_ptrs.size());
        return out;
    }

    static int32_t get_priority_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_priority;
    }

    static int32_t used_config_keys_bridge(void *plugin_ctx, raw_used_config_key *keys)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        if (keys != nullptr) {
            for (size_t i = 0; i < plugin->m_used_config_key_views.size(); ++i)
                keys[i] = plugin->m_used_config_key_views[i];
        }
        return int32_t(plugin->m_used_config_key_views.size());
    }

    static int32_t defined_config_keys_bridge(void *plugin_ctx, const char **keys)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        if (keys != nullptr) {
            for (size_t i = 0; i < plugin->m_defined_config_key_ptrs.size(); ++i)
                keys[i] = plugin->m_defined_config_key_ptrs[i];
        }
        return int32_t(plugin->m_defined_config_key_ptrs.size());
    }

    // Initialize under Python's interpreter lock using borrowed storage. Return success or an error id without exceptions crossing C.
    static int32_t initialize_bridge(void *plugin_ctx, storage_handle *storage)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        PyGILState_STATE gil_state = PyGILState_Ensure();
        PyObject *arg = nullptr;
        PyObject *args = nullptr;
        int32_t result = PLUGIN_ERROR_GENERAL;
        try {
            arg = pointer_to_python_uint(storage);
            args = arg != nullptr ? PyTuple_Pack(1, arg) : nullptr;
            std::string failure;
            const bool initialized = args != nullptr &&
                call_python_void(plugin->m_plugin, "initialize", args, plugin->m_id, nullptr, &failure);
            if (args == nullptr) failure = take_python_error();
            result = initialized ? PLUGIN_ERROR_OK :
                plugin->m_set_error_message(plugin->m_orchestrator, failure.c_str());
        } catch (...) { PyErr_Clear(); }
        Py_XDECREF(args);
        Py_XDECREF(arg);
        PyGILState_Release(gil_state);
        return result;
    }

    // Borrow plugin/context to call Python setup under its interpreter lock; release arguments and log complete Python failures.
    static void setup_bridge(void *plugin_ctx, const plugin_run_context *run_ctx, uint32_t run_count)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        PyGILState_STATE gil_state = PyGILState_Ensure();
        PyObject *py_run_ctx = pointer_to_python_uint(run_ctx);
        PyObject *py_run_count = PyLong_FromUnsignedLong(run_count);
        PyObject *args = py_run_ctx != nullptr && py_run_count != nullptr ? PyTuple_Pack(2, py_run_ctx, py_run_count) : nullptr;
        Py_XDECREF(py_run_ctx);
        Py_XDECREF(py_run_count);
        if (args != nullptr) {
            // PluginBase owns the safety wrapper: it resets aggregate progress,
            // consumes cooperative cancellation and reports Python failures.
            call_python_void(plugin->m_plugin, "_slic3r_setup", args, plugin->m_id, "setup");
            Py_DECREF(args);
        } else {
            send_python_log(RAW_PLUGIN_LOG_ERROR, "Python plugin '" + plugin->m_id + "' cannot prepare setup arguments: " + take_python_error());
        }
        PyGILState_Release(gil_state);
    }

    // Borrow plugin/context to call Python setup_run under its interpreter lock; release arguments and log complete Python failures.
    static void setup_run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        PyGILState_STATE gil_state = PyGILState_Ensure();
        PyObject *arg = pointer_to_python_uint(run_ctx);
        PyObject *args = arg != nullptr ? PyTuple_Pack(1, arg) : nullptr;
        Py_XDECREF(arg);
        if (args != nullptr) {
            call_python_void(plugin->m_plugin, "_slic3r_setup_run", args, plugin->m_id, "setup_run");
            Py_DECREF(args);
        } else {
            send_python_log(RAW_PLUGIN_LOG_ERROR, "Python plugin '" + plugin->m_id + "' cannot prepare setup_run arguments: " + take_python_error());
        }
        PyGILState_Release(gil_state);
    }

    // Borrow plugin/context to call Python run under its interpreter lock; release arguments and log complete Python failures.
    static void run_bridge(void *plugin_ctx, const plugin_run_context *run_ctx)
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        PyGILState_STATE gil_state = PyGILState_Ensure();
        PyObject *arg = pointer_to_python_uint(run_ctx);
        PyObject *args = arg != nullptr ? PyTuple_Pack(1, arg) : nullptr;
        Py_XDECREF(arg);
        if (args != nullptr) {
            call_python_void(plugin->m_plugin, "_slic3r_run", args, plugin->m_id, "run");
            Py_DECREF(args);
        } else {
            send_python_log(RAW_PLUGIN_LOG_ERROR, "Python plugin '" + plugin->m_id + "' cannot prepare run arguments: " + take_python_error());
        }
        PyGILState_Release(gil_state);
    }

    static int32_t clear_bridge(void *plugin_ctx,
                                const print_handle *print,
                                storage_handle *plugin_storage) noexcept
    {
        PythonPlugin *plugin = static_cast<PythonPlugin *>(plugin_ctx);
        try {
            // Destruction runs in reverse declaration order: release Python references before releasing the GIL.
            struct GilGuard {
                PyGILState_STATE state = PyGILState_Ensure();
                ~GilGuard() { PyGILState_Release(state); }
            } gil;
            const std::unique_ptr<PyObject, decltype(&Py_DecRef)> py_print(
                pointer_to_python_uint(print), &Py_DecRef);
            const std::unique_ptr<PyObject, decltype(&Py_DecRef)> py_storage(
                pointer_to_python_uint(plugin_storage), &Py_DecRef);
            const std::unique_ptr<PyObject, decltype(&Py_DecRef)> args(
                py_print && py_storage ? PyTuple_Pack(2, py_print.get(), py_storage.get()) : nullptr,
                &Py_DecRef);
            if (args == nullptr)
                return take_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                               "Python plugin clear argument allocation failed");
            const std::unique_ptr<PyObject, decltype(&Py_DecRef)> method(
                PyObject_GetAttrString(plugin->m_plugin, "clear"), &Py_DecRef);
            if (method == nullptr)
                return take_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                               "Python plugin clear method lookup failed");
            if (!PyCallable_Check(method.get()))
                return store_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                                  "Python plugin clear attribute is not callable");
            const std::unique_ptr<PyObject, decltype(&Py_DecRef)> result(
                PyObject_CallObject(method.get(), args.get()), &Py_DecRef);
            if (result == nullptr)
                return take_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                               "Python plugin failed in clear()");

            // clear() returns PLUGIN_ERROR_OK, a registered diagnostic id, or
            // None for success. Anything else is a plugin-side contract error
            // the host must see as a failure.
            if (result.get() == Py_None)
                return PLUGIN_ERROR_OK;
            if (!PyLong_Check(result.get()) || PyBool_Check(result.get()))
                return store_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                                  "Python plugin clear() must return an int32 status or None");
            const long long status = PyLong_AsLongLong(result.get());
            if (PyErr_Occurred() || status < 0 || status > (std::numeric_limits<int32_t>::max)()) {
                PyErr_Clear();
                return store_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                                  "Python plugin clear() status is outside the int32 range");
            }
            return static_cast<int32_t>(status);
        } catch (const std::exception &error) {
            return store_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                              error.what());
        } catch (...) {
            return store_python_error_message(plugin->m_set_error_message, plugin->m_orchestrator,
                                              "Unknown C++ exception in Python plugin clear");
        }
    }

    // Return the borrowed object's copied preference. No Python calls, state changes, allocations or failures occur.
    static int32_t get_exclusive_group_priority_bridge(void *plugin_ctx)
    {
        return static_cast<PythonPlugin *>(plugin_ctx)->m_exclusive_group_priority;
    }

    // Borrow the function table, including preference and optional cleanup. No state changes or allocations occur.
    static const plugin_vtable &vtable()
    {
        static const plugin_vtable vt = {
            sizeof(plugin_vtable),
            &PythonPlugin::get_id_bridge,
            &PythonPlugin::get_name_bridge,
            &PythonPlugin::get_description_bridge,
            &PythonPlugin::get_exclusive_group_bridge,
            &PythonPlugin::get_step_bridge,
            &PythonPlugin::get_dependencies_bridge,
            &PythonPlugin::get_priority_bridge,
            &PythonPlugin::used_config_keys_bridge,
            &PythonPlugin::defined_config_keys_bridge,
            &PythonPlugin::initialize_bridge,
            &PythonPlugin::setup_bridge,
            &PythonPlugin::setup_run_bridge,
            &PythonPlugin::run_bridge,
            nullptr,
            &PythonPlugin::get_exclusive_group_priority_bridge
        };
        return vt;
    }

    PyObject *m_plugin = nullptr;
    std::string m_id;
    std::string m_name;
    std::string m_description;
    std::string m_exclusive_group;
    slicing_step_t m_step;
    int32_t m_priority = 0;
    int32_t m_exclusive_group_priority = 100;
    std::vector<std::string> m_dependencies;
    std::vector<const char *> m_dependency_ptrs;
    std::vector<PythonUsedConfigKey> m_used_config_keys;
    std::vector<raw_used_config_key> m_used_config_key_views;
    std::vector<std::string> m_defined_config_keys;
    std::vector<const char *> m_defined_config_key_ptrs;
    bool m_has_clear = false;
    // Host hook and Orchestrator owning the messages returned by clear().
    OrchestratorSetErrorMessageFn m_set_error_message = nullptr;
    orchestrator_handle *m_orchestrator = nullptr;
    plugin_vtable m_instance_vtable = {};
};

bool resolve_registration_functions(const boost::filesystem::path &host_library_path,
                                    OrchestratorFunctions &functions)
{
    functions = {};
#ifdef _WIN32
    static std::vector<HMODULE> loaded_modules;
    HMODULE module = LoadLibraryW(host_library_path.wstring().c_str());
    if (module == NULL) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot open host library '" + host_library_path.string() +
                        "' for Python plugin registration: error " + std::to_string(GetLastError()));
        return false;
    }
    loaded_modules.push_back(module);

    FARPROC farproc = GetProcAddress(module, "orchestrator_register_plugin");
    if (farproc == NULL) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export orchestrator_register_plugin.");
        return false;
    }
    FARPROC package_farproc = GetProcAddress(module, "orchestrator_register_plugin_from_package");
    if (package_farproc == NULL) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export orchestrator_register_plugin_from_package.");
        return false;
    }
    FARPROC begin_farproc = GetProcAddress(module, "orchestrator_begin_plugin_package_load");
    FARPROC report_farproc = GetProcAddress(module, "orchestrator_report_plugin_package_load_error");
    FARPROC finish_farproc = GetProcAddress(module, "orchestrator_finish_plugin_package_load");
    FARPROC error_message_farproc = GetProcAddress(module, "orchestrator_set_error_message");
    FARPROC group_farproc = GetProcAddress(module, "orchestrator_register_exclusive_group");
    FARPROC find_farproc = GetProcAddress(module, "orchestrator_is_plugin_registered");
    FARPROC log_farproc = GetProcAddress(module, "orchestrator_send_log");
    s_send_log = reinterpret_cast<OrchestratorSendLogFn>(log_farproc);
    if (begin_farproc == NULL || report_farproc == NULL || finish_farproc == NULL ||
        error_message_farproc == NULL || group_farproc == NULL || find_farproc == NULL || log_farproc == NULL) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export plugin package diagnostic functions.");
        return false;
    }
    functions.register_plugin = reinterpret_cast<OrchestratorRegisterPluginFn>(farproc);
    functions.register_package_plugin = reinterpret_cast<OrchestratorRegisterPluginFromPackageFn>(package_farproc);
    functions.begin_package_load = reinterpret_cast<OrchestratorBeginPackageLoadFn>(begin_farproc);
    functions.report_package_error = reinterpret_cast<OrchestratorReportPackageErrorFn>(report_farproc);
    functions.finish_package_load = reinterpret_cast<OrchestratorFinishPackageLoadFn>(finish_farproc);
    functions.set_error_message = reinterpret_cast<OrchestratorSetErrorMessageFn>(error_message_farproc);
    functions.register_exclusive_group = reinterpret_cast<OrchestratorRegisterExclusiveGroupFn>(group_farproc);
    functions.is_plugin_registered = reinterpret_cast<OrchestratorIsPluginRegisteredFn>(find_farproc);
    functions.send_log = reinterpret_cast<OrchestratorSendLogFn>(log_farproc);
#else
    void *symbol = dlsym(RTLD_DEFAULT, "orchestrator_register_plugin");
    void *package_symbol = dlsym(RTLD_DEFAULT, "orchestrator_register_plugin_from_package");
    void *begin_symbol = dlsym(RTLD_DEFAULT, "orchestrator_begin_plugin_package_load");
    void *report_symbol = dlsym(RTLD_DEFAULT, "orchestrator_report_plugin_package_load_error");
    void *finish_symbol = dlsym(RTLD_DEFAULT, "orchestrator_finish_plugin_package_load");
    void *error_message_symbol = dlsym(RTLD_DEFAULT, "orchestrator_set_error_message");
    void *group_symbol = dlsym(RTLD_DEFAULT, "orchestrator_register_exclusive_group");
    void *find_symbol = dlsym(RTLD_DEFAULT, "orchestrator_is_plugin_registered");
    void *log_symbol = dlsym(RTLD_DEFAULT, "orchestrator_send_log");
    s_send_log = reinterpret_cast<OrchestratorSendLogFn>(log_symbol);
    if (symbol != nullptr && package_symbol != nullptr && begin_symbol != nullptr &&
        report_symbol != nullptr && finish_symbol != nullptr && error_message_symbol != nullptr &&
        group_symbol != nullptr && find_symbol != nullptr && log_symbol != nullptr) {
        functions.register_plugin = reinterpret_cast<OrchestratorRegisterPluginFn>(symbol);
        functions.register_package_plugin = reinterpret_cast<OrchestratorRegisterPluginFromPackageFn>(package_symbol);
        functions.begin_package_load = reinterpret_cast<OrchestratorBeginPackageLoadFn>(begin_symbol);
        functions.report_package_error = reinterpret_cast<OrchestratorReportPackageErrorFn>(report_symbol);
        functions.finish_package_load = reinterpret_cast<OrchestratorFinishPackageLoadFn>(finish_symbol);
        functions.set_error_message = reinterpret_cast<OrchestratorSetErrorMessageFn>(error_message_symbol);
        functions.register_exclusive_group = reinterpret_cast<OrchestratorRegisterExclusiveGroupFn>(group_symbol);
        functions.is_plugin_registered = reinterpret_cast<OrchestratorIsPluginRegisteredFn>(find_symbol);
        functions.send_log = reinterpret_cast<OrchestratorSendLogFn>(log_symbol);
        return true;
    }

    void *module = dlopen(host_library_path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (module == nullptr) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot open host library '" + host_library_path.string() + "' for Python plugin registration: " + dlerror());
        return false;
    }

    symbol = dlsym(module, "orchestrator_register_plugin");
    if (symbol == nullptr) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export orchestrator_register_plugin: " + dlerror());
        return false;
    }
    package_symbol = dlsym(module, "orchestrator_register_plugin_from_package");
    if (package_symbol == nullptr) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export orchestrator_register_plugin_from_package: " + dlerror());
        return false;
    }
    begin_symbol = dlsym(module, "orchestrator_begin_plugin_package_load");
    report_symbol = dlsym(module, "orchestrator_report_plugin_package_load_error");
    finish_symbol = dlsym(module, "orchestrator_finish_plugin_package_load");
    error_message_symbol = dlsym(module, "orchestrator_set_error_message");
    group_symbol = dlsym(module, "orchestrator_register_exclusive_group");
    find_symbol = dlsym(module, "orchestrator_is_plugin_registered");
    log_symbol = dlsym(module, "orchestrator_send_log");
    s_send_log = reinterpret_cast<OrchestratorSendLogFn>(log_symbol);
    if (begin_symbol == nullptr || report_symbol == nullptr || finish_symbol == nullptr ||
        error_message_symbol == nullptr || group_symbol == nullptr || find_symbol == nullptr || log_symbol == nullptr) {
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Host library '" + host_library_path.string() + "' does not export plugin package diagnostic functions.");
        return false;
    }
    functions.register_plugin = reinterpret_cast<OrchestratorRegisterPluginFn>(symbol);
    functions.register_package_plugin = reinterpret_cast<OrchestratorRegisterPluginFromPackageFn>(package_symbol);
    functions.begin_package_load = reinterpret_cast<OrchestratorBeginPackageLoadFn>(begin_symbol);
    functions.report_package_error = reinterpret_cast<OrchestratorReportPackageErrorFn>(report_symbol);
    functions.finish_package_load = reinterpret_cast<OrchestratorFinishPackageLoadFn>(finish_symbol);
    functions.set_error_message = reinterpret_cast<OrchestratorSetErrorMessageFn>(error_message_symbol);
    functions.register_exclusive_group = reinterpret_cast<OrchestratorRegisterExclusiveGroupFn>(group_symbol);
    functions.is_plugin_registered = reinterpret_cast<OrchestratorIsPluginRegisteredFn>(find_symbol);
    functions.send_log = reinterpret_cast<OrchestratorSendLogFn>(log_symbol);
#endif
    return true;
}

void register_python_group_definitions(PyObject *api, PyObject *plugin, const char *id, slicing_step_t step)
{
    PyObject *module = PyImport_ImportModule("slic3r_api");
    if (module == nullptr) throw std::runtime_error(take_python_error());
    PyObject *helper = PyObject_GetAttrString(module, "_register_exclusive_group_definitions");
    Py_DECREF(module);
    if (helper == nullptr) throw std::runtime_error(take_python_error());
    PyObject *result = PyObject_CallFunction(helper, "OOsi", api, plugin, id, int(step));
    Py_DECREF(helper);
    if (result == nullptr) throw std::runtime_error("Cannot register Python exclusive group definition: " + take_python_error());
    Py_DECREF(result);
}

void register_python_plugin_object(PyObject *plugin_object,
                                   const OrchestratorFunctions &functions,
                                   orchestrator_handle *orchestrator,
                                   const boost::filesystem::path &package_root, PyObject *api)
{
    if (plugin_object == nullptr || plugin_object == Py_None)
        return;

    const std::string package_id = package_root.empty() ? current_module_path().parent_path().filename().string() : package_root.filename().string();
    std::unique_ptr<PythonPlugin> plugin(new PythonPlugin(plugin_object, functions.set_error_message, orchestrator));
    plugin_instance instance = plugin->c_instance();
    const char *id = instance.vt->get_id(instance.ctx);
    const slicing_step_t step = instance.vt->get_step(instance.ctx);
    if (functions.is_plugin_registered(orchestrator, id) != 0)
        throw std::runtime_error("Python plugin '" + std::string(id) + "' is already registered.");
    s_python_plugins.reserve(s_python_plugins.size() + 1);
    s_python_plugins.emplace_back(std::move(plugin));
    if (package_root.empty())
        functions.register_plugin(orchestrator, instance);
    else
        functions.register_package_plugin(orchestrator, instance, package_root.string().c_str());
    if (functions.is_plugin_registered(orchestrator, id) == 0)
        throw std::runtime_error("Cannot register Python plugin '" + std::string(id) + "'.");
    for (const char *attribute : {"exclusive_group_label", "exclusive_group_tooltip"}) {
        if (PyObject_HasAttrString(plugin_object, attribute)) {
            const std::string message = "Python package '" + package_id + "', plugin '" + id +
                "': obsolete attribute '" + attribute + "'; declare the exclusive group explicitly.";
            functions.send_log(orchestrator, RAW_PLUGIN_LOG_WARNING, message.c_str());
        }
    }
    register_python_group_definitions(api, plugin_object, id, step);
    send_python_log(RAW_PLUGIN_LOG_INFO, "Registered Python plugin '" + std::string(id) + "'.");
}

void register_python_plugin_result(PyObject *result,
                                   const OrchestratorFunctions &functions,
                                   orchestrator_handle *orchestrator,
                                   const boost::filesystem::path &package_root, PyObject *api)
{
    if (result == nullptr || result == Py_None)
        return;

    if (PyList_Check(result) || PyTuple_Check(result)) {
        const Py_ssize_t count = PySequence_Size(result);
        for (Py_ssize_t idx = 0; idx < count; ++idx) {
            PyObject *item = PySequence_GetItem(result, idx);
            if (item == nullptr) throw std::runtime_error(take_python_error());
            try { register_python_plugin_object(item, functions, orchestrator, package_root, api); }
            catch (...) { Py_XDECREF(item); throw; }
            Py_XDECREF(item);
        }
        return;
    }

    register_python_plugin_object(result, functions, orchestrator, package_root, api);
}

PyObject *create_python_api(orchestrator_handle *orchestrator, const boost::filesystem::path &host_library_path)
{
    PyObject *api_module = PyImport_ImportModule("slic3r_api");
    if (api_module == nullptr)
        return nullptr;

    PyObject *api_class = PyObject_GetAttrString(api_module, "Slic3rAPI");
    Py_DECREF(api_module);
    if (api_class == nullptr)
        return nullptr;

    PyObject *py_orchestrator = pointer_to_python_uint(orchestrator);
    PyObject *py_host_library = python_path_from_boost(host_library_path);
    PyObject *api = py_orchestrator != nullptr && py_host_library != nullptr ?
                        PyObject_CallFunctionObjArgs(api_class, py_orchestrator, py_host_library, nullptr) :
                        nullptr;
    Py_DECREF(api_class);
    Py_XDECREF(py_orchestrator);
    Py_XDECREF(py_host_library);

    return api;
}

void call_python_register(PyObject *module,
                          PyObject *api,
                          const OrchestratorFunctions &functions,
                          orchestrator_handle *orchestrator,
                          const boost::filesystem::path &plugin_path,
                          const boost::filesystem::path &package_root)
{
    PyObject *register_fn = PyObject_GetAttrString(module, "register_plugin");
    if (register_fn == nullptr) {
        PyErr_Clear();
        const std::string detail = "Python plugin '" + plugin_path.string() +
                                   "' does not define register_plugin(api).";
        send_python_log(RAW_PLUGIN_LOG_WARNING, detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED, detail);
        return;
    }

    if (!PyCallable_Check(register_fn)) {
        const std::string detail = "Python plugin '" + plugin_path.string() +
                                   "' has a non-callable register_plugin attribute.";
        send_python_log(RAW_PLUGIN_LOG_WARNING, detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED, detail);
        Py_DECREF(register_fn);
        return;
    }

    PyObject *result = PyObject_CallFunctionObjArgs(register_fn, api, nullptr);
    Py_DECREF(register_fn);

    if (result == nullptr) {
        const std::string detail = take_python_error();
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Python plugin '" + plugin_path.string() + "' failed during registration: " + detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED, detail);
        return;
    }

    try {
        register_python_plugin_result(result, functions, orchestrator, package_root, api);
    } catch (const std::exception &error) {
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED,
                                    error.what());
    }
    Py_DECREF(result);
    send_python_log(RAW_PLUGIN_LOG_INFO, "Loaded Python plugin module '" + plugin_path.string() + "'.");
}

void load_python_plugin(const boost::filesystem::path &plugin_path,
                        PyObject *api,
                        const OrchestratorFunctions &functions,
                        orchestrator_handle *orchestrator,
                        const boost::filesystem::path &package_root)
{
    const std::string module_name = plugin_path.stem().string();
    PyObject *module = PyImport_ImportModule(module_name.c_str());
    if (module == nullptr) {
        const std::string detail = take_python_error();
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot import Python plugin '" + plugin_path.string() + "': " + detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_IMPORT_FAILED, detail);
        return;
    }

    call_python_register(module, api, functions, orchestrator, plugin_path, package_root);
    Py_DECREF(module);
}

void load_python_plugin_from_path(const boost::filesystem::path &plugin_path,
                                  PyObject *api,
                                  const OrchestratorFunctions &functions,
                                  orchestrator_handle *orchestrator,
                                  const boost::filesystem::path &package_root)
{
    boost::nowide::ifstream stream(plugin_path.string(), std::ios::in | std::ios::binary);
    if (!stream) {
        const std::string detail = "Cannot read Python plugin '" + plugin_path.string() + "'.";
        send_python_log(RAW_PLUGIN_LOG_ERROR, detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_READ_FAILED, detail);
        return;
    }
    const std::string source((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    static uint64_t next_module_id = 0;
    const std::string module_name = "_slic3r_package_" + std::to_string(++next_module_id);
    PyObject *code = Py_CompileString(source.c_str(), plugin_path.string().c_str(), Py_file_input);
    if (code == nullptr) {
        const std::string detail = take_python_error();
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot compile Python plugin '" + plugin_path.string() + "': " + detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_COMPILE_FAILED, detail);
        return;
    }
    PyObject *module = PyImport_ExecCodeModuleEx(
        module_name.c_str(), code, plugin_path.string().c_str());
    Py_DECREF(code);
    if (module == nullptr) {
        const std::string detail = take_python_error();
        send_python_log(RAW_PLUGIN_LOG_ERROR, "Cannot load Python plugin '" + plugin_path.string() + "': " + detail);
        report_python_package_error(functions, orchestrator, package_root,
                                    RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_IMPORT_FAILED, detail);
        return;
    }

    call_python_register(module, api, functions, orchestrator, plugin_path, package_root);
    Py_DECREF(module);
}

std::string native_plugin_filename()
{
#ifdef _WIN32
    return "plugin.dll";
#elif defined(__APPLE__)
    return "plugin.dylib";
#else
    return "plugin.so";
#endif
}

boost::filesystem::path external_python_entry(const boost::filesystem::path &package_root)
{
    const std::string package_id = package_root.filename().string();
    const boost::filesystem::path named_entry = package_root / (package_id + ".py");
    if (boost::filesystem::is_regular_file(named_entry))
        return named_entry;
    const boost::filesystem::path conventional_entry = package_root / "plugin.py";
    if (boost::filesystem::is_regular_file(conventional_entry))
        return conventional_entry;

    boost::filesystem::path unique_entry;
    for (boost::filesystem::directory_iterator it(package_root), end; it != end; ++it) {
        if (!boost::filesystem::is_regular_file(it->path()) || it->path().extension() != ".py")
            continue;
        if (!unique_entry.empty())
            return {};
        unique_entry = it->path();
    }
    return unique_entry;
}

void load_python_plugins(orchestrator_handle *orchestrator)
{
    s_log_orchestrator = orchestrator;
    const boost::filesystem::path loader_path = current_module_path();
    if (loader_path.empty()) {
        send_python_log(RAW_PLUGIN_LOG_WARNING, "Cannot locate python_plugin_loader module.");
        return;
    }

    const boost::filesystem::path plugin_repository = loader_path.parent_path();
    const boost::filesystem::path python_root = plugin_repository / "python";
    const boost::filesystem::path python_plugins = python_root / "plugins";
    const boost::filesystem::path host_path = host_library_path();

    OrchestratorFunctions functions;
    if (!resolve_registration_functions(host_path, functions))
        throw std::runtime_error("Cannot resolve the host functions required by the Python plugin runtime.");

    if (!ensure_python_initialized_for_plugins()) {
        throw std::runtime_error("Cannot initialize Python runtime for plugins.");
    }

    PyGILState_STATE gil_state = PyGILState_Ensure();
    append_python_path(python_root);
    append_python_path(python_plugins);

    PyObject *api = create_python_api(orchestrator, host_path);
    if (api != nullptr) {
        // Scripts bundled with the Python loader keep the loader package scope
        // supplied by the native PluginLoader.
        if (boost::filesystem::is_directory(python_plugins)) {
            for (boost::filesystem::directory_iterator it(python_plugins), end; it != end; ++it) {
                const boost::filesystem::path plugin_path = it->path();
                if (boost::filesystem::is_regular_file(plugin_path) && plugin_path.extension() == ".py")
                    load_python_plugin(plugin_path, api, functions, orchestrator, {});
            }
        }

        // Pure Python packages are siblings of native packages. A missing ABI
        // manifest is reported below, while absence of a library avoids double loading a
        // native package that also ships helper scripts.
        const boost::filesystem::path installed_packages = plugin_repository.parent_path();
        for (boost::filesystem::directory_iterator it(installed_packages), end; it != end; ++it) {
            const boost::filesystem::path package_root = it->path();
            if (!boost::filesystem::is_directory(package_root) || package_root == plugin_repository ||
                boost::filesystem::is_regular_file(package_root / native_plugin_filename()) ||
                !boost::filesystem::is_regular_file(package_root / "description.ini"))
                continue;
            const boost::filesystem::path plugin_path = external_python_entry(package_root);
            if (plugin_path.empty())
                continue;
            const PythonPackageLoadScope package_load(functions, orchestrator, package_root.string());
            try {
                Slic3r::PluginPackageMetadata metadata;
                std::string error;
                if (!Slic3r::read_plugin_package_metadata((package_root / "version.ini").string(), metadata, error) ||
                    !metadata.compatibility.compatible()) {
                    report_python_package_error(functions, orchestrator, package_root,
                        RAW_PLUGIN_PACKAGE_LOAD_ERROR_API_HEADER_VERSION_MISMATCH,
                        metadata.compatibility.message());
                } else {
                    append_python_path(package_root);
                    load_python_plugin_from_path(plugin_path, api, functions, orchestrator, package_root);
                }
            } catch (const std::exception &error) {
                report_python_package_error(functions, orchestrator, package_root,
                                            RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED,
                                            error.what());
            } catch (...) {
                report_python_package_error(functions, orchestrator, package_root,
                                            RAW_PLUGIN_PACKAGE_LOAD_ERROR_PYTHON_REGISTRATION_FAILED,
                                            "The Python package loader threw an unknown exception.");
            }
        }
        Py_DECREF(api);
    } else {
        const std::string detail = "Cannot create the Python plugin API: " + take_python_error();
        PyGILState_Release(gil_state);
        throw std::runtime_error(detail);
    }

    PyGILState_Release(gil_state);
}

} // namespace

#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orch)
{
    load_python_plugins(orch);
}
