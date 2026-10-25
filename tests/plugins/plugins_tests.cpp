// Entry point of plugins_tests: the Catch2 main comes from catch_main.hpp, and the listener at the end of the file
// initializes the shared plugin test runtime before any test case runs.
//
// This file also holds a test-only crash diagnostic, active only when the environment variable
// SLIC3R_CRASH_TRACE is set (nothing is installed otherwise):
//   SLIC3R_CRASH_TRACE=1          print the call stack (module+RVA and, when dbghelp finds the .pdb,
//                                 symbol+offset and file:line) on a failed Debug CRT assert/error, an
//                                 access violation, std::terminate and SIGABRT, then let the process end
//                                 as it would have;
//                                 It does not check the heap.
//   SLIC3R_CRASH_TRACE=testcheck  the same, and in Debug builds also check the whole Debug heap after
//                                 every test case (_CrtCheckMemory). When the check fails, the name of the
//                                 test case is printed and the process ends at once with exit code 70 (the
//                                 heap cannot be trusted any more), so a corrupted heap never goes unnoticed;
//   SLIC3R_CRASH_TRACE=heapcheck  the same as testcheck, and freed blocks are kept (and checked) instead of
//                                 being reused, which makes writes after free visible, at the price of
//                                 memory growth and much slower runs (about 50 times in one measured run).
// Stack traces go to stderr and are flushed at once, because the process may die right after.
#include <catch_main.hpp>

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include <windows.h>
#include <dbghelp.h>
#ifdef _DEBUG
#include <crtdbg.h>
#endif

#pragma comment(lib, "dbghelp.lib")

namespace {

// Return true when SLIC3R_CRASH_TRACE is set; `testcheck` is true for the values "testcheck" and "heapcheck",
// `heapcheck` only for "heapcheck".
bool crash_trace_requested(bool &testcheck, bool &heapcheck);
// Print the current call stack to stderr; `once` makes the first call the only one of the process.
void print_stack(const char *reason, bool once);
// Print one stack frame: module+RVA, then symbol+offset and file:line when dbghelp resolves them.
void print_frame(HANDLE process, unsigned frame_idx, void *address);
// Vectored exception handler: print the stack of access violations (0xC0000005), heap corruption (0xC0000374)
// and fast-fail/stack-cookie failures (0xC0000409), when the process is still able to deliver them.
LONG CALLBACK on_exception(PEXCEPTION_POINTERS info);
// std::terminate and SIGABRT handlers: print the stack, then end the process as abort() does.
void on_terminate();
void on_abort_signal(int signal_number);
#ifdef _DEBUG
// Debug CRT report hook (assertions and errors such as an invalid heap pointer): print the stack, let the report go on.
int __cdecl on_crt_report(int report_type, char *message, int *return_value);
// Catch2 listener for SLIC3R_CRASH_TRACE=testcheck or heapcheck.
class HeapCheckListener;
#endif

// Set while a stack is being printed (dbghelp is not thread safe and may allocate, which could re-enter a hook).
std::atomic<bool> g_printing{ false };

bool crash_trace_requested(bool &testcheck, bool &heapcheck)
{
    char value[32] = {};
    const DWORD length = GetEnvironmentVariableA("SLIC3R_CRASH_TRACE", value, DWORD(sizeof(value)));
    const std::string text = length > 0 && length < sizeof(value) ? std::string(value) : std::string();
    heapcheck = text == "heapcheck";
    testcheck = heapcheck || text == "testcheck";
    return length > 0;
}

void print_stack(const char *reason, bool once)
{
    // Another print is running (or this is a re-entrance): skip. With `once` the flag is never released.
    if (g_printing.exchange(true))
        return;
    std::fprintf(stderr, "\n=== SLIC3R_CRASH_TRACE: %s (thread %lu) ===\n", reason, GetCurrentThreadId());
    std::fflush(stderr);

    void *frames[62] = {};
    const unsigned count = CaptureStackBackTrace(0, 62, frames, nullptr);

    // Symbols are loaded lazily by dbghelp; the .pdb files sit next to the executable.
    char exe_dir[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exe_dir, MAX_PATH);
    char *last_separator = std::strrchr(exe_dir, '\\');
    if (last_separator != nullptr)
        *last_separator = '\0';
    const HANDLE process = GetCurrentProcess();
    SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
    SymInitialize(process, exe_dir, TRUE);
    for (unsigned idx = 0; idx < count; ++idx)
        print_frame(process, idx, frames[idx]);
    std::fprintf(stderr, "=== end of trace ===\n");
    std::fflush(stderr);
    SymCleanup(process);
    if (! once)
        g_printing = false;
}

void print_frame(HANDLE process, unsigned frame_idx, void *address)
{
    const DWORD64 address_value = DWORD64(reinterpret_cast<uintptr_t>(address));

    // Module name and offset of the frame inside it, always available.
    HMODULE module = nullptr;
    char module_path[MAX_PATH] = "?";
    DWORD64 rva = address_value;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCSTR>(address), &module) && module != nullptr) {
        GetModuleFileNameA(module, module_path, MAX_PATH);
        rva = address_value - DWORD64(reinterpret_cast<uintptr_t>(module));
    }
    const char *module_name = std::strrchr(module_path, '\\');
    module_name = module_name != nullptr ? module_name + 1 : module_path;

    // Symbol name and source line, when the .pdb is found.
    alignas(SYMBOL_INFO) char symbol_storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
    SYMBOL_INFO &symbol = *reinterpret_cast<SYMBOL_INFO *>(symbol_storage);
    symbol.SizeOfStruct = sizeof(SYMBOL_INFO);
    symbol.MaxNameLen = MAX_SYM_NAME;
    DWORD64 symbol_offset = 0;
    const bool has_symbol = SymFromAddr(process, address_value, &symbol_offset, &symbol) != FALSE;
    IMAGEHLP_LINE64 line = {};
    line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
    DWORD line_offset = 0;
    const bool has_line = SymGetLineFromAddr64(process, address_value, &line_offset, &line) != FALSE;

    std::fprintf(stderr, "#%02u %s+0x%llx  %s+0x%llx", frame_idx, module_name, (unsigned long long)rva,
                 has_symbol ? symbol.Name : "?", (unsigned long long)symbol_offset);
    if (has_line)
        std::fprintf(stderr, "  [%s:%lu]", line.FileName, line.LineNumber);
    std::fprintf(stderr, "\n");
    std::fflush(stderr);
}

LONG CALLBACK on_exception(PEXCEPTION_POINTERS info)
{
    const DWORD code = info->ExceptionRecord->ExceptionCode;
    if (code == 0xC0000005 || code == 0xC0000374 || code == 0xC0000409) {
        char reason[96];
        std::snprintf(reason, sizeof(reason), "exception 0x%08lX at %p", code, info->ExceptionRecord->ExceptionAddress);
        print_stack(reason, true);
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

void on_terminate()
{
    print_stack("std::terminate", true);
    std::abort();
}

void on_abort_signal(int)
{
    print_stack("SIGABRT", true);
    // Restore the default action so that the process ends with the exit code abort() normally gives.
    std::signal(SIGABRT, SIG_DFL);
    std::abort();
}

#ifdef _DEBUG
int __cdecl on_crt_report(int report_type, char *message, int *)
{
    if (report_type == _CRT_ASSERT || report_type == _CRT_ERROR) {
        std::fprintf(stderr, "\nCRT report: %s\n", message != nullptr ? message : "");
        print_stack("Debug CRT assertion/error", false);
    }
    // FALSE: the normal report (see NoCrtDialogs in catch_main.hpp) still happens.
    return FALSE;
}

// After each test case, validate every block of the Debug heap. A failure names the test case that left the heap
// corrupt (or whose destructors, run before the check, did); the process then ends with a non-zero exit code.
class HeapCheckListener : public Catch::TestEventListenerBase {
public:
    // Catch2 builds one listener per run; it checks the heap only when SLIC3R_CRASH_TRACE=testcheck or heapcheck.
    explicit HeapCheckListener(const Catch::ReporterConfig &config) : Catch::TestEventListenerBase(config)
    {
        bool heapcheck = false;
        crash_trace_requested(m_enabled, heapcheck);
    }

    void testCaseEnded(const Catch::TestCaseStats &stats) override
    {
        if (! m_enabled)
            return;
        if (! _CrtCheckMemory()) {
            std::fprintf(stderr, "\n=== HEAPCHECK: the Debug heap is corrupt after test case \"%s\" ===\n",
                         stats.testInfo.name.c_str());
            // Flush every stream before ending the process without running destructors on a corrupt heap.
            std::fflush(nullptr);
            std::_Exit(70);
        }
    }

private:
    bool m_enabled = false;
};

#endif // _DEBUG

// Installs the handlers before main() when SLIC3R_CRASH_TRACE is set.
struct CrashTraceInstaller {
    CrashTraceInstaller()
    {
        bool testcheck = false, heapcheck = false;
        if (! crash_trace_requested(testcheck, heapcheck))
            return;
        AddVectoredExceptionHandler(1, &on_exception);
        std::set_terminate(&on_terminate);
        std::signal(SIGABRT, &on_abort_signal);
#ifdef _DEBUG
        _CrtSetReportHook2(_CRT_RPTHOOK_INSTALL, &on_crt_report);
        if (heapcheck)
            // Keep freed blocks (filled with 0xDD and re-checked) and check the whole heap every 1024 allocations.
            _CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_DELAY_FREE_MEM_DF | _CRTDBG_CHECK_EVERY_1024_DF);
#endif
    }
};
const CrashTraceInstaller crash_trace_installer;

} // namespace

#ifdef _DEBUG
CATCH_REGISTER_LISTENER(HeapCheckListener)
#endif

#endif // _WIN32

#include "plugin_test_helpers.hpp"

namespace {

// Parallel launcher buckets run in separate processes. The shared "Scenario: init_print
// functionality" of tests/data/cpp/test_data.cpp cannot initialize plugin providers, so this
// target-specific listener does so at run start.
class PluginTestRuntimeListener final : public Catch::TestEventListenerBase
{
public:
    using Catch::TestEventListenerBase::TestEventListenerBase;

    // Catch2 run metadata is ignored. This void callback initializes Orchestrator-owned plugin
    // state before cases; registration exceptions propagate and missing plugins log then assert.
    void testRunStarting(const Catch::TestRunInfo &) override
    {
        Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    }
};

CATCH_REGISTER_LISTENER(PluginTestRuntimeListener)

} // namespace