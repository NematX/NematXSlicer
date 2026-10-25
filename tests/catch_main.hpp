#ifndef CATCH_MAIN
#define CATCH_MAIN

#define CATCH_CONFIG_EXTERNAL_INTERFACES
#define CATCH_CONFIG_MAIN
// #define CATCH_CONFIG_DEFAULT_REPORTER "verboseconsole"
#include <catch2/catch.hpp>

#ifdef _MSC_VER
#include <crtdbg.h>
#include <stdlib.h>

namespace {
// The MSVC runtime answers a failed assert(), an abort() and a failed Debug check of the standard library
// (for example an out-of-range vector index, reported through _CrtDbgReport) with a dialog box or a
// breakpoint, which blocks or silently kills unattended and parallel test runs. These settings print the
// assertion or the check on stderr and end the process instead, without the abort message box or the
// Windows Error Reporting dialog. The assertions stay active. The object is built before main(), so before
// any test runs.
struct NoCrtDialogs {
    // Print failed assert() and Debug runtime checks on stderr, and let abort() end the process without a dialog.
    NoCrtDialogs()
    {
        _set_error_mode(_OUT_TO_STDERR);
        _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#ifdef _DEBUG
        // Debug runtime reports: _CRT_ASSERT (standard library Debug checks) and _CRT_ERROR (for example Debug heap
        // corruption reports) open a dialog by default; _CRT_WARN goes to the debugger output and is left alone.
        _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
        _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
        _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
        _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
#endif // _DEBUG
    }
};
const NoCrtDialogs no_crt_dialogs;
} // namespace
#endif // _MSC_VER

namespace Catch {
struct VerboseConsoleReporter : public ConsoleReporter {
    double duration = 0.;
    using ConsoleReporter::ConsoleReporter;
    
    void testCaseStarting(TestCaseInfo const& _testInfo) override
    {
        Colour::use(Colour::Cyan);
        stream << "Testing ";
        Colour::use(Colour::None);
        stream << _testInfo.name << std::endl;
        ConsoleReporter::testCaseStarting(_testInfo);
    }
    
    void sectionStarting(const SectionInfo &_sectionInfo) override
    {
        if (_sectionInfo.name != currentTestCaseInfo->name)
            stream << _sectionInfo.name << std::endl;
        
        ConsoleReporter::sectionStarting(_sectionInfo);
    }
    
    void sectionEnded(const SectionStats &_sectionStats) override {
        duration += _sectionStats.durationInSeconds;
        ConsoleReporter::sectionEnded(_sectionStats);
    } 
    
    void testCaseEnded(TestCaseStats const& stats) override
    {
        if (stats.totals.assertions.allOk()) {
            Colour::use(Colour::BrightGreen);
            stream << "Passed";
            Colour::use(Colour::None);
            stream << " in " << duration << " [seconds]\n" << std::endl;
        }
        
        duration = 0.;            
        ConsoleReporter::testCaseEnded(stats);
    }
};

CATCH_REGISTER_REPORTER( "verboseconsole", VerboseConsoleReporter )

} // namespace Catch

#endif // CATCH_MAIN
