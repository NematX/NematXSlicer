///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2023 Tomáš Mészáros @tamasmeszaros, Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2018 Colin Gilgenbach @hexane360
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Post-processing scripts
=======================

This file starts the user scripts of the `post_process` option on a G-code file. The only public entry point is
run_post_process_scripts(), used by the STEP_GCODE plugin "gcode.post_process", which runs it on the file the
G-code writer has just produced.

The normal execution flow is:

The copy is handed to the caller, who publishes it; on any error it is removed here. run_script() has one
implementation per platform: Windows starts the program through CreateProcessW (perl, python and batch files get
their interpreter), the other systems go through the shell. Neither waits with a time limit or reacts to a
cancellation.

https://blogs.msdn.microsoft.com/twistylittlepassagesallalike/2011/04/23/everyone-quotes-command-line-arguments-the-wrong-way/
This routine appends the given argument to a command line such that CommandLineToArgvW will return the argument string unchanged.
Arguments in a command line should be separated by spaces; this function does not add these spaces.
Argument    - Supplies the argument to encode.
CommandLine - Supplies the command line to which we append the encoded argument string.

Operation notes:
- run_post_process_scripts()
- configured_scripts(): list the non-empty lines of the option (none: nothing is done)
- copy the G-code to "<file>.pp"; the original is never touched
- ConfigBase::setenv_(): publish the configuration as SLIC3R_* environment variables
- run_script() for every line, in order, and wait for its end
- (the path of the copy is the last argument; a non-zero exit code is an error)
- discard_output_name_file(): a script cannot rename the exported file any more

Representative direct calls (branches are not a timeline):

    run_post_process_scripts()
        |-- configured_scripts()
        |-- boost::filesystem::exists()
        |-- remove_file_quietly()
        |-- discard_output_name_file()
        |-- copy_file_inner()
        `-- run_script()
*/

#include "PostProcessor.hpp"

#include <cstdlib>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/convert.hpp>

#include "libslic3r/format.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"

#ifdef WIN32
// The standard Windows includes.
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <shellapi.h>
#else
// POSIX
#include <sstream>
#include <boost/process.hpp>
#include <unistd.h>     //readlink
#endif


#ifdef WIN32


static void quote_argv_winapi(const std::wstring &argument, std::wstring &commmand_line_out)
{
	// Don't quote unless we actually need to do so --- hopefully avoid problems if programs won't parse quotes properly.
	if (argument.empty() == false && argument.find_first_of(L" \t\n\v\"") == argument.npos)
		commmand_line_out.append(argument);
	else {
		commmand_line_out.push_back(L'"');
		for (auto it = argument.begin(); ; ++ it) {
			unsigned number_backslashes = 0;
			while (it != argument.end() && *it == L'\\') {
				++ it;
				++ number_backslashes;
			}
			if (it == argument.end()) {
				// Escape all backslashes, but let the terminating double quotation mark we add below be interpreted as a metacharacter.
				commmand_line_out.append(number_backslashes * 2, L'\\');
				break;
			} else if (*it == L'"') {
				// Escape all backslashes and the following double quotation mark.
				commmand_line_out.append(number_backslashes * 2 + 1, L'\\');
				commmand_line_out.push_back(*it);
			} else {
				// Backslashes aren't special here.
				commmand_line_out.append(number_backslashes, L'\\');
				commmand_line_out.push_back(*it);
			}
		}
		commmand_line_out.push_back(L'"');
	}
}

static DWORD execute_process_winapi(const std::wstring &command_line)
{
    // Extract the current environment to be passed to the child process.
	std::wstring envstr;
	{
		wchar_t *env = GetEnvironmentStrings();
		assert(env != nullptr);
		const wchar_t* var = env;
		size_t totallen = 0;
		size_t len;
		while ((len = wcslen(var)) > 0) {
			totallen += len + 1;
			var += len + 1;
		}
		envstr = std::wstring(env, totallen);
		FreeEnvironmentStrings(env);
	}

	STARTUPINFOW startup_info;
	memset(&startup_info, 0, sizeof(startup_info));
	startup_info.cb			 = sizeof(STARTUPINFO);
#if 0
	startup_info.dwFlags	 = STARTF_USESHOWWINDOW;
	startup_info.wShowWindow = SW_HIDE;
#endif
	PROCESS_INFORMATION process_info;
	if (! ::CreateProcessW(
            nullptr /* lpApplicationName */, (LPWSTR)command_line.c_str(), nullptr /* lpProcessAttributes */, nullptr /* lpThreadAttributes */, false /* bInheritHandles */,
			CREATE_UNICODE_ENVIRONMENT /* | CREATE_NEW_CONSOLE */ /* dwCreationFlags */, (LPVOID)envstr.c_str(), nullptr /* lpCurrentDirectory */, &startup_info, &process_info))
		throw Slic3r::RuntimeError(std::string("Failed starting the script ") + boost::nowide::narrow(command_line) + ", Win32 error: " + std::to_string(int(::GetLastError())));
	::WaitForSingleObject(process_info.hProcess, INFINITE);
	ULONG rc = 0;
	::GetExitCodeProcess(process_info.hProcess, &rc);
	::CloseHandle(process_info.hThread);
	::CloseHandle(process_info.hProcess);
	return rc;
}

// Run the script. If it is a perl script, run it through the bundled perl interpreter.
// If it is a batch file, run it through the cmd.exe.
// Otherwise run it directly.
static int run_script(const std::string &script, const std::string &gcode, std::string &/*std_err*/)
{
    // Unpack the argument list provided by the user.
    int     nArgs;
    LPWSTR *szArglist = CommandLineToArgvW(boost::nowide::widen(script).c_str(), &nArgs);
    if (szArglist == nullptr || nArgs <= 0) {
        // CommandLineToArgvW failed. Maybe the command line escapment is invalid?
		throw Slic3r::RuntimeError(std::string("Post processing script ") + script + " on file " + gcode + " failed. CommandLineToArgvW() refused to parse the command line path.");
    }

    std::wstring command_line;
    const std::wstring command = szArglist[0];
    bool need_absolute_path = false;
    // for perl and python, hope that the os can find it in the path.
    if (boost::iends_with(command, L".pl")) {
        BOOST_LOG_TRIVIAL(debug) << boost::format("Executing script : detecting perl script");
        // This is a perl script. Run it through the perl interpreter.
        quote_argv_winapi(boost::nowide::widen("perl.exe"), command_line);
        command_line += L" ";
        need_absolute_path = true;
    } else if (boost::iends_with(command, L".py")) {
        BOOST_LOG_TRIVIAL(debug) << boost::format("Executing script : detecting python script");
        // This is a python script. Run it through the python interpreter.
        quote_argv_winapi(boost::nowide::widen("python.exe"), command_line);
        command_line += L" ";
        need_absolute_path = true;
    } else if (boost::iends_with(command, ".bat")) {
        BOOST_LOG_TRIVIAL(debug) << boost::format("Executing script : detecting bat script");
        // Run a batch file through the command line interpreter.
        command_line = L"cmd.exe /C ";
        need_absolute_path = true;
    }
    
    std::wstring absolute_command_path;
    //check if it's an exe
    if (!need_absolute_path && boost::iends_with(command, ".exe")) {
        // exe: it may come from the path, don't check.
        absolute_command_path = command;
    } else {
        //try to find the file, in different directories
        absolute_command_path = Slic3r::find_full_path(boost::filesystem::path(command)).generic_wstring();
        if (absolute_command_path.empty()) {
            if (need_absolute_path) {
                BOOST_LOG_TRIVIAL(warning) << "The configured post-processing script may not exist: " << command;
            }
            absolute_command_path = command;
        }
    }
    quote_argv_winapi(absolute_command_path, command_line);
    command_line += L" ";

    for (int i = 1; i < nArgs; ++ i) {
        quote_argv_winapi(szArglist[i], command_line);
        command_line += L" ";
    }
    LocalFree(szArglist);
	quote_argv_winapi(boost::nowide::widen(gcode), command_line);
    BOOST_LOG_TRIVIAL(debug) << (boost::format("Executing script : %1%") % boost::nowide::narrow(command_line));
    return (int)execute_process_winapi(command_line);
}

#else

namespace process = boost::process;

static int run_script(const std::string &script, const std::string &gcode, std::string &std_err)
{
    // Try to obtain user's default shell
    const char* shell = ::getenv("SHELL");
    if (shell == nullptr) { shell = "/bin/sh"; }

    // Quote and escape the gcode path argument
    std::string command_line;
    size_t first_space = script.find(' ');
    bool need_absolute_path = false;
    const std::string command = (std::string::npos != first_space) ? script.substr(0, first_space) : script;
    const std::string args = (std::string::npos != first_space) ? script.substr(first_space) : "";
    if (boost::iends_with(command, L".pl")) {
        BOOST_LOG_TRIVIAL(trace) << boost::format("Executing script : detecting perl script");
        // This is a perl script. Run it through the perl interpreter.
        command_line = "perl ";
        need_absolute_path = true;
    } else if (boost::iends_with(command, L".py")) {
        BOOST_LOG_TRIVIAL(trace) << boost::format("Executing script : detecting python script");
        // This is a python script. Run it through the python interpreter.
        command_line = "python3 ";
        need_absolute_path = true;
    }

    std::string absolute_command_path;

    //check if it's an exe/command (ie it doesn't have an extension)
    if (!need_absolute_path && command.find('.') != std::string::npos) {
        // command: it may come from the path, don't check.
        absolute_command_path = command;
    } else {
        //try to find the file, in different directories
        absolute_command_path = Slic3r::find_full_path(boost::filesystem::path(command)).generic_string();
        if (absolute_command_path.empty()) {
            if (need_absolute_path) {
                BOOST_LOG_TRIVIAL(warning) << "The configured post-processing script may not exist: " << command;
            }
            absolute_command_path = command;
        }
    }
    command_line += absolute_command_path;
    command_line += args;

    command_line.append(" '");
    for (char c : gcode) {
        if (c == '\'') { command_line.append("'\\''"); }
        else { command_line.push_back(c); }
    }
    command_line.push_back('\'');

    BOOST_LOG_TRIVIAL(trace) << boost::format("Executing script, shell: %1%, command: %2%") % shell % command_line;
    process::ipstream istd_err;
    process::child child(shell, "-c", command_line, process::std_err > istd_err);

    std_err.clear();
    std::string line;

    while (child.running() && std::getline(istd_err, line)) {
        std_err.append(line);
        std_err.push_back('\n');
    }

    child.wait();
    return child.exit_code();
}

#endif

namespace Slic3r {

// Return the non-empty, trimmed lines of the post_process option. A configuration without the option (SLA) gives none.
static std::vector<std::string> configured_scripts(const DynamicPrintConfig &config);

// Remove a file if it exists. A failure is logged with the given role and does not throw: it only concerns temporary files.
static void remove_file_quietly(const std::string &path, const char *role);

// Remove the file "<path>.output_name", which older versions let a script create to rename the exported file. It is ignored:
// the name of the exported file is not decided by a script. Its presence is reported in the log when warn is set.
static void discard_output_name_file(const std::string &path, bool warn);

// Run the post_process scripts of the configuration on a temporary copy of src_path.
// Returns the path of the copy ("<src_path>.pp"), which the caller owns, or an empty string when no script is configured
// (nothing is read or written in that case). Throws on error, after removing the copy.
//
// Each script is a line of the post_process option. It is started with the path of the copy as its last argument and is
// expected to modify that file in place; the next script then sees the result of the previous one. The program waits for
// every script to end: there is no time limit and no cancellation.
//
// The scripts also read the configuration: every key that is allowed in a project file is put in the environment of the
// process as SLIC3R_<KEY> (see ConfigBase::setenv_). The environment inherited from the user is left as it is.
std::string run_post_process_scripts(const std::string &src_path, const DynamicPrintConfig &config)
{
    const std::vector<std::string> scripts = configured_scripts(config);
    if (scripts.empty())
        return std::string();

    if (! boost::filesystem::exists(src_path))
        throw Slic3r::RuntimeError(Slic3r::format("Post-processor can't find the G-code file %1%", src_path));

    // Scripts never work on the input file: it may be memory mapped by the G-code viewer, and a failing script
    // must leave it untouched.
    const std::string path = src_path + ".pp";
    remove_file_quietly(path, "an old temporary file");
    discard_output_name_file(path, false);

    try {
        // copy_file_inner() writes the target directly: the copy is not published, only processed.
        std::string error_message;
        if (copy_file_inner(src_path, path, error_message) != SUCCESS)
            throw Slic3r::RuntimeError(Slic3r::format("Failed making a temporary copy of G-code file %1% before running a post-processing script: %2%", src_path, error_message));

        // The environment of the process is global and is not restored: the next run overwrites the same variables.
        config.setenv_();

        for (const std::string &script : scripts) {
            BOOST_LOG_TRIVIAL(info) << "Executing script " << script << " on file " << path;
            std::string std_err;
            const int result = run_script(script, path, std_err);
            if (result != 0) {
                const std::string msg = std_err.empty() ? (boost::format("Post-processing script %1% on file %2% failed.\nError code: %3%") % script % path % result).str()
                    : (boost::format("Post-processing script %1% on file %2% failed.\nError code: %3%\nOutput:\n%4%") % script % path % result % std_err).str();
                BOOST_LOG_TRIVIAL(error) << msg;
                throw Slic3r::RuntimeError(msg);
            }
            if (! boost::filesystem::exists(path)) {
                const std::string msg = (boost::format(_u8L(
                    "Post-processing script %1% failed.\n\n"
                    "The post-processing script is expected to change the G-code file %2% in place, but the G-code file was deleted and likely saved under a new name.\n"
                    "Please adjust the post-processing script to change the G-code in place.\n"))
                    % script % path).str();
                BOOST_LOG_TRIVIAL(error) << msg;
                throw Slic3r::RuntimeError(msg);
            }
        }
        discard_output_name_file(path, true);
    } catch (...) {
        discard_output_name_file(path, false);
        remove_file_quietly(path, "a temporary copy");
        throw;
    }
    return path;
}

static std::vector<std::string> configured_scripts(const DynamicPrintConfig &config)
{
    std::vector<std::string> scripts;
    const ConfigOptionStrings *post_process = config.opt<ConfigOptionStrings>("post_process");
    if (post_process == nullptr)
        return scripts;
    for (const std::string &lines : post_process->get_values()) {
        std::vector<std::string> split_lines;
        boost::split(split_lines, lines, boost::is_any_of("\r\n"));
        for (std::string &script : split_lines) {
            boost::trim(script);
            if (! script.empty())
                scripts.push_back(std::move(script));
        }
    }
    return scripts;
}

static void remove_file_quietly(const std::string &path, const char *role)
{
    try {
        if (boost::filesystem::exists(path))
            boost::filesystem::remove(path);
    } catch (const std::exception &err) {
        BOOST_LOG_TRIVIAL(error) << Slic3r::format("Failed deleting %1% %2% of a post-processing script: %3%", role, path, err.what());
    }
}

static void discard_output_name_file(const std::string &path, bool warn)
{
    const std::string path_output_name = path + ".output_name";
    if (warn && boost::filesystem::exists(path_output_name))
        BOOST_LOG_TRIVIAL(warning) << "A post-processing script created " << path_output_name
            << ". A script can no longer rename the exported G-code, so the file is ignored.";
    remove_file_quietly(path_output_name, "the file");
}

} // namespace Slic3r
