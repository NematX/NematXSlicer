/*
 * Shared REST server launch implementation.
 *
 * Configuration is loaded before the listener starts. The process remains
 * alive until SIGINT/SIGTERM (or Windows SIGBREAK), then delegates shutdown so sockets
 * and the worker thread are closed deterministically.
 */

#include "rest_server.hpp"
#include "engine.hpp"
#include "rest_main_internal.hpp"

#include <boost/dll/runtime_symbol_info.hpp>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace Slic3r::Rest {

namespace {

// sig_atomic_t is safe to write from the C signal handler. Launches are sequential.
volatile std::sig_atomic_t stop_requested = 0;

// Parse only REST options, without ever initializing the application CLI or GUI.
struct CommandLine;
CommandLine parse_command_line(int argc, char **argv);
std::string required_value(int argc, char **argv, int &index, const std::string &option);
std::uint16_t parse_port(const std::string &value);
// Display the actual host invocation instead of the standalone target name.
void print_help(const std::string &invocation);
void handle_signal(int);

void handle_signal(int)
{
    stop_requested = 1;
}

// Restore the embedding application's handlers on success and on exceptions.
struct SignalScope {
    using Handler = void (*)(int);
    Handler interrupt;
    Handler terminate;
#ifdef _WIN32
    Handler console_break;
#endif
    SignalScope()
    {
        stop_requested = 0;
        interrupt = std::signal(SIGINT, handle_signal);
        if (interrupt == SIG_ERR)
            throw std::runtime_error("Cannot install SIGINT handler.");
        terminate = std::signal(SIGTERM, handle_signal);
        if (terminate == SIG_ERR) {
            std::signal(SIGINT, interrupt);
            throw std::runtime_error("Cannot install SIGTERM handler.");
        }
#ifdef _WIN32
        console_break = std::signal(SIGBREAK, handle_signal);
        if (console_break == SIG_ERR) {
            std::signal(SIGINT, interrupt);
            std::signal(SIGTERM, terminate);
            throw std::runtime_error("Cannot install SIGBREAK handler.");
        }
#endif
    }
    ~SignalScope()
    {
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, terminate);
#ifdef _WIN32
        std::signal(SIGBREAK, console_break);
#endif
    }
};

struct CommandLine {
    std::filesystem::path config_file;
    Slic3r::Rest::CommandLineOverrides overrides;
    bool help = false;
};

std::string required_value(int argc, char **argv, int &index, const std::string &option)
{
    if (index + 1 >= argc || argv[index + 1] == nullptr)
        throw std::invalid_argument(option + " requires a value");
    ++index;
    return argv[index];
}

std::uint16_t parse_port(const std::string &value)
{
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("--port must be an integer from 0 to 65535");
    std::size_t parsed_length = 0;
    const unsigned long parsed = std::stoul(value, &parsed_length, 10);
    if (parsed_length != value.size() || parsed > 65535UL)
        throw std::invalid_argument("--port must be an integer from 0 to 65535");
    return static_cast<std::uint16_t>(parsed);
}

CommandLine parse_command_line(int argc, char **argv)
{
    CommandLine command_line;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index] != nullptr ? argv[index] : std::string();
        if (option == "--help" || option == "-h") {
            command_line.help = true;
        } else if (option == "--config") {
            command_line.config_file = std::filesystem::u8path(required_value(argc, argv, index, option));
        } else if (option == "--bind-address") {
            command_line.overrides.bind_address = required_value(argc, argv, index, option);
        } else if (option == "--port") {
            command_line.overrides.port = parse_port(required_value(argc, argv, index, option));
        } else if (option == "--resources-dir") {
            command_line.overrides.resource_directory = std::filesystem::u8path(required_value(argc, argv, index, option));
        } else if (option == "--profiles-dir") {
            command_line.overrides.profiles_directory = std::filesystem::u8path(required_value(argc, argv, index, option));
        } else if (option == "--work-dir") {
            command_line.overrides.work_directory = std::filesystem::u8path(required_value(argc, argv, index, option));
        } else if (option == "--accounts-file") {
            command_line.overrides.accounts_file = std::filesystem::u8path(required_value(argc, argv, index, option));
        } else {
            throw std::invalid_argument("unknown option: " + option);
        }
    }
    return command_line;
}

void print_help(const std::string &invocation)
{
    std::cout
        << "Usage: " << invocation << " [options]\n"
        << "  --config PATH          Load a JSON server configuration\n"
        << "  --bind-address VALUE   Override listen.address\n"
        << "  --port VALUE           Override listen.port (0 selects an OS port)\n"
        << "  --resources-dir PATH   Override paths.resources\n"
        << "  --profiles-dir PATH    Override paths.profiles\n"
        << "  --work-dir PATH        Override paths.work\n"
        << "  --accounts-file PATH   Override paths.accounts\n"
        << "  --help                 Show this help\n";
}

} // namespace

int run_rest_server(int argc, char **argv, const RestLaunchDefaults &defaults,
    const std::function<void(std::uint16_t)> &on_started)
{
    const std::string invocation = defaults.invocation.empty() ? argv[0] : defaults.invocation;
    try {
        const CommandLine command_line = parse_command_line(argc, argv);
        if (command_line.help) {
            print_help(invocation);
            return EXIT_SUCCESS;
        }

        Slic3r::Rest::ServerConfig config = Slic3r::Rest::load_server_config(
            command_line.config_file,
            command_line.overrides, defaults.resource_directory);
        Slic3r::Rest::EnginePaths engine_paths;
        engine_paths.resource_directory = config.resource_directory;
        engine_paths.data_directory = config.work_directory / "engine_data";
        // argv[0] may be a bare PATH command or a link, not an absolute file.
        engine_paths.binary_file = std::filesystem::path(boost::dll::program_location().native());
        Slic3r::Rest::Engine::instance().initialize(engine_paths);
        SignalScope signals;
        const std::string bind_address = config.bind_address;
        Slic3r::Rest::RestServer server(std::move(config));
        server.start();
        // Brackets separate an IPv6 literal's colons from the listener port.
        const std::string listen_host = bind_address.find(':') == std::string::npos
            ? bind_address : '[' + bind_address + ']';
        std::cout << "listening on " << listen_host << ':' << server.bound_port() << std::endl;
        for (const std::string &warning : server.startup_warnings())
            std::cerr << invocation << ": " << warning << std::endl;

        if (on_started)
            on_started(server.bound_port());
        while (!stop_requested)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "stopping" << std::endl;
        server.stop();
        std::cout << "stopped" << std::endl;
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << invocation << ": " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

int rest_server_main(int argc, char **argv, const RestLaunchDefaults &defaults)
{
    return run_rest_server(argc, argv, defaults, {});
}

} // namespace Slic3r::Rest
