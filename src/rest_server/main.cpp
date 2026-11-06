/*
 * Standalone REST server entry point.
 *
 * Configuration is loaded before the listener starts. The process remains
 * alive until SIGINT/SIGTERM, then delegates shutdown to RestServer so sockets
 * and the worker thread are closed deterministically.
 */

#include "rest_server.hpp"
#include "engine.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

std::atomic<bool> stop_requested{false};

void handle_signal(int)
{
    stop_requested.store(true);
}

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
            command_line.config_file = required_value(argc, argv, index, option);
        } else if (option == "--bind-address") {
            command_line.overrides.bind_address = required_value(argc, argv, index, option);
        } else if (option == "--port") {
            command_line.overrides.port = parse_port(required_value(argc, argv, index, option));
        } else if (option == "--resources-dir") {
            command_line.overrides.resource_directory = required_value(argc, argv, index, option);
        } else if (option == "--profiles-dir") {
            command_line.overrides.profiles_directory = required_value(argc, argv, index, option);
        } else if (option == "--work-dir") {
            command_line.overrides.work_directory = required_value(argc, argv, index, option);
        } else if (option == "--accounts-file") {
            command_line.overrides.accounts_file = required_value(argc, argv, index, option);
        } else {
            throw std::invalid_argument("unknown option: " + option);
        }
    }
    return command_line;
}

void print_help()
{
    std::cout
        << "Usage: slic3r_rest_server [options]\n"
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

int main(int argc, char **argv)
{
    try {
        const CommandLine command_line = parse_command_line(argc, argv);
        if (command_line.help) {
            print_help();
            return EXIT_SUCCESS;
        }

        Slic3r::Rest::ServerConfig config = Slic3r::Rest::load_server_config(
            command_line.config_file,
            command_line.overrides);
        Slic3r::Rest::EnginePaths engine_paths;
        engine_paths.resource_directory = config.resource_directory;
        engine_paths.data_directory = config.work_directory / "engine_data";
        Slic3r::Rest::Engine::instance().initialize(engine_paths);
        Slic3r::Rest::RestServer server(std::move(config));
        server.start();

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        while (!stop_requested.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        server.stop();
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "slic3r_rest_server: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
