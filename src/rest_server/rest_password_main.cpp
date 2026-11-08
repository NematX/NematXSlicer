/*
 * Local REST account provisioning tool.
 *
 * The username is supplied as an option and the password is read without
 * terminal echo. Only a complete accounts JSON document containing the
 * Argon2id hash is written to standard output.
 */

#include "auth.hpp"
#include "rest_main_internal.hpp"

#include <nlohmann/json.hpp>

#ifdef _WIN32
#    include <conio.h>
#    include <io.h>
#else
#    include <termios.h>
#    include <unistd.h>
#endif

#include <cstdlib>
#include <iostream>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace Slic3r::Rest {

namespace {

// Check options before reading credentials so usage errors never prompt.
struct PasswordOptions {
    std::string username;
    bool from_stdin = false;
};
PasswordOptions parse_options(int argc, char **argv, const std::string &invocation);
// Production reader retains the interactive console behavior on each platform.
std::string read_password();
// Erase the owned plaintext on both successful hashing and exception paths.
void clear_password(std::string &password);

PasswordOptions parse_options(int argc, char **argv, const std::string &invocation)
{
    PasswordOptions options;
    bool has_username = false;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--username" && !has_username && index + 1 < argc) {
            options.username = argv[++index];
            if (options.username.rfind("--", 0) == 0)
                throw std::invalid_argument("--username requires a name, not an option");
            has_username = true;
        } else if (option == "--password-stdin" && !options.from_stdin) {
            options.from_stdin = true;
        } else {
            throw std::invalid_argument("usage: " + invocation + " --username NAME [--password-stdin]");
        }
    }
    if (!has_username)
        throw std::invalid_argument("usage: " + invocation + " --username NAME [--password-stdin]");
    normalize_username(options.username);
    return options;
}

// Read a password without echo. The Windows path reads the console directly;
// the POSIX path restores the terminal settings after the line.
std::string read_password()
{
    std::cerr << "Password: " << std::flush;
    std::string password;
#ifdef _WIN32
    password = read_ascii_console_password([]() { return _getch(); });
#else
    termios original{};
    if (tcgetattr(STDIN_FILENO, &original) == 0) {
        termios hidden = original;
        hidden.c_lflag &= static_cast<tcflag_t>(~ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &hidden);
        std::getline(std::cin, password);
        tcsetattr(STDIN_FILENO, TCSANOW, &original);
    } else {
        std::getline(std::cin, password);
    }
#endif
    std::cerr << '\n';
    return password;
}

void clear_password(std::string &password)
{
    volatile char *characters = password.empty() ? nullptr : password.data();
    for (std::size_t index = 0; index < password.size(); ++index)
        characters[index] = '\0';
    password.clear();
}

} // namespace

std::string read_ascii_console_password(const std::function<int()> &read_character)
{
    std::string password;
    try {
        for (;;) {
            const int character = read_character();
            if (character == '\r' || character == '\n')
                return password;
            if (character == 8) {
                if (!password.empty())
                    password.pop_back();
                continue;
            }
            if (character == 3)
                throw std::runtime_error("password input interrupted");
            // Never hash a silently truncated password from an unsupported key.
            if (character < 32 || character > 126)
                throw std::invalid_argument("Windows interactive passwords support printable ASCII only; use UTF-8 --password-stdin for other characters");
            password.push_back(static_cast<char>(character));
        }
    } catch (...) {
        clear_password(password);
        throw;
    }
}

int run_password_tool(int argc, char **argv, const std::function<std::string()> &reader,
    std::ostream &out, std::ostream &err, const std::string &host_invocation,
    std::istream *password_input, bool input_is_console)
{
    const std::string invocation = host_invocation.empty() ? argv[0] : host_invocation;
    try {
        const PasswordOptions options = parse_options(argc, argv, invocation);
        std::string password;
        if (options.from_stdin) {
            if (input_is_console)
                throw std::invalid_argument("--password-stdin requires redirected input, not a console");
            if (password_input == nullptr || !std::getline(*password_input, password))
                throw std::invalid_argument("cannot read password from standard input");
            if (!password.empty() && password.back() == '\r')
                password.pop_back();
        } else {
            password = reader();
        }
        if (password.empty())
            throw std::invalid_argument("password must not be empty");
        std::string encoded_hash;
        try {
            encoded_hash = Slic3r::Rest::hash_password(password);
        } catch (...) {
            clear_password(password);
            throw;
        }
        clear_password(password);
        const nlohmann::json document = {
            {"users", {{options.username, {{"password_hash", encoded_hash}}}}}
        };
        out << document.dump(4) << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        err << invocation << ": " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

int rest_password_main(int argc, char **argv, const std::string &invocation)
{
#ifdef _WIN32
    const bool console = _isatty(_fileno(stdin)) != 0;
#else
    const bool console = isatty(STDIN_FILENO) != 0;
#endif
    return run_password_tool(argc, argv, read_password, std::cout, std::cerr, invocation, &std::cin, console);
}

} // namespace Slic3r::Rest
