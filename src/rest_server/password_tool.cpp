/*
 * Local REST account provisioning tool.
 *
 * The username is supplied as an option and the password is read without
 * terminal echo. Only a complete accounts JSON document containing the
 * Argon2id hash is written to standard output.
 */

#include "auth.hpp"

#include <nlohmann/json.hpp>

#ifdef _WIN32
#    include <conio.h>
#else
#    include <termios.h>
#    include <unistd.h>
#endif

#include <cstdlib>
#include <iostream>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace {

std::string required_username(int argc, char **argv)
{
    if (argc != 3 || std::string(argv[1]) != "--username" || argv[2] == nullptr)
        throw std::invalid_argument("usage: slic3r_rest_password --username NAME");
    return argv[2];
}

// Read a password without echo. The Windows path reads the console directly;
// the POSIX path restores the terminal settings after the line.
std::string read_password()
{
    std::cerr << "Password: " << std::flush;
    std::string password;
#ifdef _WIN32
    for (;;) {
        const int character = _getch();
        if (character == '\r' || character == '\n')
            break;
        if (character == 8) {
            if (!password.empty())
                password.pop_back();
            continue;
        }
        if (character == 3)
            throw std::runtime_error("password input interrupted");
        if (character >= 32 && character <= 126)
            password.push_back(static_cast<char>(character));
    }
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

int main(int argc, char **argv)
{
    try {
        const std::string username = required_username(argc, argv);
        std::string password = read_password();
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
            {"users", {{username, {{"password_hash", encoded_hash}}}}}
        };
        std::cout << document.dump(4) << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "slic3r_rest_password: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
