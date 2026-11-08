/*
 * Authentication tests for the REST server.
 *
 * These tests use small Argon2 parameters to keep the suite fast while
 * exercising account-file validation, case-insensitive usernames, fixed login
 * windows, token expiration and independent token revocation.
 */

#include <catch_main.hpp>

#include "auth.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace {

class TemporaryAccounts {
public:
    TemporaryAccounts()
    {
        static std::atomic<std::uint64_t> next_id{1};
        root_ = std::filesystem::temp_directory_path() /
            ("superslicer_rest_auth_" + std::to_string(next_id.fetch_add(1)));
        std::filesystem::create_directories(root_);
    }

    ~TemporaryAccounts()
    {
        std::error_code ignored;
        std::filesystem::remove_all(root_, ignored);
    }

    const std::filesystem::path &path() const { return root_; }

private:
    std::filesystem::path root_;
};

Slic3r::Rest::Argon2Parameters test_parameters()
{
    Slic3r::Rest::Argon2Parameters parameters;
    parameters.memory_kib = 8U * 1024U;
    parameters.time_cost = 2;
    parameters.parallelism = 1;
    parameters.hash_bytes = 16;
    parameters.salt_bytes = 16;
    return parameters;
}

std::string test_hash(const std::string &password)
{
    return Slic3r::Rest::hash_password(password, test_parameters());
}

void write_accounts(
    const std::filesystem::path &path,
    const std::string &username,
    const std::string &encoded_hash)
{
    const nlohmann::json document = {
        {"users", {{username, {{"password_hash", encoded_hash}}}}}
    };
    std::ofstream output(path);
    output << document.dump(2);
}

Slic3r::Rest::AuthTimePoint at_seconds(std::int64_t seconds)
{
    return Slic3r::Rest::AuthTimePoint{} + std::chrono::seconds(seconds);
}

// Replace the parameter field of an encoded hash while keeping its salt and
// digest, so malformed account entries share a verifiable hash shape.
std::string with_parameters(const std::string &encoded_hash, const std::string &parameters)
{
    const std::size_t version_separator = encoded_hash.find('$', encoded_hash.find('$', 1) + 1);
    const std::size_t parameters_end = encoded_hash.find('$', version_separator + 1);
    return encoded_hash.substr(0, version_separator + 1) + parameters +
        encoded_hash.substr(parameters_end);
}

} // namespace

TEST_CASE("Argon2id hashes verify without storing the password", "[rest_auth]")
{
    const std::string password = "correct horse battery staple";
    const std::string encoded_hash = test_hash(password);

    REQUIRE(encoded_hash.rfind("$argon2id$", 0) == 0);
    REQUIRE(encoded_hash.find(password) == std::string::npos);
    REQUIRE(Slic3r::Rest::verify_password(password, encoded_hash));
    REQUIRE(!Slic3r::Rest::verify_password("wrong password", encoded_hash));
}

TEST_CASE("Account files normalize usernames and reject malformed entries", "[rest_auth]")
{
    TemporaryAccounts temporary;
    const std::string encoded_hash = test_hash("secret");
    const std::filesystem::path accounts_file = temporary.path() / "accounts.json";
    write_accounts(accounts_file, "Alice", encoded_hash);

    const Slic3r::Rest::AccountStore accounts(accounts_file);
    REQUIRE(accounts.size() == 1);
    REQUIRE(accounts.verify("alice", "secret"));
    REQUIRE(accounts.verify("ALICE", "secret"));
    REQUIRE(!accounts.verify("alice", "wrong"));
    REQUIRE(!accounts.verify("unknown", "secret"));

    const std::filesystem::path duplicate_file = temporary.path() / "duplicate.json";
    std::ofstream duplicate_output(duplicate_file);
    duplicate_output << nlohmann::json{
        {"users", {
            {"Alice", {{"password_hash", encoded_hash}}},
            {"alice", {{"password_hash", encoded_hash}}}
        }}
    }.dump();
    duplicate_output.close();
    REQUIRE_THROWS_AS(
        Slic3r::Rest::AccountStore(duplicate_file),
        std::invalid_argument);

    const std::filesystem::path malformed_file = temporary.path() / "malformed.json";
    std::ofstream malformed_output(malformed_file);
    malformed_output << R"({"users":{"alice":{"password_hash":"not-a-hash"}}})";
    malformed_output.close();
    REQUIRE_THROWS_AS(
        Slic3r::Rest::AccountStore(malformed_file),
        std::invalid_argument);
}

TEST_CASE("Login attempts use a fixed five-attempt window", "[rest_auth]")
{
    TemporaryAccounts temporary;
    const std::filesystem::path accounts_file = temporary.path() / "accounts.json";
    write_accounts(accounts_file, "alice", test_hash("secret"));

    Slic3r::Rest::AuthConfig config;
    config.token_ttl = std::chrono::seconds(120);
    config.login_window = std::chrono::seconds(60);
    config.max_login_attempts = 5;
    Slic3r::Rest::AuthService service(accounts_file, config);

    for (int attempt = 0; attempt < 5; ++attempt) {
        const Slic3r::Rest::LoginResult result = service.login(
            "alice", "wrong", at_seconds(attempt));
        REQUIRE(result.status == Slic3r::Rest::LoginStatus::invalid_credentials);
    }
    REQUIRE(service.login("alice", "secret", at_seconds(5)).status ==
        Slic3r::Rest::LoginStatus::rate_limited);
    REQUIRE(service.login("alice", "secret", at_seconds(60)).status ==
        Slic3r::Rest::LoginStatus::success);
}

TEST_CASE("Tokens expire and revoke independently", "[rest_auth]")
{
    TemporaryAccounts temporary;
    const std::filesystem::path accounts_file = temporary.path() / "accounts.json";
    write_accounts(accounts_file, "Alice", test_hash("secret"));

    Slic3r::Rest::AuthConfig config;
    config.token_ttl = std::chrono::seconds(10);
    Slic3r::Rest::AuthService service(accounts_file, config);
    const Slic3r::Rest::LoginResult first = service.login("alice", "secret", at_seconds(0));
    const Slic3r::Rest::LoginResult second = service.login("ALICE", "secret", at_seconds(1));
    REQUIRE(first.status == Slic3r::Rest::LoginStatus::success);
    REQUIRE(second.status == Slic3r::Rest::LoginStatus::success);
    REQUIRE(first.token->value != second.token->value);
    REQUIRE(service.username_for_token(first.token->value, at_seconds(2)) == "alice");
    REQUIRE(service.revoke_token(first.token->value, at_seconds(2)));
    REQUIRE(!service.username_for_token(first.token->value, at_seconds(2)).has_value());
    REQUIRE(service.username_for_token(second.token->value, at_seconds(2)) == "alice");
    REQUIRE(!service.username_for_token(second.token->value, at_seconds(11)).has_value());
}

TEST_CASE("Account files reject unbounded Argon2 parameters", "[rest_auth]")
{
    TemporaryAccounts temporary;
    const std::string encoded_hash = test_hash("secret");

    const auto check_rejected = [&](const std::string &parameters) {
        const std::filesystem::path accounts_file =
            temporary.path() / ("rejected_" + std::to_string(parameters.size()) + ".json");
        write_accounts(accounts_file, "alice", with_parameters(encoded_hash, parameters));
        REQUIRE_THROWS_AS(Slic3r::Rest::AccountStore(accounts_file), std::invalid_argument);
    };
    check_rejected("m=" + std::to_string(Slic3r::Rest::Argon2Parameters::max_memory_kib + 1U) + ",t=3,p=1");
    check_rejected("m=8192,t=" + std::to_string(Slic3r::Rest::Argon2Parameters::max_time_cost + 1U) + ",p=1");
    check_rejected("m=8192,t=2,p=" + std::to_string(Slic3r::Rest::Argon2Parameters::max_parallelism + 1U));

    const std::filesystem::path accepted_file = temporary.path() / "accepted.json";
    write_accounts(accepted_file, "alice", with_parameters(encoded_hash, "m=65536,t=3,p=2"));
    REQUIRE(Slic3r::Rest::AccountStore(accepted_file).size() == 1);
}

TEST_CASE("Login limiter state is bounded and expired tokens are purged", "[rest_auth]")
{
    Slic3r::Rest::LoginRateLimiter limiter(std::chrono::seconds(60), 5);
    bool all_allowed = true;
    for (int index = 0; index < 2048; ++index)
        all_allowed = limiter.allow("user" + std::to_string(index), at_seconds(0)) && all_allowed;
    REQUIRE(all_allowed);
    REQUIRE(limiter.tracked_users() == 2048);

    // The sweep runs before inserting the new user once the window elapsed.
    REQUIRE(limiter.allow("alice", at_seconds(61)));
    REQUIRE(limiter.tracked_users() == 1);

    Slic3r::Rest::TokenStore tokens(std::chrono::seconds(10));
    tokens.issue("alice", at_seconds(0));
    REQUIRE(tokens.token_count() == 1);
    tokens.issue("alice", at_seconds(11));
    REQUIRE(tokens.token_count() == 1);

    REQUIRE_THROWS_AS(
        Slic3r::Rest::normalize_username(
            std::string(Slic3r::Rest::max_username_length + 1U, 'a')),
        std::invalid_argument);
}
