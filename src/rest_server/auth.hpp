/*
 * REST authentication primitives.
 *
 * Account hashes are loaded from the administrator-managed JSON file. Password
 * hashing uses Argon2id, login attempts use a fixed per-user window, and tokens
 * are kept in memory with independently expiring and revocable entries.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace Slic3r::Rest {

using AuthTimePoint = std::chrono::system_clock::time_point;

struct Argon2Parameters {
    std::uint32_t memory_kib = 64U * 1024U;
    std::uint32_t time_cost = 3;
    std::uint32_t parallelism = 1;
    std::uint32_t hash_bytes = 32;
    std::uint32_t salt_bytes = 16;

    // Upper bounds accepted from account files and hashing parameters: a
    // malformed entry must not make the server allocate or compute unbounded
    // work.
    static constexpr std::uint32_t max_memory_kib = 1024U * 1024U;
    static constexpr std::uint32_t max_time_cost = 16;
    static constexpr std::uint32_t max_parallelism = 16;
};

// Longest accepted username, applied to account files and login requests.
constexpr std::size_t max_username_length = 64;

struct AuthConfig {
    std::chrono::seconds token_ttl{24 * 60 * 60};
    std::chrono::seconds login_window{60};
    std::uint32_t max_login_attempts = 5;
};

std::string normalize_username(const std::string &username);
// Best-effort wipe of a secret string before it is released.
void clear_secret(std::string &secret);
std::string hash_password(
    const std::string &password,
    const Argon2Parameters &parameters = {});
bool verify_password(const std::string &password, const std::string &encoded_hash);

class AccountStore {
public:
    explicit AccountStore(const std::filesystem::path &accounts_file);

    bool verify(const std::string &username, const std::string &password) const;
    std::size_t size() const noexcept;

private:
    std::map<std::string, std::string> password_hashes_;
};

class LoginRateLimiter {
public:
    LoginRateLimiter(std::chrono::seconds window, std::uint32_t max_attempts);

    bool allow(const std::string &normalized_username, AuthTimePoint now);
    std::size_t tracked_users() const;

private:
    struct Window {
        AuthTimePoint started;
        std::uint32_t attempts;
    };

    std::chrono::seconds window_;
    std::uint32_t max_attempts_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Window> windows_;
};

struct IssuedToken {
    std::string value;
    std::string username;
    AuthTimePoint expires_at;
};

class TokenStore {
public:
    explicit TokenStore(std::chrono::seconds ttl);

    IssuedToken issue(const std::string &username, AuthTimePoint now);
    std::optional<IssuedToken> find(const std::string &token, AuthTimePoint now);
    bool revoke(const std::string &token, AuthTimePoint now);
    std::size_t token_count() const;

private:
    std::chrono::seconds ttl_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, IssuedToken> tokens_;
};

enum class LoginStatus {
    success,
    invalid_credentials,
    rate_limited
};

struct LoginResult {
    LoginStatus status;
    std::optional<IssuedToken> token;
};

class AuthService {
public:
    AuthService(
        const std::filesystem::path &accounts_file,
        const AuthConfig &config = {});

    LoginResult login(
        const std::string &username,
        const std::string &password,
        AuthTimePoint now);
    std::optional<std::string> username_for_token(
        const std::string &token,
        AuthTimePoint now);
    bool revoke_token(const std::string &token, AuthTimePoint now);
    std::size_t account_count() const noexcept;
    std::chrono::seconds login_window() const noexcept;

private:
    AuthConfig config_;
    AccountStore accounts_;
    LoginRateLimiter rate_limiter_;
    TokenStore tokens_;
};

} // namespace Slic3r::Rest
