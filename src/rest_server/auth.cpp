/*
 * REST account, password and token implementation.
 *
 * The account file is parsed and validated before a server can start. Password
 * input is copied only for the Argon2 call and cleared afterwards; bearer
 * tokens are generated from the operating system's secure random source.
 */

#include "auth.hpp"

#include <argon2.h>
#include <nlohmann/json.hpp>

#ifdef _WIN32
#    include <windows.h>
#    include <bcrypt.h>
#endif

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace Slic3r::Rest {

namespace {

// Sweep the rate-limit map when it grows past this many tracked usernames.
constexpr std::size_t max_tracked_windows = 1024;

// Best-effort wipe the optimizer is not allowed to elide.
void clear_bytes(void *data, std::size_t size)
{
    volatile unsigned char *bytes = static_cast<volatile unsigned char *>(data);
    while (size > 0) {
        *bytes = 0;
        ++bytes;
        --size;
    }
}

void clear_string(std::string &value)
{
    if (!value.empty())
        clear_bytes(value.data(), value.size());
    value.clear();
}

std::vector<std::uint8_t> secure_random_bytes(std::size_t size)
{
    std::vector<std::uint8_t> result(size);
    if (size == 0)
        return result;

#ifdef _WIN32
    if (size > static_cast<std::size_t>(std::numeric_limits<ULONG>::max()) ||
        BCryptGenRandom(
            nullptr,
            reinterpret_cast<PUCHAR>(result.data()),
            static_cast<ULONG>(size),
            BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0)
        throw std::runtime_error("could not obtain secure random bytes");
#else
    std::ifstream random_source("/dev/urandom", std::ios::binary);
    random_source.read(
        reinterpret_cast<char *>(result.data()),
        static_cast<std::streamsize>(result.size()));
    if (!random_source)
        throw std::runtime_error("could not obtain secure random bytes");
#endif
    return result;
}

std::string hexadecimal(const std::vector<std::uint8_t> &bytes)
{
    static constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(bytes.size() * 2);
    for (std::uint8_t byte : bytes) {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 0x0f]);
    }
    return result;
}

std::vector<std::string> split(const std::string &value, char separator)
{
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (start <= value.size()) {
        const std::size_t end = value.find(separator, start);
        if (end == std::string::npos) {
            fields.push_back(value.substr(start));
            break;
        }
        fields.push_back(value.substr(start, end - start));
        start = end + 1;
    }
    return fields;
}

bool parse_uint32(std::string_view text, std::uint32_t &value)
{
    if (text.empty())
        return false;
    std::uint64_t parsed = 0;
    for (char character : text) {
        if (character < '0' || character > '9')
            return false;
        parsed = parsed * 10 + static_cast<std::uint64_t>(character - '0');
        if (parsed > std::numeric_limits<std::uint32_t>::max())
            return false;
    }
    value = static_cast<std::uint32_t>(parsed);
    return true;
}

bool parse_prefixed_uint32(
    const std::string &field,
    const std::string &prefix,
    std::uint32_t &value)
{
    if (field.rfind(prefix, 0) != 0)
        return false;
    return parse_uint32(std::string_view(field).substr(prefix.size()), value);
}

bool valid_base64(
    std::string_view value,
    std::uint32_t minimum_bytes,
    std::uint64_t maximum_bytes)
{
    if (value.empty() || value.find('=') != std::string_view::npos || value.size() % 4 == 1)
        return false;
    for (char character : value) {
        const bool upper = character >= 'A' && character <= 'Z';
        const bool lower = character >= 'a' && character <= 'z';
        const bool digit = character >= '0' && character <= '9';
        if (!upper && !lower && !digit && character != '+' && character != '/')
            return false;
    }
    const std::size_t remainder = value.size() % 4;
    const std::uint64_t decoded_bytes = (value.size() / 4) * 3 +
        (remainder == 2 ? 1 : remainder == 3 ? 2 : 0);
    return decoded_bytes >= minimum_bytes && decoded_bytes <= maximum_bytes;
}

// Validate the Argon2id encoded-hash shape and its parameter bounds before the
// verifier can consume it.
void validate_encoded_hash(const std::string &encoded_hash)
{
    const std::vector<std::string> fields = split(encoded_hash, '$');
    if (fields.size() < 5 || fields.size() > 6 || !fields[0].empty() ||
        fields[1] != "argon2id")
        throw std::invalid_argument("password_hash must be an Argon2id hash");

    std::size_t parameters_index = 2;
    if (fields.size() == 6) {
        std::uint32_t version = 0;
        if (!parse_prefixed_uint32(fields[2], "v=", version) ||
            (version != ARGON2_VERSION_10 && version != ARGON2_VERSION_13))
            throw std::invalid_argument("password_hash has an invalid Argon2 version");
        parameters_index = 3;
    }

    const std::vector<std::string> parameters = split(fields[parameters_index], ',');
    if (parameters.size() != 3)
        throw std::invalid_argument("password_hash has invalid Argon2 parameters");
    std::uint32_t memory_kib = 0;
    std::uint32_t time_cost = 0;
    std::uint32_t parallelism = 0;
    if (!parse_prefixed_uint32(parameters[0], "m=", memory_kib) ||
        !parse_prefixed_uint32(parameters[1], "t=", time_cost) ||
        !parse_prefixed_uint32(parameters[2], "p=", parallelism) ||
        memory_kib < ARGON2_MIN_MEMORY ||
        memory_kib < parallelism * 8U ||
        memory_kib > Argon2Parameters::max_memory_kib ||
        time_cost < ARGON2_MIN_TIME ||
        time_cost > Argon2Parameters::max_time_cost ||
        parallelism < ARGON2_MIN_LANES ||
        parallelism > Argon2Parameters::max_parallelism)
        throw std::invalid_argument("password_hash has invalid Argon2 parameters");

    const std::size_t salt_index = parameters_index + 1;
    if (!valid_base64(
            fields[salt_index],
            ARGON2_MIN_SALT_LENGTH,
            ARGON2_MAX_SALT_LENGTH) ||
        !valid_base64(
            fields[salt_index + 1],
            ARGON2_MIN_OUTLEN,
            ARGON2_MAX_OUTLEN))
        throw std::invalid_argument("password_hash has invalid salt or digest data");
}

} // namespace

void clear_secret(std::string &secret)
{
    clear_string(secret);
}

std::string normalize_username(const std::string &username)
{
    if (username.empty())
        throw std::invalid_argument("username must not be empty");
    if (username.size() > max_username_length)
        throw std::invalid_argument("username is too long");

    std::string normalized = username;
    for (char &character : normalized) {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (byte < 0x21 || byte > 0x7e)
            throw std::invalid_argument("username must contain printable ASCII characters");
        if (character >= 'A' && character <= 'Z')
            character = static_cast<char>(character - 'A' + 'a');
    }
    return normalized;
}

std::string hash_password(const std::string &password, const Argon2Parameters &parameters)
{
    if (parameters.memory_kib < ARGON2_MIN_MEMORY ||
        parameters.memory_kib < parameters.parallelism * 8U ||
        parameters.memory_kib > Argon2Parameters::max_memory_kib ||
        parameters.time_cost < ARGON2_MIN_TIME ||
        parameters.time_cost > Argon2Parameters::max_time_cost ||
        parameters.parallelism < ARGON2_MIN_LANES ||
        parameters.parallelism > Argon2Parameters::max_parallelism ||
        parameters.salt_bytes < ARGON2_MIN_SALT_LENGTH ||
        parameters.salt_bytes > ARGON2_MAX_SALT_LENGTH ||
        parameters.hash_bytes < ARGON2_MIN_OUTLEN ||
        parameters.hash_bytes > ARGON2_MAX_OUTLEN ||
        password.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("invalid Argon2id parameters");

    std::vector<std::uint8_t> salt = secure_random_bytes(parameters.salt_bytes);
    const std::size_t encoded_size = argon2_encodedlen(
        parameters.time_cost,
        parameters.memory_kib,
        parameters.parallelism,
        parameters.salt_bytes,
        parameters.hash_bytes,
        Argon2_id);
    std::vector<char> encoded(encoded_size + 1, '\0');
    std::string password_copy = password;
    const int result = argon2id_hash_encoded(
        parameters.time_cost,
        parameters.memory_kib,
        parameters.parallelism,
        password_copy.data(),
        password_copy.size(),
        salt.data(),
        salt.size(),
        parameters.hash_bytes,
        encoded.data(),
        encoded.size());
    clear_string(password_copy);
    clear_bytes(salt.data(), salt.size());
    if (result != ARGON2_OK) {
        clear_bytes(encoded.data(), encoded.size());
        throw std::runtime_error(
            std::string("Argon2id password hashing failed: ") +
            argon2_error_message(result));
    }

    std::string result_hash(encoded.data());
    clear_bytes(encoded.data(), encoded.size());
    return result_hash;
}

bool verify_password(const std::string &password, const std::string &encoded_hash)
{
    if (password.size() > std::numeric_limits<std::uint32_t>::max() || encoded_hash.empty())
        return false;
    std::string password_copy = password;
    const int result = argon2id_verify(
        encoded_hash.c_str(),
        password_copy.data(),
        password_copy.size());
    clear_string(password_copy);
    return result == ARGON2_OK;
}

AccountStore::AccountStore(const std::filesystem::path &accounts_file)
{
    std::ifstream input(accounts_file);
    if (!input)
        throw std::invalid_argument("could not open accounts file: " + accounts_file.string());

    nlohmann::json document;
    try {
        input >> document;
    } catch (const nlohmann::json::parse_error &error) {
        throw std::invalid_argument(std::string("invalid accounts JSON: ") + error.what());
    }
    if (!document.is_object() || !document.contains("users") || !document.at("users").is_object())
        throw std::invalid_argument("accounts JSON must contain an object named users");

    const nlohmann::json &users = document.at("users");
    for (nlohmann::json::const_iterator iterator = users.begin(); iterator != users.end(); ++iterator) {
        const std::string normalized = normalize_username(iterator.key());
        if (password_hashes_.find(normalized) != password_hashes_.end())
            throw std::invalid_argument("duplicate account username: " + iterator.key());
        if (!iterator.value().is_object() ||
            !iterator.value().contains("password_hash") ||
            !iterator.value().at("password_hash").is_string())
            throw std::invalid_argument("account " + iterator.key() + " has no password_hash string");

        const std::string encoded_hash = iterator.value().at("password_hash").get<std::string>();
        validate_encoded_hash(encoded_hash);
        password_hashes_.emplace(normalized, encoded_hash);
    }
}

bool AccountStore::verify(const std::string &username, const std::string &password) const
{
    std::string normalized;
    try {
        normalized = normalize_username(username);
    } catch (const std::invalid_argument &) {
        return false;
    }
    const std::map<std::string, std::string>::const_iterator iterator = password_hashes_.find(normalized);
    if (iterator == password_hashes_.end())
        return false;
    return verify_password(password, iterator->second);
}

std::size_t AccountStore::size() const noexcept
{
    return password_hashes_.size();
}

LoginRateLimiter::LoginRateLimiter(std::chrono::seconds window, std::uint32_t max_attempts)
    : window_(window), max_attempts_(max_attempts)
{
    if (window_.count() <= 0 || max_attempts_ == 0)
        throw std::invalid_argument("login rate limit must be positive");
}

bool LoginRateLimiter::allow(const std::string &normalized_username, AuthTimePoint now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Amortized cleanup: without it, an attacker sending distinct usernames to
    // the unauthenticated login route would grow the map for the whole server
    // lifetime.
    if (windows_.size() >= max_tracked_windows) {
        for (std::unordered_map<std::string, Window>::iterator iterator = windows_.begin();
             iterator != windows_.end();) {
            if (now >= iterator->second.started + window_)
                iterator = windows_.erase(iterator);
            else
                ++iterator;
        }
    }

    std::unordered_map<std::string, Window>::iterator iterator = windows_.find(normalized_username);
    if (iterator == windows_.end() || now >= iterator->second.started + window_) {
        windows_[normalized_username] = Window{now, 1};
        return true;
    }
    if (iterator->second.attempts >= max_attempts_)
        return false;
    ++iterator->second.attempts;
    return true;
}

std::size_t LoginRateLimiter::tracked_users() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return windows_.size();
}

TokenStore::TokenStore(std::chrono::seconds ttl)
    : ttl_(ttl)
{
    if (ttl_.count() <= 0)
        throw std::invalid_argument("token TTL must be positive");
}

IssuedToken TokenStore::issue(const std::string &username, AuthTimePoint now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Expired entries are otherwise only dropped when their token is presented
    // again, so purge them while issuing a new one.
    for (std::unordered_map<std::string, IssuedToken>::iterator iterator = tokens_.begin();
         iterator != tokens_.end();) {
        if (iterator->second.expires_at <= now)
            iterator = tokens_.erase(iterator);
        else
            ++iterator;
    }

    std::string value;
    do {
        value = hexadecimal(secure_random_bytes(32));
    } while (tokens_.find(value) != tokens_.end());

    IssuedToken token{value, username, now + ttl_};
    tokens_.emplace(token.value, token);
    return token;
}

std::optional<IssuedToken> TokenStore::find(const std::string &token, AuthTimePoint now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, IssuedToken>::iterator iterator = tokens_.find(token);
    if (iterator == tokens_.end())
        return std::nullopt;
    if (iterator->second.expires_at <= now) {
        tokens_.erase(iterator);
        return std::nullopt;
    }
    return iterator->second;
}

bool TokenStore::revoke(const std::string &token, AuthTimePoint now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, IssuedToken>::iterator iterator = tokens_.find(token);
    if (iterator == tokens_.end())
        return false;
    if (iterator->second.expires_at <= now) {
        tokens_.erase(iterator);
        return false;
    }
    tokens_.erase(iterator);
    return true;
}

std::size_t TokenStore::token_count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return tokens_.size();
}

AuthService::AuthService(const std::filesystem::path &accounts_file, const AuthConfig &config)
    : config_(config),
      accounts_(accounts_file),
      rate_limiter_(config.login_window, config.max_login_attempts),
      tokens_(config.token_ttl)
{
}

LoginResult AuthService::login(
    const std::string &username,
    const std::string &password,
    AuthTimePoint now)
{
    std::string normalized;
    try {
        normalized = normalize_username(username);
    } catch (const std::invalid_argument &) {
        return LoginResult{LoginStatus::invalid_credentials, std::nullopt};
    }
    if (!rate_limiter_.allow(normalized, now))
        return LoginResult{LoginStatus::rate_limited, std::nullopt};
    if (!accounts_.verify(normalized, password))
        return LoginResult{LoginStatus::invalid_credentials, std::nullopt};
    return LoginResult{
        LoginStatus::success,
        tokens_.issue(normalized, now)
    };
}

std::optional<std::string> AuthService::username_for_token(
    const std::string &token,
    AuthTimePoint now)
{
    const std::optional<IssuedToken> issued = tokens_.find(token, now);
    if (!issued.has_value())
        return std::nullopt;
    return issued->username;
}

bool AuthService::revoke_token(const std::string &token, AuthTimePoint now)
{
    return tokens_.revoke(token, now);
}

std::size_t AuthService::account_count() const noexcept
{
    return accounts_.size();
}

std::chrono::seconds AuthService::login_window() const noexcept
{
    return config_.login_window;
}

} // namespace Slic3r::Rest
