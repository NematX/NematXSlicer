// Launch tests exercise the shared parsers/lifecycle before application CLI setup.
// Real loopback requests validate emitted accounts and repeated signal shutdown;
// injected input avoids reading the person's console or supplying CLI passwords.
#include <catch_main.hpp>

#include "rest_main_internal.hpp"
#include "rest_server_fixture.hpp"

#include <csignal>
#include <iostream>
#include <sstream>

using namespace RestServerTest;
using namespace Slic3r::Rest;

namespace {

// Mutable UTF-8 argv storage, valid for the complete entry-point invocation.
struct Arguments {
    std::vector<std::string> strings;
    std::vector<char *> pointers;
    explicit Arguments(std::vector<std::string> values) : strings(std::move(values))
    {
        for (std::string &value : strings)
            pointers.push_back(value.data());
        pointers.push_back(nullptr);
    }
    int count() const { return static_cast<int>(strings.size()); }
};

// Help/errors are captured without leaking altered streams to subsequent tests.
struct Capture {
    std::ostringstream out, err;
    std::streambuf *old_out = std::cout.rdbuf(out.rdbuf());
    std::streambuf *old_err = std::cerr.rdbuf(err.rdbuf());
    ~Capture()
    {
        std::cout.rdbuf(old_out);
        std::cerr.rdbuf(old_err);
    }
};

// Signals are process-global; restore the test process even on an assertion.
struct PreviousSignals {
    using Handler = void (*)(int);
    Handler interrupt, terminate;
    explicit PreviousSignals(Handler handler)
#ifdef _WIN32
    Handler console_break;
#endif
        : interrupt(std::signal(SIGINT, handler)), terminate(std::signal(SIGTERM, handler))
    {
#ifdef _WIN32
        console_break = std::signal(SIGBREAK, handler);
#endif
    }
    ~PreviousSignals()
    {
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, terminate);
    }
#ifdef _WIN32
        std::signal(SIGBREAK, console_break);
#endif
};

volatile std::sig_atomic_t observed_signal = 0;
// This handler must survive the server's temporary signal-handler installation.
void prior_signal(int signal);
// Send one real HTTP login directly to the port reported by the launch callback.
HttpResponseMessage login_at(std::uint16_t port, const std::string &host = "127.0.0.1");

void prior_signal(int signal)
{
    observed_signal = signal;
}

HttpResponseMessage login_at(std::uint16_t port, const std::string &host)
{
    asio::io_context context;
    beast::tcp_stream stream(context);
    stream.connect(Tcp::endpoint(asio::ip::make_address(host), port));
    stream.expires_after(std::chrono::seconds(300));
    http::request<http::string_body> request(http::verb::post, "/login", 11);
    request.set(http::field::host, host.find(':') == std::string::npos ? host : '[' + host + ']');
    request.set(http::field::content_type, "application/json");
    request.body() = R"({"username":"Alice","password":"secret"})";
    request.prepare_payload();
    beast::flat_buffer buffer;
    HttpResponseMessage response;
    ErrorCode error;
    http::async_write(stream, request, [&](ErrorCode written, std::size_t) {
        if (written) {
            error = written;
            return;
        }
        http::async_read(stream, buffer, response, [&](ErrorCode read, std::size_t) { error = read; });
    });
    context.run();
    REQUIRE_FALSE(error);
    return response;
}

} // namespace

TEST_CASE("REST launch account writer produces a document accepted by HTTP", "[rest_launch]")
{
    Arguments args({"test", "--username", "Alice"});
    std::ostringstream out, err;
    REQUIRE(run_password_tool(args.count(), args.pointers.data(), []() { return "secret"; }, out, err) == 0);
    CHECK(err.str().empty());
    const nlohmann::json document = nlohmann::json::parse(out.str());
    REQUIRE(document.size() == 1);
    REQUIRE(document.at("users").size() == 1);
    REQUIRE(document.at("users").at("Alice").size() == 1);
    REQUIRE(document.at("users").at("Alice").at("password_hash").is_string());
    TempDirectory root;
    const std::filesystem::path accounts = root.path() / "accounts.json";
    std::ofstream(accounts) << out.str();
    AccountStore store(accounts);
    CHECK(store.verify("Alice", "secret"));
    CHECK_FALSE(store.verify("Alice", "wrong"));
    RestServerFixture fixture({}, true, false, "work", out.str());
    REQUIRE(fixture.request_without_auth(http::verb::post, "/login",
        R"({"username":"Alice","password":"secret"})", "application/json").result() == http::status::ok);
}

TEST_CASE("REST launch account errors emit no document", "[rest_launch]")
{
    for (const std::vector<std::string> &values : std::vector<std::vector<std::string>>{
        {"test"}, {"test", "--username", "Alice", "extra"}, {"test", "--username", "Alice"}}) {
        Arguments args(values);
        std::ostringstream out, err;
        REQUIRE(run_password_tool(args.count(), args.pointers.data(), []() { return ""; }, out, err) == 1);
        CHECK(out.str().empty());
        CHECK_FALSE(err.str().empty());
    }
}

TEST_CASE("REST launch help options and resource-default precedence", "[rest_launch]")
{
    Capture capture;
    Arguments help({"test", "--help"});
    REQUIRE(rest_server_main(help.count(), help.pointers.data()) == 0);
    CHECK(capture.out.str().find("--config") != std::string::npos);
    CHECK(capture.out.str().find("Usage: test [options]") != std::string::npos);
    RestLaunchDefaults host_defaults;
    host_defaults.invocation = "host --rest-server";
    REQUIRE(rest_server_main(help.count(), help.pointers.data(), host_defaults) == 0);
    CHECK(capture.out.str().find("Usage: host --rest-server [options]") != std::string::npos);
    Arguments host_error({"test", "--unknown"});
TEST_CASE("REST Windows console input refuses unsupported keys without hashing a truncated password", "[rest_launch]")
{
    for (const std::vector<int> &keys : std::vector<std::vector<int>>{
        {'p', 'a', 'x', 8, 's', 's', '\r'}, {'p', 0xe4, 's', 's', '\r'},
        {'p', 0, '\r'}, {'p', 224, '\r'}, {'p', 3}, {'p', -1}}) {
        std::size_t index = 0;
        Arguments args({"test", "--username", "Alice"});
        std::ostringstream out, err;
        const int result = run_password_tool(args.count(), args.pointers.data(), [&]() {
            return read_ascii_console_password([&]() {
                REQUIRE(index < keys.size());
                return keys[index++];
            });
        }, out, err);
        if (keys[1] == 'a') {
            REQUIRE(result == 0);
            CHECK(err.str().empty());
            CHECK(verify_password("pass", nlohmann::json::parse(out.str())["users"]["Alice"]["password_hash"]));
        } else {
            CHECK(result == 1);
            CHECK(out.str().empty());
            CHECK_FALSE(err.str().empty());
            CHECK((err.str().find("password input interrupted") != std::string::npos ||
                err.str().find("printable ASCII only") != std::string::npos));
        }
    }
}

TEST_CASE("REST password UTF-8 stdin preserves the password accepted by HTTP login", "[rest_launch]")
{
    const std::string password = u8"pässword";
    Arguments args({"test", "--username", "Alice", "--password-stdin"});
    std::istringstream input(password + "\n");
    std::ostringstream out, err;
    REQUIRE(run_password_tool(args.count(), args.pointers.data(), []() { return "unused"; },
        out, err, {}, &input, false) == 0);
    CHECK(err.str().empty());
    // This account uses its own password, not the fixture's automatic secret login.
    RestServerFixture fixture({}, false, false, "work", out.str());
    CHECK(fixture.request_without_auth(http::verb::post, "/login",
        nlohmann::json{{"username", "Alice"}, {"password", password}}.dump(), "application/json").result() == http::status::ok);
    CHECK(fixture.request_without_auth(http::verb::post, "/login",
        R"({"username":"Alice","password":"pssword"})", "application/json").result() == http::status::unauthorized);
}

TEST_CASE("REST password username cannot consume a following option", "[rest_launch]")
{
    for (const std::vector<std::string> &values : std::vector<std::vector<std::string>>{
        {"test", "--username", "--password-stdin"}, {"test", "--username", "--Alice"}}) {
        Arguments args(values);
        std::ostringstream out, err;
        bool prompted = false;
        CHECK(run_password_tool(args.count(), args.pointers.data(), [&]() {
            prompted = true;
            return "secret";
        }, out, err) == 1);
        CHECK_FALSE(prompted);
        CHECK(out.str().empty());
        CHECK(err.str().find("--username requires a name") != std::string::npos);
    }
}

TEST_CASE("REST password stdin reads one line without echo and refuses consoles or duplicate options", "[rest_launch]")
{
    for (const std::vector<std::string> &values : std::vector<std::vector<std::string>>{
        {"test", "--username", "Alice", "--password-stdin"},
        {"test", "--password-stdin", "--username", "Alice"}}) {
        Arguments args(values);
        std::istringstream input("secret\r\nsecond\n");
        std::ostringstream out, err;
        bool console_reader_called = false;
        REQUIRE(run_password_tool(args.count(), args.pointers.data(), [&]() {
            console_reader_called = true;
            return "unused";
        }, out, err, {}, &input, false) == 0);
        CHECK_FALSE(console_reader_called);
        CHECK(err.str().empty());
        CHECK(out.str().find("secret") == std::string::npos);
        const nlohmann::json document = nlohmann::json::parse(out.str());
        CHECK(verify_password("secret", document.at("users").at("Alice").at("password_hash")));
        CHECK_FALSE(verify_password("secret\r", document.at("users").at("Alice").at("password_hash")));
        std::string remaining;
        std::getline(input, remaining);
        CHECK(remaining == "second");
        std::ostringstream console_out, console_err;
        CHECK(run_password_tool(args.count(), args.pointers.data(), []() { return "unused"; },
            console_out, console_err, {}, &input, true) == 1);
        CHECK(console_out.str().empty());
        CHECK(console_err.str().find("not a console") != std::string::npos);
    }
    for (const std::vector<std::string> &values : std::vector<std::vector<std::string>>{
        {"test", "--username", "Alice", "--password-stdin", "--password-stdin"},
        {"test", "--username", "Alice", "--username", "Bob"},
        {"test", "--username", "Alice", "--other"},
        {"test", "--username", "Alice", "--password-stdin"}}) {
        Arguments args(values);
        for (const std::string &text : {std::string(), std::string("\n")}) {
            std::istringstream input(text);
            std::ostringstream out, err;
            CHECK(run_password_tool(args.count(), args.pointers.data(), []() { return "unused"; },
                out, err, {}, &input, false) == 1);
            CHECK(out.str().empty());
        }
    }
}

    CHECK(rest_server_main(host_error.count(), host_error.pointers.data(), host_defaults) == 1);
    CHECK(capture.err.str().find("host --rest-server: unknown option") != std::string::npos);
    for (const std::vector<std::string> &values : std::vector<std::vector<std::string>>{
        {"test", "--unknown"}, {"test", "--port", "70000"}, {"test", "--port", "-1"}}) {
        Arguments invalid(values);
        CHECK(rest_server_main(invalid.count(), invalid.pointers.data()) == 1);
    }
    TempDirectory root;
    const ServerConfig valid = make_valid_config(root.path());
    const std::filesystem::path config_file = root.path() / "server.json";
    nlohmann::json document = {{"paths", {{"profiles", valid.profiles_directory.generic_u8string()},
        {"work", valid.work_directory.generic_u8string()}, {"accounts", valid.accounts_file.generic_u8string()}}}};
    std::ofstream(config_file) << document.dump();
    const RestLaunchDefaults defaults{valid.resource_directory};
    CHECK(load_server_config(config_file, {}, defaults.resource_directory).resource_directory == valid.resource_directory);
    const std::filesystem::path configured = root.path() / "configured";
    const std::filesystem::path explicit_path = root.path() / "explicit";
    std::filesystem::create_directory(configured);
    std::filesystem::create_directory(explicit_path);
    document["paths"]["resources"] = configured.generic_u8string();
    std::ofstream(config_file) << document.dump();
    CHECK(load_server_config(config_file, {}, defaults.resource_directory).resource_directory == configured);
    CommandLineOverrides overrides;
    overrides.resource_directory = explicit_path;
    CHECK(load_server_config(config_file, overrides, defaults.resource_directory).resource_directory == explicit_path);
}

TEST_CASE("REST launch accepts UTF-8 work paths and restores shutdown signals", "[rest_launch]")
{
    // The engine owns these paths for the process lifetime and cannot be reset.
    // Retain the root across all launches and any test-case reentry.
    static TempDirectory root;
    ServerConfig config = make_valid_config(root.path());
    Capture lifecycle;
    config.resource_directory = SLIC3R_REST_RESOURCE_DIRECTORY;
    const std::filesystem::path work = root.path() / std::filesystem::u8path(u8"travail-é");
    const std::filesystem::path config_file = root.path() / "server.json";
    Arguments account_args({"test", "--username", "Alice"});
    std::ostringstream accounts, errors;
    REQUIRE(run_password_tool(account_args.count(), account_args.pointers.data(), []() { return "secret"; }, accounts, errors) == 0);
    std::ofstream(config.accounts_file) << accounts.str();
    const nlohmann::json document = {{"listen", {{"port", 0}}}, {"paths", {
        {"resources", config.resource_directory.generic_u8string()},
        {"profiles", config.profiles_directory.generic_u8string()},
        {"accounts", config.accounts_file.generic_u8string()}}}};
    std::ofstream(config_file) << document.dump();
    Arguments args({"rest_launch_test", "--config", config_file.u8string(), "--work-dir", work.u8string()});
    PreviousSignals previous(prior_signal);
    const std::vector<int> shutdown_signals{SIGINT, SIGTERM
#ifdef _WIN32
        , SIGBREAK
#endif
    };
    for (const int shutdown_signal : shutdown_signals) {
        bool started = false;
        REQUIRE(run_rest_server(args.count(), args.pointers.data(), {}, [&](std::uint16_t port) {
            started = true;
            REQUIRE(port != 0);
            REQUIRE(login_at(port).result() == http::status::ok);
            // The synthetic argv[0] must not become an invented engine path.
            const std::filesystem::path binary(Slic3r::binary_file().native());
            CHECK(binary.is_absolute());
            CHECK(std::filesystem::is_regular_file(binary));
            CHECK(binary.stem() == "rest_launch_tests");
            REQUIRE(std::raise(shutdown_signal) == 0);
        }) == 0);
        REQUIRE(started);
        CHECK(std::signal(SIGINT, prior_signal) == prior_signal);
        CHECK(std::signal(SIGTERM, prior_signal) == prior_signal);
        observed_signal = 0;
#ifdef _WIN32
        CHECK(std::signal(SIGBREAK, prior_signal) == prior_signal);
#endif
        REQUIRE(std::raise(SIGINT) == 0);
        CHECK(observed_signal == SIGINT);
        // The Windows CRT resets a delivered handler to SIG_DFL; rearm the test
        // handler before the next launch captures the surrounding process state.
        std::signal(SIGINT, prior_signal);
    }
    REQUIRE(std::filesystem::is_directory(work / "engine_data"));
    // The same process-global engine paths also serve an IPv6 listener.
    nlohmann::json ipv6_document = document;
    ipv6_document["listen"]["address"] = "::1";
    std::ofstream(config_file) << ipv6_document.dump();
    REQUIRE(run_rest_server(args.count(), args.pointers.data(), {}, [&](std::uint16_t port) {
        REQUIRE(login_at(port, "::1").result() == http::status::ok);
        REQUIRE(std::raise(SIGTERM) == 0);
    }) == 0);
    CHECK(lifecycle.out.str().find("listening on [::1]:") != std::string::npos);
    for (const std::filesystem::directory_entry &entry : std::filesystem::recursive_directory_iterator(work)) {
    CHECK(lifecycle.out.str().find("listening on 127.0.0.1:") != std::string::npos);
    CHECK(lifecycle.out.str().find("stopping\nstopped\n") != std::string::npos);
    CHECK(lifecycle.out.str().find("secret") == std::string::npos);
        const std::string name = entry.path().filename().string();
        CHECK(name.rfind("rest-upload-", 0) != 0);
        CHECK(name.rfind("rest-export-", 0) != 0);
        CHECK(name.rfind("rest-result-", 0) != 0);
    }
}
