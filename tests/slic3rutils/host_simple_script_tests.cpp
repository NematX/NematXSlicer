// Executes the real filament layout script with a small model of the host's integer-vector bindings.
// Edited values belong to preset A; selected values are its saved values. Preset B is inaccessible to the VM and does not prove production preset isolation.
#define CATCH_CONFIG_RUNNER
#include <catch2/catch.hpp>
#include "angelscript/source/as_config.h"
#include "angelscript/include/angelscript.h"
#include "angelscript/add_on/scriptstdstring/scriptstdstring.h"
#ifdef AS_MAX_PORTABILITY
#include "angelscript/add_on/autowrapper/aswrappedcall.h"
#endif
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#ifdef _MSC_VER
#include <crtdbg.h>
#endif

using namespace AngelScript;
namespace {
using Values = std::map<std::string, std::vector<int>>;
struct Store {
    Values edited{{"temperature", {210, 310}}, {"first_layer_temperature", {220, 320}},
                  {"bed_temperature", {130, 130}}, {"first_layer_bed_temperature", {130, 130}}};
    Values selected{{"temperature", {200, 230}}, {"first_layer_temperature", {205, 235}},
                    {"bed_temperature", {60, 65}}, {"first_layer_bed_temperature", {55, 0}}};
    Values pending;
    std::set<std::string> resets;
    std::vector<std::string> writes, reset_log, errors;
    bool can_set = false;

    // Record a binding failure without unwinding C++ through the VM; the active context owns its exception text.
    void fail(const std::string &message)
    {
        errors.push_back(message);
        if (auto *context = asGetActiveContext()) context->SetException(message.c_str());
    }
    // Borrow a known nonempty integer vector, selecting saved values for pending resets; failures return null.
    const std::vector<int> *read(const std::string &key)
    {
        const Values &source = resets.count(key) ? selected : edited;
        auto it = source.find(key);
        if (it == source.end() || it->second.empty()) { fail("Unknown or empty integer vector " + key); return nullptr; }
        return &it->second;
    }
    // Read index zero without consulting deferred writes; return zero only after recording an invalid read.
    int get(std::string &key)
    {
        const auto *value = read(key);
        return value ? value->front() : 0;
    }
    // Broadcast into an owned deferred vector in setter mode, cancel its reset, and reject unknown or empty keys.
    void set(std::string &key, int value)
    {
        if (!can_set) return;
        const auto *source = read(key);
        if (!source) return;
        auto it = pending.find(key);
        std::vector<int> copy = it == pending.end() ? *source : it->second;
        std::fill(copy.begin(), copy.end(), value);
        pending.insert_or_assign(key, std::move(copy));
        resets.erase(key);
        writes.push_back(key);
    }
    // Queue a known saved vector for restoration after execution; failures are recorded rather than creating keys.
    void reset(std::string &key)
    {
        if (!read(key)) return;
        auto it = selected.find(key);
        if (it == selected.end() || it->second.empty()) { fail("Missing selected vector " + key); return; }
        resets.insert(key);
        reset_log.push_back(key);
    }
    // Apply deferred writes and clone complete saved reset vectors into edited A; the maps retain their own storage.
    void apply()
    {
        for (const auto &entry : pending) edited.at(entry.first) = entry.second;
        for (const auto &key : resets) edited.at(key) = selected.at(key);
        pending.clear();
        resets.clear();
    }
};
Store *active = nullptr;
// Forward a VM integer read to the borrowed active store; the harness keeps it alive throughout execution.
int get_int(std::string &key) { return active->get(key); }
// Forward a VM broadcast write to the borrowed active store without transferring ownership.
void set_int(std::string &key, int value) { active->set(key, value); }
// Forward a VM reset request to the borrowed active store without transferring ownership.
void back_initial_value(std::string &key) { active->reset(key); }

// Read an existing resource into owned text and normalize CRLF to LF; report unreadable files outside VM callbacks.
std::string read_file(const std::string &path)
{
    std::ifstream stream(path, std::ios::binary);
    REQUIRE(stream.good());
    std::string text{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
    for (size_t position = 0; (position = text.find("\r\n", position)) != std::string::npos; ++position)
        text.erase(position, 1);
    return text;
}
// Collect compilation warnings and errors in borrowed storage; informational messages are not failures.
void message(const asSMessageInfo *info, void *data)
{
    if (info->type != asMSGTYPE_INFORMATION)
        static_cast<std::vector<std::string> *>(data)->push_back(info->message);
}
struct EngineDeleter {
    // Release the owned engine and its module after all contexts have been destroyed.
    void operator()(asIScriptEngine *engine) const { engine->ShutDownAndRelease(); }
};
struct ContextDeleter {
    // Release an owned execution context before its engine is destroyed.
    void operator()(asIScriptContext *context) const { context->Release(); }
};
struct VM {
    std::vector<std::string> diagnostics;
    std::unique_ptr<asIScriptEngine, EngineDeleter> engine{asCreateScriptEngine()};
    asIScriptModule *module = nullptr;
    // Own an engine, register the three host signatures, and compile the complete explicit source resource or fail.
    explicit VM(const std::string &path)
    {
        REQUIRE(engine);
        REQUIRE(engine->SetMessageCallback(asFUNCTION(message), &diagnostics, asCALL_CDECL) >= 0);
        RegisterStdString(engine.get());
        REQUIRE(engine->GetTypeInfoByDecl("string"));
#ifdef AS_MAX_PORTABILITY
        REQUIRE(engine->RegisterGlobalFunction("int get_int(string &in)", WRAP_FN(get_int), asCALL_GENERIC) >= 0);
        REQUIRE(engine->RegisterGlobalFunction("void set_int(string &in, int new_val)", WRAP_FN(set_int), asCALL_GENERIC) >= 0);
        REQUIRE(engine->RegisterGlobalFunction("void back_initial_value(string &in)", WRAP_FN(back_initial_value), asCALL_GENERIC) >= 0);
#else
        REQUIRE(engine->RegisterGlobalFunction("int get_int(string &in)", asFUNCTION(get_int), asCALL_CDECL) >= 0);
        REQUIRE(engine->RegisterGlobalFunction("void set_int(string &in, int new_val)", asFUNCTION(set_int), asCALL_CDECL) >= 0);
        REQUIRE(engine->RegisterGlobalFunction("void back_initial_value(string &in)", asFUNCTION(back_initial_value), asCALL_CDECL) >= 0);
#endif
        module = engine->GetModule("filament", asGM_ALWAYS_CREATE);
        REQUIRE(module);
        const auto source = read_file(path);
        REQUIRE(module->AddScriptSection(path.c_str(), source.data(), source.size()) >= 0);
        const int result = module->Build();
        std::string diagnostic_text;
        for (const auto &diagnostic : diagnostics) diagnostic_text += diagnostic + "\n";
        INFO(diagnostic_text);
        REQUIRE(diagnostics.empty());
        REQUIRE(result >= 0);
        REQUIRE(module->GetFunctionCount() == 4);
        REQUIRE(module->GetGlobalVarCount() == 0);
        for (const auto *decl : {"int s_easy_temp_get()", "void s_easy_temp_set(int)",
                                "int s_bed_fil_fff_get(string &out)", "void s_bed_fil_fff_set(string &in, int idx)"})
            REQUIRE(module->GetFunctionByDecl(decl));
    }
    // Execute an exact declaration with checked arguments; own its context, apply changes only after success, and return GetReturnDWord as int for getters or zero for setters.
    int call(Store &store, const char *decl, bool setter, int value = 0, std::string label = {})
    {
        auto *function = module->GetFunctionByDecl(decl);
        REQUIRE(function);
        std::unique_ptr<asIScriptContext, ContextDeleter> context(engine->CreateContext());
        REQUIRE(context);
        REQUIRE(context->Prepare(function) >= 0);
        if (std::string(decl).find("string &in") != std::string::npos) {
            REQUIRE(context->SetArgAddress(0, &label) >= 0);
            REQUIRE(context->SetArgDWord(1, static_cast<asDWORD>(value)) >= 0);
        } else if (std::string(decl).find("string &out") != std::string::npos) {
            REQUIRE(context->SetArgObject(0, &label) >= 0);
        } else if (setter) REQUIRE(context->SetArgDWord(0, static_cast<asDWORD>(value)) >= 0);
        store.can_set = setter;
        active = &store;
        const int result = context->Execute();
        active = nullptr;
        store.can_set = false;
        INFO((context->GetExceptionString() ? context->GetExceptionString() : ""));
        REQUIRE(result == asEXECUTION_FINISHED);
        REQUIRE(store.errors.empty());
        if (std::string(decl).find("string &out") != std::string::npos)
            CHECK(label.empty());
        const int returned = setter ? 0 : static_cast<int>(context->GetReturnDWord());
        if (setter) store.apply();
        return returned;
    }
};
const char *temp_get = "int s_easy_temp_get()";
const char *temp_set = "void s_easy_temp_set(int)";
const char *bed_get = "int s_bed_fil_fff_get(string &out)";
const char *bed_set = "void s_bed_fil_fff_set(string &in, int idx)";
std::string script_path, ui_path;

// Require the exact enum token in borrowed UI text, then return its three known labels; missing or reordered tokens fail.
std::vector<std::string> bed_labels(const std::string &ui)
{
    const std::string token = "enum$custom$Custom$glue$With glue$noglue$Without glue";
    REQUIRE(ui.find(token) != std::string::npos);
    return {"Custom", "With glue", "Without glue"};
}
}

// Inspect owned resource text for page placement, widget bounds and visibility; failed assertions do not construct wx fields.
TEST_CASE("Simple filament resource layout", "[host-simple-filament]")
{
    const auto ui = read_file(ui_path);
    const auto spool = ui.find("group:filament_spool_weight_event:Filament");
    const auto simple = ui.find("\ngroup:Filament\n");
    const auto flow = ui.find("\ngroup:Flow\n");
    const auto speed = ui.find("\npage:Speed & Pressure advance:time");
    REQUIRE(spool != std::string::npos);
    REQUIRE(simple != std::string::npos);
    REQUIRE(flow != std::string::npos);
    REQUIRE(speed != std::string::npos);
    CHECK(spool < simple);
    CHECK(simple < flow);
    CHECK(flow < speed);
    CHECK(ui.find("setting:tags$Simple:id$0:extrusion_multiplier") != std::string::npos);
    CHECK(ui.find("setting:tags$Simple:script:depends$temperature$first_layer_temperature:int:max$1500:label$Temperature:") != std::string::npos);
    CHECK(ui.find("tooltip$Sets nozzle temperatures for the filament preset being edited (first layer and other layers).:s_easy_temp\n") != std::string::npos);
    CHECK(ui.find("setting:tags$Simple:script:depends$bed_temperature$first_layer_bed_temperature:enum$") != std::string::npos);
    CHECK(ui.find("tooltip$Sets bed temperatures for the filament preset being edited (first layer and other layers).:s_bed_fil_fff") != std::string::npos);
    CHECK(ui.find("all current filaments") == std::string::npos);
    CHECK(ui.find("this extruder") == std::string::npos);
    const std::string multiplier = ":extrusion_multiplier\n";
    const auto first_multiplier = ui.find(multiplier);
    REQUIRE(first_multiplier != std::string::npos);
    const auto second_multiplier = ui.find(multiplier, first_multiplier + multiplier.size());
    REQUIRE(second_multiplier != std::string::npos);
    CHECK(ui.find(multiplier, second_multiplier + multiplier.size()) == std::string::npos);
    for (const auto *key : {"filament_cost", "filament_spool_weight", "extrusion_multiplier", "first_layer_temperature", "temperature", "first_layer_bed_temperature"})
        CHECK(ui.find(std::string("setting:tags$Advanced$Expert$Prusa:id$0:") + key + "\n") != std::string::npos);
    CHECK(ui.find("setting:tags$Advanced$Expert$Prusa:id$0:label$Other layers:bed_temperature") != std::string::npos);
    bed_labels(ui);
}

// Execute the real script against owned model stores and assert reads, writes and resets; VM failures are checked outside callbacks.
TEST_CASE("Simple filament scripts execute real resources", "[host-simple-filament]")
{
    VM vm(script_path);
    const auto labels = bed_labels(read_file(ui_path));
    const Store preset_b;
    const auto unchanged_b = preset_b.edited;
    SECTION("Nozzle getters use index zero") {
        for (const auto pair : {std::pair<int,int>{210,220}, {230,220}, {220,220}, {0,0}}) {
            Store a;
            a.edited.at("temperature") = {pair.first, 999};
            a.edited.at("first_layer_temperature") = {pair.second, 998};
            const auto before = a.edited;
            CHECK(vm.call(a, temp_get, false) == std::max(pair.first, pair.second));
            CHECK(a.edited == before);
            CHECK(a.writes.empty());
        }
    }
    SECTION("Nozzle bounds and broadcast") {
        for (int value : {-1, 0, 225, 1500, 1501}) {
            Store a;
            a.edited.at("temperature") = {200, 230};
            const auto before = a.edited;
            vm.call(a, temp_set, true, value);
            if (value < 0 || value > 1500) {
                CHECK(a.edited == before);
                CHECK(a.writes.empty());
            } else {
                CHECK(a.writes == std::vector<std::string>{"temperature", "first_layer_temperature"});
                CHECK(a.edited.at("temperature") == std::vector<int>{value, value});
                CHECK(a.edited.at("first_layer_temperature") == std::vector<int>{value, value});
                CHECK(a.edited.at("bed_temperature") == before.at("bed_temperature"));
                CHECK(a.edited.at("first_layer_bed_temperature") == before.at("first_layer_bed_temperature"));
            }
            CHECK(a.reset_log.empty());
        }
    }
    SECTION("Bed getter uses integer indices") {
        for (const auto pair : {std::pair<int,int>{130,130}, {170,170}, {130,170}, {60,55}}) {
            Store a;
            a.edited.at("bed_temperature") = {pair.first, 999};
            a.edited.at("first_layer_bed_temperature") = {pair.second, 998};
            CHECK(vm.call(a, bed_get, false) == (pair.first == pair.second && pair.first == 130 ? 1 : pair.first == pair.second && pair.first == 170 ? 2 : 0));
            CHECK(a.writes.empty());
            CHECK(a.reset_log.empty());
        }
    }
    SECTION("Bed labels do not override index") {
        for (int index : {-1, 0, 1, 2, 3}) {
            for (const auto &label : {index >= 0 && index < 3 ? labels.at(index) : std::string("invalid"), std::string("Without glue"), std::string("Avec colle")}) {
                Store a;
                const auto before = a.edited;
                vm.call(a, bed_set, true, index, label);
                CHECK(a.edited.at("temperature") == before.at("temperature"));
                CHECK(a.edited.at("first_layer_temperature") == before.at("first_layer_temperature"));
                if (index == 0) {
                    CHECK(a.reset_log == std::vector<std::string>{"bed_temperature", "first_layer_bed_temperature"});
                    CHECK(a.writes.empty());
                    CHECK(a.edited.at("bed_temperature") == a.selected.at("bed_temperature"));
                    CHECK(a.edited.at("first_layer_bed_temperature") == a.selected.at("first_layer_bed_temperature"));
                } else if (index == 1 || index == 2) {
                    CHECK(a.writes == std::vector<std::string>{"bed_temperature", "first_layer_bed_temperature"});
                    CHECK(a.edited.at("bed_temperature") == std::vector<int>{index == 1 ? 130 : 170, index == 1 ? 130 : 170});
                    CHECK(a.edited.at("first_layer_bed_temperature") == a.edited.at("bed_temperature"));
                    CHECK(a.reset_log.empty());
                } else {
                    CHECK(a.edited == before);
                    CHECK(a.writes.empty());
                    CHECK(a.reset_log.empty());
                }
            }
        }
    }
    CHECK(preset_b.edited == unchanged_b);
}

// Exercise deferred operations on an owned model store, checking reset precedence and recorded errors without entering the VM.
TEST_CASE("Integer binding model preserves deferred semantics", "[host-simple-filament]")
{
    Store a;
    std::string key = "bed_temperature";
    a.can_set = true;
    a.set(key, 170);
    CHECK(a.get(key) == 130);
    a.reset(key);
    CHECK(a.get(key) == 60);
    a.set(key, 130);
    CHECK(a.resets.empty());
    a.apply();
    CHECK(a.edited.at(key) == std::vector<int>{130,130});
    a.can_set = false;
    a.set(key, 170);
    CHECK(a.pending.empty());
    std::string unknown = "missing";
    const auto count = a.edited.size();
    CHECK(a.get(unknown) == 0);
    CHECK(a.errors.size() == 1);
    CHECK(a.edited.size() == count);
    a.edited.at(key).clear();
    CHECK(a.get(key) == 0);
    CHECK(a.errors.size() == 2);
}

// Run Catch with explicit source paths from CTest and disable interactive CRT assertion dialogs on MSVC.
int main(int argc, char **argv)
{
#ifdef _MSC_VER
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    if (argc < 3) return 2;
    script_path = argv[1];
    ui_path = argv[2];
    argv[2] = argv[0];
    return Catch::Session().run(argc - 2, argv + 2);
}
