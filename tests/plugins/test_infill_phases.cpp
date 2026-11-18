// STEP_INFILL is an ordered object-plugin phase, not an exclusive generator
// runner. Metadata-only passes need no patterns; generation still initializes
// each used pattern once with discovery from every object.
#include <catch2/catch.hpp>
#include <atomic>
#include <mutex>
#include <set>
#include <vector>
#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "probe_pattern_plugin.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
constexpr const char *before_id="test.infill.phase.before";
constexpr const char *after_id="test.infill.phase.after";
constexpr const char *pattern_id="test.infill.phase.pattern";
std::vector<int> calls;
std::vector<int> setup_counts;
bool bad_payload=false;

class Active {
public:
    Active(std::initializer_list<const char *> ids) {
        auto &host=Orchestrator::instance();
        previous.assign(host.active_plugins().begin(),host.active_plugins().end());host.clear_active_plugins();
        for (const char *id : ids) REQUIRE(host.set_plugin_active(id,true));
        host.reset_plugin_cancel();(void)host.consume_plugin_messages();
    }
    ~Active() {
        auto &host=Orchestrator::instance();host.reset_plugin_cancel();host.clear_active_plugins();
        for (Plugin *p : previous) host.set_plugin_active(p,true);
        (void)host.consume_plugin_messages();
    }
private:
    std::vector<Plugin *> previous;
};

class Phase final : public slic3r_api::PluginBase {
public:
    Phase(orchestrator_handle *host,const char *id,int priority) : PluginBase(host),id_(id),priority_(priority) {}
private:
    const char *id_impl() const noexcept override { return id_; }
    slicing_step_t step_impl() const noexcept override { return STEP_INFILL; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    int32_t priority_impl() const noexcept override { return priority_; }
    void run_impl(const plugin_run_context *ctx) const override {
        const auto *payload=plugin_ctx_as_generate_infill(ctx);
        bad_payload=payload==nullptr || payload->print==nullptr || payload->object==nullptr;
        calls.push_back(priority_);
        setup_counts.push_back(Test::ProbePatternPlugin::pattern_setup_count.load());
    }
    const char *id_;
    int priority_;
};

class Pattern final : public slic3r_api::PluginBase {
public:
    explicit Pattern(orchestrator_handle *host) : PluginBase(host) {}
    mutable std::atomic<int> setups{0},runs{0};
    mutable size_t objects=0,surfaces=0;
    mutable bool valid=false;
    mutable std::mutex mutex;
    mutable std::set<const object_handle *> seen;
private:
    const char *id_impl() const noexcept override { return pattern_id; }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    int32_t priority_impl() const noexcept override { return 0; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    void setup_impl(const plugin_run_context *ctx,uint32_t count) const override {
        ++setups;
        const auto *payload=plugin_ctx_as_infill_pattern_init(ctx);
        objects=payload ? payload->object_count : 0;surfaces=0;valid=payload && payload->print;
        if (payload) for (uint32_t i=0;i<payload->object_count;++i) {
            const auto &object=payload->objects[i];
            valid=valid && object.object!=nullptr && object.assignment_count>0;
            if (i && payload->objects[i-1].object==object.object) valid=false;
            for (uint32_t j=0;j<object.assignment_count;++j) surfaces+=object.assignments[j].surface_count;
        }
        valid=valid && surfaces==count;
    }
    void run_impl(const plugin_run_context *ctx) const override {
        const auto *payload=plugin_ctx_as_infill_pattern(ctx);
        ++runs;
        std::lock_guard<std::mutex> lock(mutex);
        if (payload) seen.insert(payload->object);
    }
};

Pattern &register_probes() {
    auto &host=Orchestrator::instance();auto *handle=reinterpret_cast<orchestrator_handle *>(&host);
    static Phase before(handle,before_id,-100),after(handle,after_id,100);
    static Pattern pattern(handle);
    const std::pair<const char *,slic3r_api::PluginBase *> probes[]={{before_id,&before},{after_id,&after},{pattern_id,&pattern}};
    for (const auto &probe : probes)
        if (host.get_plugin(probe.first)==nullptr) REQUIRE(host.register_plugin(probe.second->c_instance()));
    return pattern;
}

class AlternativeGenerator final : public slic3r_api::PluginBase {
public:
    explicit AlternativeGenerator(orchestrator_handle *host) : PluginBase(host) {}
private:
    const char *id_impl() const noexcept override { return "test.infill.phase.alternative"; }
    slicing_step_t step_impl() const noexcept override { return STEP_INFILL; }
    const char *exclusive_group_impl() const noexcept override { return "generator"; }
    int32_t priority_impl() const noexcept override { return 0; }
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    void run_impl(const plugin_run_context *) const override {}
};

DynamicPrintConfig config() {
    return perimeter_config({{"fill_pattern","gyroid"},{"fill_density","20%"},
        {"top_solid_layers","0"},{"bottom_solid_layers","0"},{"solid_infill_every_layers","0"}});
}
}

TEST_CASE("STEP_INFILL runs ordered metadata passes without any pattern", "[plugins][infill][infill-phases]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();register_probes();
    PreparedPerimeterPrint prepared;prepare_cube_print(prepared,config());
    Active active({before_id,after_id});calls.clear();setup_counts.clear();bad_payload=false;
    REQUIRE_NOTHROW(Steps::StepGenerateInfill::run_step(Orchestrator::instance(),prepared.print));
    CHECK(calls==std::vector<int>{-100,100});CHECK_FALSE(bad_payload);
}

TEST_CASE("The infill generator selector does not exclude other passes", "[plugins][infill][infill-phases]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();register_probes();
    auto &host=Orchestrator::instance();
    constexpr const char *alternative_id="test.infill.phase.alternative";
    static AlternativeGenerator alternative(reinterpret_cast<orchestrator_handle *>(&host));
    if (host.get_plugin(alternative_id)==nullptr) REQUIRE(host.register_plugin(alternative.c_instance()));
    Active active({before_id,"infill.generator.default",alternative_id,after_id});
    DynamicPrintConfig choice;
    CHECK(Steps::plugin_exclusive_group_option_key(host,STEP_INFILL,"generator")=="step_infill_plugin");
    choice.set_key_value("step_infill_plugin",new ConfigOptionString(alternative_id));
    const auto plugins=Steps::selected_or_active_plugins_for_step(host,STEP_INFILL,&choice);
    std::vector<std::string> ids;
    for (const Plugin *plugin : plugins) ids.push_back(plugin->get_id());
    CHECK(ids==std::vector<std::string>{before_id,alternative_id,after_id});
}

TEST_CASE("STEP_INFILL only initializes patterns when its generator requests them", "[plugins][infill][infill-phases]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();register_probes();
    PreparedPerimeterPrint prepared;prepare_cube_print(prepared,config());
    Active active({SIMPLE_PERIMETER_GENERATOR,INITIAL_TYPED_SURFACE_BUILDER,before_id,
        "infill.generator.default",after_id,Test::ProbePatternPlugin::probe_pattern_id});
    auto &host=Orchestrator::instance();
    Steps::StepGeneratePerimeter::run_step(host,prepared.print);Steps::StepSurfaceGeneration::run_step(host,prepared.print);
    Test::ProbePatternPlugin::pattern_setup_count=0;calls.clear();setup_counts.clear();bad_payload=false;
    REQUIRE_NOTHROW(Steps::StepGenerateInfill::run_step(host,prepared.print));
    CHECK(calls==std::vector<int>{-100,100});CHECK(setup_counts==std::vector<int>{0,1});CHECK_FALSE(bad_payload);
}

TEST_CASE("Generic infill keeps one print-wide pattern setup after multi-object discovery", "[plugins][infill][infill-phases]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();Pattern &pattern=register_probes();
    Print print;Model model;
    prepare_objects_print(print,model,{make_cube(8.,8.,2.),make_cube(6.,6.,2.)},config());
    REQUIRE(print.objects().size()==2);
    Active active({SIMPLE_PERIMETER_GENERATOR,INITIAL_TYPED_SURFACE_BUILDER,"infill.generator.default",pattern_id});
    auto &host=Orchestrator::instance();Steps::StepGeneratePerimeter::run_step(host,print);Steps::StepSurfaceGeneration::run_step(host,print);
    pattern.setups=0;pattern.runs=0;pattern.seen.clear();
    REQUIRE_NOTHROW(Steps::StepGenerateInfill::run_step(host,print));
    CHECK(pattern.setups==1);CHECK(pattern.objects==2);CHECK(pattern.surfaces>0);CHECK(pattern.valid);
    CHECK(size_t(pattern.runs.load())==pattern.surfaces);CHECK(pattern.seen.size()==2);
}
