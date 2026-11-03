///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepsupportdemand_hpp_
#define steps_stepsupportdemand_hpp_

#include <map>
#include <memory>
#include <stddef.h>
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

/* This interface lets StepPipeline collect support requests in State, indexed by print object.
   Register the settings metadata defined by StepSelectorTraits.hpp with register_selector_traits()
   at host startup. run_step() creates or fills State; pass it to support generation afterward.
   State::reset() discards the collected requests. */
namespace Slic3r {
class Orchestrator;
class Print;
class PrintObject;

namespace ApiHost::Steps { class SupportDemandSet; }

namespace Steps::StepSupportDemand {

// Declare host selector traits for support-demand processing on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

class State
{
public:
    State();
    ~State();
    State(State &&) noexcept;
    State &operator=(State &&) noexcept;
    State(const State &) = delete;
    State &operator=(const State &) = delete;

    void reset();
    size_t object_count() const;
    ApiHost::Steps::SupportDemandSet &demand_for(PrintObject &object);
    const ApiHost::Steps::SupportDemandSet *demand_for(const PrintObject &object) const;

private:
    std::map<const PrintObject *, std::unique_ptr<ApiHost::Steps::SupportDemandSet>> m_demands;
};

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string &error);
bool validate_post(const Print &print, std::string &error);
State run_step(Orchestrator &orchestrator, Print &print);
void run_step(Orchestrator &orchestrator, Print &print, State &state);

} // namespace Steps::StepSupportDemand
} // namespace Slic3r

#endif // steps_stepsupportdemand_hpp_
