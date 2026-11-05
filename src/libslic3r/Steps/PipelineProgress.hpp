///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Steps_PipelineProgress_hpp_
#define slic3r_Steps_PipelineProgress_hpp_

#include <map>
#include <string>
#include <vector>

#include "libslic3r/PrintStatus.hpp"
#include "libslic3r/Api/plugin/c/slic3r_slicing_step.h"

/*
Translate completed pipeline work into the main progress percentage.
initialize() records the ordered units, begin_step() selects a planned stage,
and complete_plugin()/complete_step() account for work that returned successfully.
PrintStatus delivers the resulting values to the caller or GUI; it does not
decide whether a unit of processing has succeeded.

Initialize once for the planned suffix, begin each requested step, report its
successful units, then verify completion. Reset the table before a failed attempt
is retried so partial work cannot count twice. An initial percentage allows an
export after a ready preview to continue the displayed bar without moving backward.
*/

namespace Slic3r::Steps {

/*
PipelineProgress translates successful top-level pipeline units into the main
status percentage. It owns the execution accounting but delegates all delivery
and GUI synchronization to PrintStatus.

Unit order is fixed by initialize(). A non-empty unit id names a selected
plugin; an empty id represents host-only work. Units complete only after their
work returns successfully, so failures and cancellation cannot advance the
reported percentage.
*/
class PipelineProgress
{
public:
    explicit PipelineProgress(PrintStatus &status) : m_status(status) {}

    // Plan the units. The percentage starts at initial_percent and never falls below it: a plan that continues
    // work already shown (an export after the preview) keeps the main bar where it was and fills it up to 100.
    void initialize(const std::map<slicing_step_t, std::vector<std::string>> &units_by_step, int initial_percent = 0);
    bool active() const { return m_active; }
    int percent() const { return m_percent; }
    // Last main percentage that was shown: published by this plan (begin_step, complete_plugin) or declared by the
    // caller with set_shown_percent(). Unlike percent() it survives reset(), because a plan that finds nothing to
    // run is empty and leaves the bar where it was. A plan of a new operation that publishes lowers it; a plan that
    // continues the shown work (the export) starts from it.
    int shown_percent() const { return m_shown_percent; }
    // Declare a main percentage published outside the plan (the 100 % that Orchestrator::export_gcode() publishes).
    void set_shown_percent(int percent) { m_shown_percent = percent; }

    void begin_step(slicing_step_t step, const std::string &message);
    void complete_plugin(slicing_step_t step, const std::string &plugin_id);
    void complete_step(slicing_step_t step);
    void verify_step_complete(slicing_step_t step) const;
    void reset();

private:
    struct Step {
        std::vector<std::string> unit_ids;
        size_t                   completed { 0 };
    };

    PrintStatus                         &m_status;
    std::map<slicing_step_t, Step>       m_steps;
    size_t                               m_total { 0 };
    size_t                               m_completed { 0 };
    int                                  m_percent { 0 };
    int                                  m_shown_percent { 0 };
    bool                                 m_active { false };
};

} // namespace Slic3r::Steps

#endif // slic3r_Steps_PipelineProgress_hpp_
