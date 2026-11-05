///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_PostProcessScripts_hpp_
#define slic3r_Plugins_GCode_PostProcessScripts_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace GCodeGeneration { namespace PostProcessScriptsPlugin {

/*
Register the STEP_GCODE plugin which runs the `post_process` scripts on the file
created by the built-in G-code writer. The Reader and the print statistics
therefore see the post-processed text.
*/
void register_post_process_scripts_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::PostProcessScriptsPlugin

#endif // slic3r_Plugins_GCode_PostProcessScripts_hpp_
