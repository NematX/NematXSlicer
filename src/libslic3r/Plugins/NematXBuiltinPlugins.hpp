// Register optional embedded NematX modules before activation. The generated
// adapter leaves an already registered standalone package authoritative.
#ifndef slic3r_NematXBuiltinPlugins_hpp_
#define slic3r_NematXBuiltinPlugins_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace Slic3r {
void register_nematx_builtin_plugins(orchestrator_handle *orchestrator);
}

#endif
