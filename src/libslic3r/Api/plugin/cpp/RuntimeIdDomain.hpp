///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_Api_plugin_cpp_RuntimeIdDomain_hpp_
#define slic3r_Api_plugin_cpp_RuntimeIdDomain_hpp_

/* A typed runtime-ID namespace. The domain name is borrowed and must outlive
   this helper (normally a static constant). Each operation takes its owning
   orchestrator explicitly; registration needs mutable access, lookup does not.
   Register before parallel processing and never persist numeric runtime IDs.

       RuntimeIdDomain modes(FAN_REQUEST_MODE_ID_TYPE);
       const auto id = modes.register_name(host, "com.example.cooling.mode");
       modes.validate(host, id);

   The host owns registered names and IDs. No operation retains the orchestrator. */
#include <stdexcept>
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api {

class RuntimeIdDomain
{
public:
    // Borrow the stable domain name for this helper's lifetime.
    explicit RuntimeIdDomain(const char *id_type) : m_id_type(id_type) {}

    // Return the borrowed domain name without consulting the host.
    const char *id_type() const { return m_id_type; }
    // Register or retrieve a stable name; rejected registration throws invalid_argument.
    orchestrator_runtime_id register_name(orchestrator_handle *orchestrator, const char *namespaced_name) const {
        const orchestrator_runtime_id id = orchestrator_register_runtime_id(
            orchestrator, m_id_type, namespaced_name);
        if (id == ORCHESTRATOR_RUNTIME_ID_INVALID)
            throw std::invalid_argument("Cannot register runtime ID.");
        return id;
    }
    // Borrow a host-owned name, or return null for an unknown or invalid ID.
    const char *name(const orchestrator_handle *orchestrator, orchestrator_runtime_id id) const {
        return id == ORCHESTRATOR_RUNTIME_ID_INVALID ? nullptr :
            orchestrator_runtime_id_name(orchestrator, m_id_type, id);
    }
    // Check registration in this domain without changing the registry.
    bool is_valid(const orchestrator_handle *orchestrator, orchestrator_runtime_id id) const {
        return name(orchestrator, id) != nullptr;
    }
    // Throw invalid_argument unless this host owns the ID in the selected domain.
    void validate(const orchestrator_handle *orchestrator, orchestrator_runtime_id id) const {
        if (!is_valid(orchestrator, id))
            throw std::invalid_argument("Unregistered runtime ID.");
    }

private:
    const char *m_id_type;
};

} // namespace slic3r_api
#endif
