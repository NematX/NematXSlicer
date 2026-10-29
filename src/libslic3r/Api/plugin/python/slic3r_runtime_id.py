#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
"""A typed runtime-ID namespace, with an explicit API per operation.

Register stable names before parallel processing. Numeric IDs are local to the
orchestrator and domain, not persistent model or preset identities.

    modes = RuntimeIdDomain("slic3r.fan.request_mode")
    mode = modes.register_name(api, "com.example.cooling.mode")
    modes.validate(api, mode)

Each helper owns its encoded domain name but borrows the API only for a call.
The host owns registered names and returns copies to Python.
"""
from __future__ import annotations

import operator

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import ORCHESTRATOR_RUNTIME_ID_INVALID


@abi_requires_headers("slic3r_orchestrator.h")
class RuntimeIdDomain:
    def __init__(self, id_type: str):
        """Own the domain name; operations still name their host API explicitly."""
        self.id_type = id_type
        self._encoded_type = id_type.encode("utf-8")

    def register_name(self, api, namespaced_name: str) -> int:
        """Register or retrieve a host-local ID, raising ValueError on rejection."""
        identifier = int(api.host.orchestrator_register_runtime_id(
            api.orchestrator, self._encoded_type, namespaced_name.encode("utf-8")))
        if identifier == ORCHESTRATOR_RUNTIME_ID_INVALID:
            raise ValueError(f"Cannot register runtime ID in domain {self.id_type}")
        return identifier

    def name(self, api, identifier: int) -> str | None:
        """Copy the host's name, or return None for an unknown or invalid ID."""
        identifier = operator.index(identifier)
        if not 0 <= identifier < ORCHESTRATOR_RUNTIME_ID_INVALID:
            return None
        name = api.host.orchestrator_runtime_id_name(
            api.orchestrator, self._encoded_type, identifier)
        return None if name is None else name.decode("utf-8")

    def is_valid(self, api, identifier: int) -> bool:
        """Check this host's domain without changing its registry."""
        return self.name(api, identifier) is not None

    def validate(self, api, identifier: int) -> None:
        """Raise ValueError unless the ID is registered in this host's domain."""
        if not self.is_valid(api, identifier):
            raise ValueError(f"Unregistered runtime ID in domain {self.id_type}")


__all__ = ["RuntimeIdDomain"]
