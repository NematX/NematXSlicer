#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
"""Public fan context and total-layer timing, stored as typed plugin properties.

Register keys during initialization with the owning API. Returned payload views
follow the property container's lifetime; no plan or device state is cached here.
Numerical policy kernels live in the C++ SDK; Python can inspect the same records.

    key = fan_policy_context_key(api)
    context = tool.properties().get(key)
    if context is not None:
        validate_fan_policy_context(context, RAW_FAN_POLICY_PHASE_CLOSED)

The host owns property storage. Validators borrow it for one call and raise
ValueError for malformed phase, reference or duration records.
"""
from __future__ import annotations

import ctypes
import math

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    CFanPolicyContext,
    CPrintingLayerTimeProperty,
    FAN_POLICY_CONTEXT_PROPERTY_NAME,
    PRINTING_LAYER_TIME_PROPERTY_NAME,
    RAW_FAN_POLICY_PHASE_BASE,
    RAW_FAN_POLICY_PHASE_TIME,
    RAW_FAN_POLICY_PHASE_CLOSED,
    RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED,
    RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED,
)
from slic3r_property_key import PluginPropertyKey

FanPolicyContext = CFanPolicyContext
PrintingLayerTimeProperty = CPrintingLayerTimeProperty


@abi_requires_headers("slic3r_fan_policy.h", "slic3r_orchestrator.h")
def fan_policy_context_key(api) -> PluginPropertyKey:
    """Register the public tool-context layout in this API's orchestrator."""
    return PluginPropertyKey.register_dynamic(api, FAN_POLICY_CONTEXT_PROPERTY_NAME, FanPolicyContext)


@abi_requires_headers("slic3r_printing_layer_time.h", "slic3r_orchestrator.h")
def printing_layer_time_property_key(api) -> PluginPropertyKey:
    """Register the unchanged per-layer duration layout and stable identity."""
    return PluginPropertyKey.register_dynamic(api, PRINTING_LAYER_TIME_PROPERTY_NAME, PrintingLayerTimeProperty)


def validate_fan_policy_context(context, expected_phase: int) -> None:
    """Reject unpublished storage or a phase/reference contract violation."""
    if (expected_phase not in (RAW_FAN_POLICY_PHASE_BASE, RAW_FAN_POLICY_PHASE_TIME, RAW_FAN_POLICY_PHASE_CLOSED)
            or context.struct_size != ctypes.sizeof(FanPolicyContext) or context.phase != expected_phase
             or not 0 <= context.dynamic_min <= context.dynamic_max <= 100):
        raise ValueError("Invalid fan context size, phase or dynamic bounds")
    for role in context.roles:
        if not -1 <= role.configured_percent <= 100 or not -1 <= role.percent <= 100:
            raise ValueError("Invalid fan role reference")


def validate_printing_layer_time(time) -> None:
    """Both origins require finite, nonnegative model time, never machine time."""
    if (time.origin not in (RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED, RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED)
            or not math.isfinite(time.duration_seconds) or time.duration_seconds < 0):
        raise ValueError("Invalid PrintingLayerGroup duration or origin")


__all__ = [
    "FanPolicyContext", "PrintingLayerTimeProperty", "fan_policy_context_key",
    "printing_layer_time_property_key", "validate_fan_policy_context", "validate_printing_layer_time",
]
