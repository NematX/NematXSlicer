import ctypes

from slic3r_datatree_views import LayerSupportProperty
from slic3r_api import (
    PluginBase,
    PLUGIN_ERROR_GENERAL,
    PLUGIN_ERROR_OK,
    STEP_POST_SLICING,
    STEP_ORDERING,
    run_context,
    store_error_message,
)


class ExternalPackageRootPlugin(PluginBase):
    def __init__(self):
        super().__init__(
            "python.external.package_root",
            STEP_POST_SLICING,
            name="External Python package root test",
            description="No-op plugin loaded from its own installed package.",
            priority=100001,
        )

    def run(self, run_ctx_address):
        return None


class ClearResultPlugin(PluginBase):
    def __init__(self, case):
        super().__init__(
            "python.test.clear." + case,
            STEP_POST_SLICING,
            exclusive_group="",
        )
        self.case = case

    def setup(self, run_ctx_address, run_count):
        context = run_context(run_ctx_address)
        self.expected_handles = (context.print, context.plugin_storage)
        self.clear_calls = 0

    def clear(self, print_address, plugin_storage_address):
        if (print_address, plugin_storage_address) != self.expected_handles:
            raise RuntimeError("clear received different Print/storage handles from setup")
        self.clear_calls += 1
        if self.clear_calls > 1:
            return PLUGIN_ERROR_OK
        if self.case == "success":
            return PLUGIN_ERROR_OK
        if self.case == "none":
            return None
        if self.case == "error":
            # The returned id carries the failure text back to the host.
            return store_error_message("Python clear error: \u00e9chec")
        if self.case == "general":
            return PLUGIN_ERROR_GENERAL
        if self.case == "unknown_id":
            return 4242
        if self.case == "long_utf8":
            return store_error_message("\u00e9" * 300)
        if self.case == "exception":
            raise RuntimeError("Python clear exception: \u00e9chec")
        if self.case == "unprintable_exception":
            class UnprintableError(Exception):
                def __str__(self):
                    raise ValueError("cannot format exception")
            raise UnprintableError()
        if self.case == "bool":
            return True
        if self.case == "float":
            return 0.0
        if self.case == "str":
            return "0"
        if self.case == "bytes":
            return b""
        if self.case == "list":
            return [PLUGIN_ERROR_OK]
        if self.case == "tuple":
            return (PLUGIN_ERROR_OK,)
        if self.case == "negative":
            return -1
        if self.case == "overflow":
            return 2**31
        raise AssertionError("unknown clear fixture case: " + self.case)


class NoncallableClearPlugin(ClearResultPlugin):
    def __init__(self):
        super().__init__("noncallable")
        self.clear_accesses = None

    def setup(self, run_ctx_address, run_count):
        super().setup(run_ctx_address, run_count)
        self.clear_accesses = 0

    @property
    def clear(self):
        # Registration sees a callable; only the first access after setup fails.
        if self.clear_accesses is not None:
            self.clear_accesses += 1
            if self.clear_accesses == 1:
                return 42
        return self.clear_restored

    def clear_restored(self, print_address, plugin_storage_address):
        if (print_address, plugin_storage_address) != self.expected_handles:
            raise RuntimeError("clear received different Print/storage handles from setup")
        return PLUGIN_ERROR_OK


class FreezeCapabilityPlugin(PluginBase):
    def __init__(self, api):
        super().__init__("python.test.freeze.capability", STEP_ORDERING, exclusive_group="")
        self.api = api

    def setup_run(self, run_ctx_address):
        context = run_context(run_ctx_address)
        # The native caller keeps this counter buffer alive throughout execute, including run().
        self.counter = ctypes.cast(context.data, ctypes.POINTER(ctypes.c_uint32))
        properties = self.api.print(context.print).object(0).mutable_properties()
        try:
            properties.get_or_add(LayerSupportProperty)
        except RuntimeError:
            if properties.valid():
                raise
            self.counter[1] += 1

    def run(self, run_ctx_address):
        self.counter[0] += 1


class RegistrationLifetimeProbe(PluginBase):
    """Expose initialization and execution counts while remaining disabled at registration."""

    def __init__(self):
        """Create a counter without enabling the object or registering settings."""
        super().__init__("python.test.registration_lifetime", STEP_POST_SLICING, exclusive_group="")
        self.initializations = 0

    def initialize(self, storage_address):
        """Count initialization without retaining or changing application storage."""
        self.initializations += 1

    def run(self, run_ctx_address):
        """Write initialization and execution counts into the caller's two-uint32 array without retaining its address."""
        context = run_context(run_ctx_address)
        counts = ctypes.cast(context.data, ctypes.POINTER(ctypes.c_uint32))
        counts[0] = self.initializations
        counts[1] += 1


class InitializationFailureProbe(PluginBase):
    """Provide a disabled plugin whose initializer raises an error with a known explanation."""

    def __init__(self):
        """Create a plugin without a selection group or default activation."""
        super().__init__("python.test.initialize.failure", STEP_POST_SLICING, exclusive_group="")

    def initialize(self, storage_address):
        """Raise the known initialization error without retaining or changing borrowed storage."""
        raise RuntimeError("Python initializer deliberate failure")


class MetadataProbe(PluginBase):
    """Expose plugin-description cases without a selection group or enabled execution."""

    def __init__(self, case):
        """Copy plugin descriptions and supply obsolete attributes only in the cases named for them."""
        plugin_id = "python.test.metadata." + case
        if case == "keyword":
            super().__init__(plugin_id, STEP_POST_SLICING, exclusive_group="",
                             exclusive_group_label="", exclusive_group_tooltip="Old tooltip")
        elif case == "positional_label":
            super().__init__(plugin_id, STEP_POST_SLICING, "Metadata", "", 0, None, None, None, "", "Old label")
        elif case == "positional":
            super().__init__(plugin_id, STEP_POST_SLICING, "Metadata", "", 0, None, None, None, "", "Old label", "Old tooltip")
        else:
            super().__init__(plugin_id, STEP_POST_SLICING, exclusive_group="")
        if case == "absent":
            del self.exclusive_group_priority
        elif case == "minimum":
            self.exclusive_group_priority = -(2**31)
        elif case == "maximum":
            self.exclusive_group_priority = 2**31 - 1
        elif case == "label":
            self.exclusive_group_label = "Old label"
        elif case == "tooltip":
            self.exclusive_group_tooltip = ""


def register_plugin(api):
    """Return disabled plugins for execution and description checks through the normal loader."""
    # The runtime only activates named production fixtures; these probes stay inactive.
    cases = (
        "success", "none", "error", "general", "unknown_id", "long_utf8",
        "exception", "unprintable_exception", "bool", "float", "str",
        "bytes", "list", "tuple", "negative", "overflow",
    )
    return [ExternalPackageRootPlugin(), *(ClearResultPlugin(case) for case in cases),
            NoncallableClearPlugin(), FreezeCapabilityPlugin(api), RegistrationLifetimeProbe(), InitializationFailureProbe(),
            *(MetadataProbe(case) for case in ("absent", "minimum", "maximum", "label", "tooltip", "positional", "positional_label", "keyword"))]
