"""Behavior tests for the high-level Python PrintingPlan views."""

import ctypes
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest


ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "src/libslic3r/Api/plugin/python"
sys.dont_write_bytecode = True
sys.path.insert(0, str(RUNTIME))


def load_python_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def address(handle):
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle or 0)


class PrintingPlanHost:
    """Small deterministic host exposing one complete printing-plan branch."""

    def __init__(self, generated):
        self.generated = generated
        self.calls = []

    def printing_plan_get_events(self, _plan):
        return 11

    def printing_plan_get_events_mutable(self, _plan):
        return 11

    def printing_plan_get_properties(self, _plan):
        return 12

    def printing_plan_get_properties_mutable(self, _plan):
        return 12

    def printing_plan_count_object_instance(self, _plan):
        return 2

    def printing_plan_register_object_instance(
        self, _plan, _object, object_instance_idx, _name, _outline
    ):
        return int(object_instance_idx)

    def printing_plan_set_object_instance_metadata(
        self, _plan, _instance_id, _name, _outline
    ):
        return 1

    def printing_plan_get_object_instance_by_printing_catalog_id(self, _plan, printing_catalog_id):
        return self.generated.CPrintingObjectInstance(
            ctypes.c_void_p(89 + int(printing_catalog_id)),
            6 + int(printing_catalog_id),
            int(printing_catalog_id),
            f"Instance {printing_catalog_id}".encode("utf-8"),
            self.generated.CPoint(9 + int(printing_catalog_id), 19 + int(printing_catalog_id)),
            ctypes.c_void_p(189 + int(printing_catalog_id)),
        )

    def printing_plan_count_group(self, _plan):
        return 1

    def printing_plan_get_group(self, _plan, idx):
        return 20 + int(idx)

    def printing_plan_get_group_mutable(self, _plan, idx):
        return 20 + int(idx)

    def printing_plan_append_group(self, plan):
        self.calls.append(("append_group", address(plan)))
        return 21

    def printing_plan_move_group(self, plan, source, destination):
        self.calls.append(("move_group", address(plan), int(source), int(destination)))
        return 1

    def printing_plan_clear(self, plan):
        self.calls.append(("clear_plan", address(plan)))

    def printing_group_get_events(self, _group):
        return 22

    def printing_group_get_events_mutable(self, _group):
        return 22

    def printing_group_get_properties(self, _group):
        return 23

    def printing_group_get_properties_mutable(self, _group):
        return 23

    def printing_group_count_object_instance(self, _group):
        return 1

    def printing_group_get_printing_catalog_id(self, _group, _idx):
        return 0

    def printing_group_append_printing_catalog_id(self, plan, group, printing_catalog_id):
        self.calls.append(
            ("append_object_id", address(plan), address(group), int(printing_catalog_id))
        )
        return 1

    def printing_group_count_layer_group(self, _group):
        return 1

    def printing_group_get_layer_group(self, _group, idx):
        return 30 + int(idx)

    def printing_group_get_layer_group_mutable(self, _group, idx):
        return 30 + int(idx)

    def printing_group_append_layer_group(self, group, print_z):
        self.calls.append(("append_layer_group", address(group), int(print_z)))
        return 31

    def printing_group_move_layer_group(self, group, source, destination):
        self.calls.append(
            ("move_layer_group", address(group), int(source), int(destination))
        )
        return 1

    def printing_group_clear(self, group):
        self.calls.append(("clear_group", address(group)))

    def printing_layer_group_get_events(self, _layer_group):
        return 32

    def printing_layer_group_get_events_mutable(self, _layer_group):
        return 32

    def printing_layer_group_get_properties(self, _layer_group):
        return 33

    def printing_layer_group_get_properties_mutable(self, _layer_group):
        return 33

    def printing_layer_group_get_print_z(self, _layer_group):
        return 42000

    def printing_layer_group_set_print_z(self, layer_group, print_z):
        self.calls.append(("set_print_z", address(layer_group), int(print_z)))

    def printing_layer_group_count_layer(self, _layer_group):
        return 1

    def printing_layer_group_get_layer(self, _layer_group, idx):
        return 100 + int(idx)

    def printing_layer_group_append_layer(self, layer_group, layer):
        self.calls.append(("append_layer", address(layer_group), address(layer)))

    def printing_layer_group_count_tool_group(self, _layer_group):
        return 1

    def printing_layer_group_get_tool_group(self, _layer_group, idx):
        return 40 + int(idx)

    def printing_layer_group_get_tool_group_mutable(self, _layer_group, idx):
        return 40 + int(idx)

    def printing_layer_group_append_tool_group(self, layer_group, extruder_id):
        self.calls.append(
            ("append_tool_group", address(layer_group), int(extruder_id))
        )
        return 41

    def printing_layer_group_move_tool_group(self, layer_group, source, destination):
        self.calls.append(
            ("move_tool_group", address(layer_group), int(source), int(destination))
        )
        return 1

    def printing_layer_group_remove_empty_tool_group(self, layer_group, idx):
        self.calls.append(("remove_tool_group", address(layer_group), int(idx)))
        return 1

    def printing_tool_group_get_events(self, _tool_group):
        return 42

    def printing_tool_group_get_events_mutable(self, _tool_group):
        return 42

    def printing_tool_group_get_properties(self, _tool_group):
        return 43

    def printing_tool_group_get_properties_mutable(self, _tool_group):
        return 43

    def printing_tool_group_get_extruder_id(self, _tool_group):
        return 2

    def printing_tool_group_set_extruder_id(self, tool_group, extruder_id):
        self.calls.append(
            ("set_extruder_id", address(tool_group), int(extruder_id))
        )

    def printing_tool_group_count_region_island(self, _tool_group):
        return 1

    def printing_tool_group_get_region_island(self, _tool_group, idx):
        return 110 + int(idx)

    def printing_tool_group_append_region_island(self, tool_group, island):
        self.calls.append(
            ("append_region_island", address(tool_group), address(island))
        )

    def printing_tool_group_count_extrusion(self, _tool_group):
        return 1

    def printing_tool_group_get_extrusion(self, _tool_group, idx):
        return 50 + int(idx)

    def printing_tool_group_get_extrusion_mutable(self, _tool_group, idx):
        return 50 + int(idx)

    def printing_tool_group_append_extrusion_clone(
        self, plan, tool_group, island, role, root, printing_catalog_id
    ):
        self.calls.append(
            (
                "clone_extrusion",
                address(plan),
                address(tool_group),
                address(island),
                int(role),
                address(root),
                int(printing_catalog_id),
            )
        )
        return 51

    def printing_tool_group_append_extrusion_clone_from_region_island(
        self, _plan, _tool_group, _island, _role, _instance_id
    ):
        return 52

    def printing_tool_group_append_extrusion_move(
        self, plan, tool_group, island, role, root, printing_catalog_id
    ):
        self.calls.append(
            (
                "move_extrusion_root",
                address(plan),
                address(tool_group),
                address(island),
                int(role),
                address(root),
                int(printing_catalog_id),
            )
        )
        return 53

    def printing_tool_group_move_extrusion(self, tool_group, source, destination):
        self.calls.append(
            ("move_extrusion", address(tool_group), int(source), int(destination))
        )
        return 1

    def printing_tool_group_transfer_extrusion(self, source, idx, destination):
        self.calls.append(
            ("transfer_extrusion", address(source), int(idx), address(destination))
        )
        return 1

    def printing_extrusion_get_region_island(self, _extrusion):
        return 110

    def printing_extrusion_get_role(self, _extrusion):
        return 8

    def printing_extrusion_set_role(self, extrusion, role):
        self.calls.append(("set_role", address(extrusion), int(role)))

    def printing_extrusion_get_printing_catalog_id(self, _extrusion):
        return 5

    def printing_extrusion_get_root(self, _extrusion):
        return 120

    def printing_extrusion_get_root_mutable(self, _extrusion):
        return 120

    def printing_extrusion_set_root_clone(self, extrusion, root):
        self.calls.append(("set_root_clone", address(extrusion), address(root)))
        return 1

    def printing_extrusion_set_root_move(self, extrusion, root):
        self.calls.append(("set_root_move", address(extrusion), address(root)))
        return 1

    def printing_scope_events_has_before(self, _events):
        return 1

    def printing_scope_events_has_after(self, _events):
        return 0

    def printing_scope_events_get_before(self, _events):
        return 121

    def printing_scope_events_get_after(self, _events):
        return 122

    def printing_scope_events_append_before_clone(self, events, root):
        self.calls.append(("before_clone", address(events), address(root)))
        return 123

    def printing_scope_events_append_before_move(self, events, root):
        self.calls.append(("before_move", address(events), address(root)))
        return 124

    def printing_scope_events_append_after_clone(self, _events, _root):
        return 125

    def printing_scope_events_append_after_move(self, _events, _root):
        return 126


class PythonPrintingPlanViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "printing_plan_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_printing_plan_views")
        cls.execution = importlib.import_module("steps.execution")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        host = PrintingPlanHost(self.generated)
        return types.SimpleNamespace(host=host), host

    def test_fan_property_preserves_exclusive_modes_and_zero_requests(self):
        extrusion = importlib.import_module("slic3r_extrusion_views")
        fan = extrusion.EPropertyFan()
        known_modes = {extrusion.FAN_REQUEST_MODE_UNSPECIFIED, extrusion.FAN_REQUEST_MODE_AUTOMATIC,
                       extrusion.FAN_REQUEST_MODE_OVERRIDE, 0x80000000}
        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(1), host=types.SimpleNamespace(
            orchestrator_runtime_id_name=lambda orch, domain, mode:
                b"tests.mode" if domain == b"slic3r.fan.request_mode" and mode in known_modes else None))
        self.assertIsNone(fan.validate(api))
        self.assertEqual(ctypes.sizeof(fan), 8)
        self.assertFalse(fan.is_set())
        self.assertEqual(fan.percent, 0)
        self.assertEqual(fan.key.type(), self.generated.EXTRUSION_PROPERTY_TYPE_FAN)
        parent = extrusion.EPropertyFan().set(70, extrusion.FAN_REQUEST_MODE_OVERRIDE)
        self.assertEqual(fan.resolved_with(parent).percent, 70)
        self.assertEqual(fan.resolved_with(parent).mode, extrusion.FAN_REQUEST_MODE_OVERRIDE)
        for mode in (extrusion.FAN_REQUEST_MODE_AUTOMATIC,
                     extrusion.FAN_REQUEST_MODE_OVERRIDE, 0x80000000):
            fan.set(0, mode)
            self.assertTrue(fan.is_set())
            fan.validate(api)
            self.assertEqual(fan.resolved_with(parent).percent, 0)
            self.assertEqual(fan.resolved_with(parent).mode, mode)
        fan.clear()
        self.assertFalse(fan.is_set())
        self.assertEqual(fan.percent, 0)
        for value in (-1, 101, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                fan.set(value, extrusion.FAN_REQUEST_MODE_AUTOMATIC)
        for mode in (-1, 0xFFFFFFFF, 0x100000000):
            with self.assertRaises(ValueError):
                fan.set(40, mode)
        with self.assertRaises(ValueError):
            fan.set(40, extrusion.FAN_REQUEST_MODE_UNSPECIFIED)
        fan.percent, fan.mode = float("nan"), extrusion.FAN_REQUEST_MODE_UNSPECIFIED
        self.assertIsNone(fan.validate(api))
        fan.mode = extrusion.FAN_REQUEST_MODE_AUTOMATIC
        with self.assertRaises(ValueError):
            fan.validate(api)
        fan.percent, fan.mode = 30, 42
        with self.assertRaises(ValueError):
            fan.validate(api)

    def test_public_fan_context_and_timing_keys_use_owning_registry(self):
        policy = importlib.import_module("slic3r_fan_policy")
        registry = {}

        def register(orch, name, size, alignment):
            identity = address(orch), name
            previous = registry.get(identity)
            if previous is not None:
                return previous[0] if previous[1:] == (size, alignment) else self.generated.SLIC3R_PROPERTY_TYPE_INVALID
            registry[identity] = (0x10000 + len(registry), size, alignment)
            return registry[identity][0]

        host = types.SimpleNamespace(orchestrator_register_property=register)
        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(11), host=host)
        other_api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(12), host=host)
        context_key = policy.fan_policy_context_key(api)
        self.assertEqual(context_key.type(), policy.fan_policy_context_key(api).type())
        self.assertNotEqual(context_key.type(), policy.fan_policy_context_key(other_api).type())
        self.assertEqual(context_key.payload_cls, policy.FanPolicyContext)
        self.assertEqual(address(context_key.orchestrator), 11)
        time_key = policy.printing_layer_time_property_key(api)
        self.assertNotEqual(context_key.type(), time_key.type())
        self.assertEqual(time_key.payload_cls, policy.PrintingLayerTimeProperty)
        self.assertEqual(ctypes.sizeof(policy.FanPolicyContext), 136)
        self.assertEqual(ctypes.sizeof(self.generated.CFanRoleReference), 8)
        for removed in ("layer_index", "disabled_layers", "full_speed_layer", "dynamic_enabled"):
            self.assertFalse(hasattr(policy.FanPolicyContext, removed))
        self.assertFalse(hasattr(self.generated.CFanRoleReference, "fallback_role"))
        self.assertEqual(policy.FanPolicyContext.roles.offset, 8)
        self.assertEqual(policy.PrintingLayerTimeProperty.origin.offset, 8)

    def test_public_fan_priorities_are_generated_from_the_step_header(self):
        priorities = ("STANDARD", "COOLING", "FIRST_LAYERS", "DYNAMIC")
        self.assertEqual(
            [getattr(self.generated, "LAYER_EXTRUSION_EDIT_PRIORITY_FAN_" + stage) for stage in priorities],
            [20, 22, 24, 26])

    def test_public_fan_context_and_timing_reject_unpublished_or_invalid_records(self):
        policy = importlib.import_module("slic3r_fan_policy")
        context = policy.FanPolicyContext()
        g = self.generated
        with self.assertRaises(ValueError):
            policy.validate_fan_policy_context(context, g.RAW_FAN_POLICY_PHASE_BASE)
        context.struct_size = ctypes.sizeof(context)
        context.phase = g.RAW_FAN_POLICY_PHASE_BASE
        context.dynamic_max = 100
        self.assertIsNone(policy.validate_fan_policy_context(context, g.RAW_FAN_POLICY_PHASE_BASE))
        for phase in (g.RAW_FAN_POLICY_PHASE_TIME, g.RAW_FAN_POLICY_PHASE_CLOSED, 42):
            with self.assertRaises(ValueError):
                policy.validate_fan_policy_context(context, phase)
        context.roles[0].percent = -2
        with self.assertRaises(ValueError):
            policy.validate_fan_policy_context(context, g.RAW_FAN_POLICY_PHASE_BASE)
        context.roles[0].percent = -1
        context.roles[0].configured_percent = -1
        self.assertIsNone(policy.validate_fan_policy_context(context, g.RAW_FAN_POLICY_PHASE_BASE))
        time = policy.PrintingLayerTimeProperty()
        for origin in (g.RAW_PRINTING_LAYER_TIME_ORIGIN_ESTIMATED, g.RAW_PRINTING_LAYER_TIME_ORIGIN_RESOLVED):
            time.origin = origin
            time.duration_seconds = 12.5
            self.assertIsNone(policy.validate_printing_layer_time(time))
            for value in (-1, float("nan"), float("inf")):
                time.duration_seconds = value
                with self.assertRaises(ValueError):
                    policy.validate_printing_layer_time(time)
        time.duration_seconds = 0
        time.origin = 42
        with self.assertRaises(ValueError):
            policy.validate_printing_layer_time(time)

    def test_runtime_id_domain_and_fan_conflict_rule(self):
        extrusion = importlib.import_module("slic3r_extrusion_views")
        RuntimeIdDomain = importlib.import_module("slic3r_runtime_id").RuntimeIdDomain
        registry = {}

        def register(orch, domain, name):
            names = registry.setdefault((address(orch), domain), {})
            if name not in names:
                names[name] = 0x80000000 + len(names)
            return names[name]

        def lookup(orch, domain, identifier):
            return next((name for name, value in registry.get((address(orch), domain), {}).items()
                         if identifier == value), None)

        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(11), host=types.SimpleNamespace(
            orchestrator_register_runtime_id=register, orchestrator_runtime_id_name=lookup))
        modes = RuntimeIdDomain(extrusion.FAN_REQUEST_MODE_ID_TYPE)
        registry[(11, extrusion.FAN_REQUEST_MODE_ID_TYPE.encode())] = {
            b"slic3r.unspecified": 0, b"slic3r.automatic": 1, b"slic3r.override": 2}
        mine = modes.register_name(api, "tests.fan.mine")
        other = modes.register_name(api, "tests.fan.other")
        self.assertEqual(modes.register_name(api, "tests.fan.mine"), mine)
        self.assertEqual(modes.name(api, mine), "tests.fan.mine")
        self.assertTrue(modes.is_valid(api, mine))
        for invalid in (-1, 42, 0xFFFFFFFF, 0x100000000):
            self.assertFalse(modes.is_valid(api, invalid))
            with self.assertRaises(ValueError):
                modes.validate(api, invalid)
        annotations = RuntimeIdDomain("tests.annotation")
        annotation = annotations.register_name(api, "tests.value")
        self.assertEqual(annotations.name(api, annotation), "tests.value")
        other_api = types.SimpleNamespace(host=api.host, orchestrator=ctypes.c_void_p(12))
        self.assertFalse(modes.is_valid(other_api, mine))

        request = extrusion.EPropertyFan()
        for percent in (0, 40):
            for inherited in (0, 1, 2, mine, other):
                request.set(0 if inherited == 0 else percent, inherited)
                self.assertIsNone(request.validate(api))
                self.assertTrue(request.is_valid(api))
                for policy in (1, 2, mine, other):
                    self.assertEqual(request.can_modify(policy),
                                     inherited != 2 and (inherited in (0, 1) or inherited == policy))
                self.assertFalse(request.can_modify(0xFFFFFFFF))
        request.clear().percent = 27
        self.assertIsNone(request.validate(api))
        request.mode = 42
        self.assertFalse(request.is_valid(api))

    def test_read_only_views_cover_the_complete_hierarchy(self):
        api, _host = self.make_api()
        plan = self.views.PrintingPlan(api, 10)

        self.assertEqual(len(plan), 1)
        group = next(plan.groups())
        self.assertEqual(group.printing_catalog_id(0), 0)
        self.assertEqual(list(group.printing_catalog_ids()), [0])
        layer_group = group.layer_group(0)
        self.assertEqual(layer_group.print_z(), 42000)
        self.assertEqual(layer_group.layer(0).address, 100)
        tool_group = layer_group.tool_group(0)
        self.assertEqual(tool_group.extruder_id(), 2)
        self.assertEqual(tool_group.region_island(0).address, 110)
        extrusion = tool_group.extrusion(0)
        self.assertEqual(extrusion.role(), 8)
        self.assertEqual(extrusion.printing_catalog_id(), 5)
        self.assertEqual(extrusion.root().address, 120)
        self.assertEqual(plan.object_instance_count(), 2)
        self.assertEqual([instance.id() for instance in plan.object_instances()], [0, 1])
        self.assertEqual(plan.object_instance(1).name(), "Instance 1")
        self.assertEqual(
            plan.object_instance(1).projected_outline_center(), (10, 20)
        )
        self.assertEqual(plan.object_instance(1).projected_outline().address, 190)
        self.assertTrue(plan.events().has_before())
        self.assertFalse(plan.events().has_after())
        self.assertEqual(plan.events().before().address, 121)
        with self.assertRaises(IndexError):
            plan.group(1)

    def test_mutable_views_forward_structure_clone_and_move_operations(self):
        api, host = self.make_api()
        plan = self.views.MutablePrintingPlan(api, 10)
        group = plan.group(0)
        layer_group = group.layer_group(0)
        tool_group = layer_group.tool_group(0)
        extrusion = tool_group.extrusion(0)

        printing_catalog_id = plan.find_or_add_object_instance(self.views.Object(api, 90), 7)
        self.assertEqual(printing_catalog_id, 7)
        group.append_printing_catalog_id(plan, printing_catalog_id)
        self.assertEqual(group.append_layer_group(42000).address, 31)
        layer_group.append_layer(self.views.Layer(api, 100))
        self.assertEqual(layer_group.append_tool_group(2).address, 41)
        tool_group.append_region_island(self.views.LayerRegionIsland(api, 110))
        cloned = tool_group.append_extrusion_clone(
            plan,
            self.views.LayerRegionIsland(api, 110),
            8,
            self.views.ExtrusionEntity(api, 120),
            printing_catalog_id,
        )
        moved = tool_group.append_extrusion_move(
            plan, None, 8, self.views.MutableExtrusionEntity(api, 130), printing_catalog_id
        )
        self.assertEqual((cloned.address, moved.address), (51, 53))
        extrusion.set_role(16)
        self.assertTrue(extrusion.set_root_clone(None))
        self.assertTrue(
            extrusion.set_root_move(self.views.MutableExtrusionEntity(api, 131))
        )
        self.assertEqual(plan.events().append_before_clone(extrusion.root()).address, 123)
        self.assertEqual(
            plan.events().append_before_move(
                self.views.MutableExtrusionEntity(api, 132)
            ).address,
            124,
        )
        # Parent-vector mutations are intentionally last: they invalidate the
        # descendant views used above in a real native plan.
        plan.clear()
        self.assertEqual(plan.append_group().address, 21)

        call_names = [call[0] for call in host.calls]
        self.assertIn("clone_extrusion", call_names)
        self.assertIn("move_extrusion_root", call_names)
        self.assertIn("set_root_move", call_names)
        foreign_api, _foreign_host = self.make_api()
        with self.assertRaises(ValueError):
            tool_group.append_region_island(
                self.views.LayerRegionIsland(foreign_api, 110)
            )

    def test_extruder_identity_boundaries_reject_before_ctypes_narrowing(self):
        api, host = self.make_api()
        plan = self.views.MutablePrintingPlan(api, 10)
        layer_group = plan.group(0).layer_group(0)
        tool_group = layer_group.tool_group(0)

        for extruder_id in (self.generated.EXTRUDER_ID_UNKNOWN, 0, 0x7FFF):
            tool_group.set_extruder_id(extruder_id)
            self.assertEqual(host.calls[-1], ("set_extruder_id", 40, extruder_id))

        call_count = len(host.calls)
        for invalid in (-2, 0x8000):
            with self.assertRaises(ValueError):
                tool_group.set_extruder_id(invalid)
        with self.assertRaises(TypeError):
            tool_group.set_extruder_id(1.5)
        self.assertEqual(len(host.calls), call_count)

        for invalid in (-2, 0x8000):
            with self.assertRaises(ValueError):
                layer_group.append_tool_group(invalid)
        with self.assertRaises(TypeError):
            layer_group.append_tool_group(1.5)
        self.assertEqual(len(host.calls), call_count)

    def test_step_contexts_expose_const_correct_plan_views(self):
        api, _host = self.make_api()
        common = self.generated.PluginRunContext()
        ordering_payload = self.generated.RunCtxExtrusionOrdering(
            ctypes.c_void_p(1), ctypes.c_void_p(10)
        )
        common.step = self.generated.STEP_ORDERING
        common.data = ctypes.addressof(ordering_payload)
        ordering = self.execution.OrderingContext.from_ordering_run_context(
            api, ctypes.addressof(common)
        )
        self.assertIsInstance(ordering.ordering_plan(), self.views.MutablePrintingPlan)
        self.assertEqual(ordering.ordering_plan_handle(), 10)

        layer_payload = self.generated.RunCtxLayerExtrusionEdition(
            ctypes.c_void_p(1),
            ctypes.c_void_p(10),
            ctypes.c_void_p(20),
            ctypes.c_void_p(30),
            0,
            0,
        )
        common.step = self.generated.STEP_LAYER_EXTRUSION_EDIT
        common.data = ctypes.addressof(layer_payload)
        layer = self.execution.LayerExtrusionEditContext.from_layer_extrusion_edit_run_context(
            api, ctypes.addressof(common)
        )
        self.assertIsInstance(layer.layer_extrusion_edit_plan(), self.views.PrintingPlan)
        self.assertNotIsInstance(
            layer.layer_extrusion_edit_plan(), self.views.MutablePrintingPlan
        )
        self.assertIsInstance(
            layer.layer_extrusion_edit_group(), self.views.PrintingGroup
        )
        self.assertIsInstance(
            layer.layer_extrusion_edit_layer_group(),
            self.views.LayerExtrusionEditPrintingLayerGroup,
        )
        restricted_layer_group = layer.layer_extrusion_edit_layer_group()
        restricted_tool_group = restricted_layer_group.tool_group(0)
        restricted_extrusion = restricted_tool_group.extrusion(0)
        self.assertEqual(restricted_extrusion.mutable_root().address, 120)
        self.assertFalse(hasattr(restricted_layer_group, "set_print_z"))
        self.assertFalse(hasattr(restricted_layer_group, "append_tool_group"))
        self.assertFalse(hasattr(restricted_tool_group, "move_extrusion"))
        self.assertFalse(hasattr(restricted_extrusion, "set_root_move"))
        self.assertFalse(hasattr(restricted_extrusion, "set_role"))

        class Payload(ctypes.Structure):
            _fields_ = [("value", ctypes.c_uint32)]
            property_type = 700

        native_payload = Payload(42)
        _host.plugin_property_data_size = lambda *_args: ctypes.sizeof(Payload)
        _host.plugin_property_data = lambda *_args: ctypes.addressof(native_payload)
        _host.plugin_property_data_mutable = lambda *_args: ctypes.addressof(native_payload)
        for view in (
            layer.layer_extrusion_edit_plan(),
            layer.layer_extrusion_edit_group(),
            restricted_layer_group,
            restricted_tool_group,
        ):
            properties = view.properties()
            for forbidden in ("get_mutable", "get_or_add", "remove", "clear",
                              "copy_from", "data_address", "data_mutable_address"):
                self.assertFalse(hasattr(properties, forbidden), forbidden)
            snapshot = properties.get(Payload)
            snapshot.value = 99
            self.assertEqual(native_payload.value, 42)

        # Full-plan editing retains the writable metadata interface.
        writable = ordering.ordering_plan().properties().get_mutable(Payload)
        writable.value = 73
        self.assertEqual(native_payload.value, 73)


if __name__ == "__main__":
    unittest.main()
