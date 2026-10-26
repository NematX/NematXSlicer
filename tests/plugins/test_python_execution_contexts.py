"""Behavior tests for Python step contexts, progress and cooperative cancellation."""

import ctypes
import gc
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "src/libslic3r/Api/plugin/python"
sys.dont_write_bytecode = True
sys.path.insert(0, str(RUNTIME))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PythonExecutionContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_module("execution_context_api_generator", RUNTIME / "generate_slic3r_api.py")
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.run_context = importlib.import_module("slic3r_run_context")
        cls.execution = importlib.import_module("steps.execution")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_common(self, step, payload, *, cancelled=False, progress=None, errors=None):
        generated = self.generated
        keepalive = []
        cancel_cb = generated.PLUGIN_IS_CANCELLED(lambda _ctx: 1 if cancelled else 0)
        warning_cb = generated.PLUGIN_REPORT(lambda _ctx, _message: None)
        error_cb = generated.PLUGIN_REPORT(
            lambda _ctx, message: errors.append(message.decode("utf-8")) if errors is not None else None
        )
        progress_cb = generated.PLUGIN_REPORT_PROGRESS(
            lambda _ctx, value, message: progress.append(
                (float(value), None if not message else message.decode("utf-8"))
            ) if progress is not None else None
        )
        keepalive.extend((cancel_cb, warning_cb, error_cb, progress_cb, payload))
        common = generated.PluginRunContext(
            step=int(step),
            print=ctypes.c_void_p(902),
            plugin_storage=ctypes.c_void_p(900),
            data=ctypes.c_void_p(ctypes.addressof(payload)),
            host_context=ctypes.c_void_p(901),
            is_cancelled=cancel_cb,
            report_warning=warning_cb,
            report_error=error_cb,
            report_progress=progress_cb,
        )
        keepalive.append(common)
        return common, keepalive

    def test_plugin_base_distinguishes_default_and_absent_exclusive_groups(self):
        singleton = self.api_module.PluginBase("test.singleton", 1)
        service = self.api_module.PluginBase("test.service", 1, exclusive_group="")

        self.assertEqual(singleton.exclusive_group, "test.singleton")
        self.assertEqual(service.exclusive_group, "")

    def test_low_level_polyholes_releases_partial_allocations_on_failure(self):
        """Inject allocation and point-building failures; every created polygon is freed without touching an unrelated storage allocation."""
        polyholes = load_module(
            "execution_context_polyholes_low_level",
            ROOT / "src/plugins_python/polyholes/polyholes_low_level.py",
        )
        for failure in ("allocation", "null_allocation", "points"):
            with self.subTest(failure=failure):
                live = {999}
                allocated = []
                released = []

                def allocate(storage):
                    self.assertEqual(storage, 900)
                    if len(allocated) == 2 and failure != "points":
                        if failure == "null_allocation":
                            return 0
                        raise MemoryError("Polygon allocation failed")
                    handle = len(allocated) + 1
                    allocated.append(handle)
                    live.add(handle)
                    return handle

                def release(storage, handle):
                    self.assertEqual(storage, 900)
                    live.remove(handle)
                    released.append(handle)
                    return 1

                def push_point(handle, point):
                    raise RuntimeError("Polygon point construction failed")

                api = types.SimpleNamespace(
                    host=types.SimpleNamespace(storage_free=release),
                    storage_new_polygon=allocate,
                    polygon_push_back=push_point,
                    polygon_make_clockwise=lambda handle: None,
                )
                with patch.object(polyholes, "throw_if_cancelled", return_value=None):
                    with self.assertRaises(MemoryError if failure != "points" else RuntimeError):
                        polyholes._create_polyholes(
                            api, 900, self.api_module.CPoint(0, 0),
                            self.api_module.scale_i(2.5), self.api_module.scale_i(0.4),
                            True, 0.0, 0,
                        )
                self.assertEqual(len(allocated), 5 if failure == "points" else 2)
                self.assertEqual(released, allocated)
                self.assertEqual(live, {999})

    def test_plugin_base_does_not_opt_into_clear(self):
        plugin = self.api_module.PluginBase("test.no_clear", 1)
        self.assertFalse(hasattr(plugin, "clear"))

    def test_simple_context_validates_step_and_wraps_borrowed_handles(self):
        payload = self.generated.RunCtxPrepareForPerimeters(ctypes.c_void_p(11), ctypes.c_void_p(12))
        common, keepalive = self.make_common(self.generated.STEP_PRE_PERIMETER, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace())

        context = self.execution.PrePerimeterContext.from_pre_perimeter_run_context(
            api, ctypes.addressof(common)
        )
        self.assertIsNotNone(context)
        self.assertEqual(context.pre_perimeter_print().address, 11)
        self.assertEqual(context.pre_perimeter_object().address, 12)
        common.step = self.generated.STEP_PRE_INFILL
        self.assertIsNone(self.execution.PrePerimeterContext.from_pre_perimeter_run_context(
            api, ctypes.addressof(common)
        ))
        self.assertTrue(keepalive)

    def test_plan_and_gcode_contexts_copy_only_the_output_path(self):
        firmware = self.generated.RawGcodeFirmwareInstance()
        published = []

        @self.generated.GCODE_PUBLISH_READER_SOURCE_PATH
        def publish(_context, path):
            published.append(path.decode("utf-8"))
            return 1

        payload = self.generated.RunCtxGenerateGcode(
            ctypes.c_void_p(21), ctypes.c_void_p(22), ctypes.pointer(firmware),
            b"out/test.gcode", ctypes.c_void_p(23), publish,
        )
        common, keepalive = self.make_common(self.generated.STEP_GCODE, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace())
        context = self.execution.GCodeContext.from_gcode_run_context(api, ctypes.addressof(common))

        self.assertEqual(context.gcode_print().address, 21)
        self.assertEqual(context.gcode_plan_handle(), 22)
        plan = context.gcode_plan()
        self.assertEqual(plan.address, 22)
        self.assertIsInstance(plan, self.execution.PrintingPlan)
        self.assertNotIsInstance(plan, self.execution.MutablePrintingPlan)
        self.assertEqual(
            context.gcode_firmware_handle(), ctypes.addressof(firmware)
        )
        self.assertEqual(context.gcode_output_path(), "out/test.gcode")
        context.publish_gcode_reader_source_path("viewer/temp.gcode")
        self.assertEqual(published, ["viewer/temp.gcode"])
        self.assertTrue(keepalive)

    def test_infill_pattern_params_are_copied_but_output_is_borrowed(self):
        params = self.generated.RawInfillPatternParams()
        params.pattern_id = 42
        payload = self.generated.RunCtxInfillPattern(
            ctypes.c_void_p(1), ctypes.c_void_p(2), ctypes.c_void_p(3), ctypes.c_void_p(4),
            ctypes.c_void_p(5), ctypes.c_void_p(6), ctypes.c_void_p(7), ctypes.c_void_p(8),
            ctypes.pointer(params), ctypes.c_void_p(9),
        )
        common, keepalive = self.make_common(self.generated.INFILL_PATTERN, payload)
        context = self.execution.InfillPatternContext.from_infill_pattern_run_context(
            types.SimpleNamespace(host=types.SimpleNamespace()), ctypes.addressof(common)
        )

        copied = context.infill_pattern_params()
        copied.pattern_id = 99
        self.assertEqual(params.pattern_id, 42)
        self.assertEqual(context.infill_pattern_output().address, 9)
        self.assertTrue(keepalive)

    def test_wipe_tower_context_borrows_print_and_mutable_whole_plan(self):
        """Match the whole-plan C payload and reject a missing plan or wrong step."""
        payload = self.generated.RunCtxGenerateWipeTower()
        payload.print = ctypes.c_void_p(71)
        payload.plan = ctypes.c_void_p(72)
        common, keepalive = self.make_common(self.generated.STEP_WIPETOWER, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace())
        context = self.execution.WipeTowerContext.from_wipe_tower_run_context(
            api, ctypes.addressof(common)
        )
        self.assertEqual(context.wipe_tower_print().address, 71)
        self.assertEqual(context.wipe_tower_plan().address, 72)
        self.assertIsInstance(context.wipe_tower_plan(), self.execution.MutablePrintingPlan)
        payload.plan = None
        self.assertIsNone(self.execution.WipeTowerContext.from_wipe_tower_run_context(
            api, ctypes.addressof(common)
        ))
        payload.plan = ctypes.c_void_p(72)
        common.step = self.generated.STEP_ORDERING
        self.assertIsNone(self.execution.WipeTowerContext.from_wipe_tower_run_context(
            api, ctypes.addressof(common)
        ))
        self.assertTrue(keepalive)

    def test_support_demand_edit_goes_through_the_run_context(self):
        generated = self.generated
        calls = []

        def get_mutable(run_ctx, island):
            calls.append((int(run_ctx.value or 0), int(island.value or 0)))
            return 51

        payload = generated.RunCtxSupportDemand(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(generated.STEP_SUPPORT_DEMAND, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace(support_demand_get_island_mutable=get_mutable))
        context = self.execution.SupportDemandContext.from_support_demand_run_context(
            api, ctypes.addressof(common)
        )

        island = types.SimpleNamespace(c_handle=lambda: ctypes.c_void_p(41))
        demand = context.support_demand_get_island_mutable(island)
        self.assertEqual(demand.address, 51)
        self.assertEqual(calls, [(ctypes.addressof(common), 41)])
        self.assertTrue(keepalive is not None)

    def test_support_toolpath_context_borrows_and_validates_handles(self):
        """The toolpath context rejects wrong steps, missing handles and missing pattern callbacks."""
        generated = self.generated
        calls = []

        def resolve(ctx, plugin_id):
            calls.append(("resolve", ctypes.cast(ctx, ctypes.c_void_p).value, plugin_id))
            return 7

        def generate(ctx, pattern_id, layer, island, region_island, primary_region,
                     surface, no_overlap_areas, params, output):
            calls.append(("generate", ctypes.cast(ctx, ctypes.c_void_p).value, pattern_id, layer, island,
                          region_island, primary_region, surface, no_overlap_areas,
                          params.contents.density, output))
            return 1

        resolve_cb = generated.SUPPORT_TOOLPATH_RESOLVE_PATTERN_ID(resolve)
        generate_cb = generated.SUPPORT_TOOLPATH_GENERATE_PATTERN(generate)
        payload = generated.RunCtxSupportToolpath(
            ctypes.c_void_p(31), ctypes.c_void_p(32), ctypes.c_void_p(33), resolve_cb, generate_cb
        )
        common, keepalive = self.make_common(generated.STEP_SUPPORT_TOOLPATH, payload)
        keepalive.extend((resolve_cb, generate_cb))
        api = types.SimpleNamespace(host=types.SimpleNamespace())
        factory = self.execution.SupportToolpathContext.from_support_toolpath_run_context
        context = factory(api, ctypes.addressof(common))
        self.assertIsNotNone(context)
        self.assertEqual(context.support_toolpath_print().address, 31)
        self.assertEqual(context.support_toolpath_object().address, 32)
        self.assertIsInstance(self.api_module.Slic3rAPI.support_toolpath_context(api, ctypes.addressof(common)),
                              self.execution.SupportToolpathContext)

        # Both callbacks receive the payload itself as their context.
        self.assertEqual(context.support_toolpath_resolve_pattern_id("supportbase"), 7)
        params = generated.RawInfillPatternParams()
        params.density = 0.5
        self.assertTrue(context.support_toolpath_generate_pattern(7, 41, 42, 43, 44, 45, 46, params, 47))
        self.assertEqual(calls, [
            ("resolve", ctypes.addressof(payload), b"supportbase"),
            ("generate", ctypes.addressof(payload), 7, 41, 42, 43, 44, 45, 46, 0.5, 47),
        ])

        common.step = generated.STEP_SUPPORT
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        common.step = generated.STEP_SUPPORT_TOOLPATH
        payload.object = None
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        payload.object = ctypes.c_void_p(32)
        payload.print = None
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        payload.print = ctypes.c_void_p(31)
        payload.generate_pattern = generated.SUPPORT_TOOLPATH_GENERATE_PATTERN()
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        payload.generate_pattern = generate_cb
        payload.resolve_pattern_id = generated.SUPPORT_TOOLPATH_RESOLVE_PATTERN_ID()
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        common.data = None
        self.assertIsNone(factory(api, ctypes.addressof(common)))
        self.assertIsNone(factory(api, 0))
        self.assertTrue(keepalive)

    def test_progress_reports_visible_phase_and_finishes_after_all_runs(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 2, "Work {done}/{max}")
        progress.add_max(2)
        progress.increment()
        progress.finish_run()
        self.assertEqual(progress.done(), 1)
        progress.increment()
        progress.finish_run()

        self.assertEqual(progress.done(), 2)
        self.assertEqual(reports[-1], (1.0, "Work 2/2"))
        progress.increment_by(-1)
        self.assertEqual(progress.done(), 1)
        self.assertTrue(keepalive)

    def test_progress_finish_reports_terminal_completion_without_duplicates(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 1, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(4)
        progress.finish_run()
        self.assertEqual(reports[-1], (1.0, "Work 10/10"))
        self.assertEqual(progress.done(), 10)

        reports.clear()
        progress.reset(ctypes.addressof(common), 2, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(4)
        progress.finish_run()
        self.assertEqual(reports, [(0.4, "Work 4/10")])

        progress.finish_run()
        self.assertEqual(reports[-1], (1.0, "Work 10/10"))
        self.assertEqual(progress.done(), 10)
        report_count = len(reports)
        progress.finish_run()
        self.assertEqual(len(reports), report_count)

        reports.clear()
        progress.reset(ctypes.addressof(common), 1, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(10)
        self.assertEqual(len(reports), 1)
        progress.finish_run()
        self.assertEqual(len(reports), 1)
        self.assertTrue(keepalive)

    def test_progress_finish_keeps_the_next_phase_visible(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 1, "First {done}/{max}")
        progress.add_max(10, phase=0)
        progress.set_phase_format(1, "Second {done}/{max}")
        progress.add_max(5, phase=1)
        progress.increment_by(4, phase=0)
        progress.finish_run(phase=0)

        self.assertEqual(reports[-1], (0.0, "Second 0/5"))
        self.assertTrue(keepalive)

    def test_plugin_base_distinguishes_cancellation_from_failure(self):
        errors = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(self.generated.STEP_PRE_INFILL, payload, errors=errors)

        class Plugin(self.api_module.PluginBase):
            def __init__(self):
                super().__init__("test.execution", self_outer.generated.STEP_PRE_INFILL)

            def setup(self, _address, _count):
                raise self_outer.run_context.PluginCancelled()

            def run(self, _address):
                raise RuntimeError("broken worker")

        self_outer = self
        plugin = Plugin()
        plugin._slic3r_setup(ctypes.addressof(common), 3)
        self.assertEqual(plugin.progress.run_count(), 3)
        self.assertEqual(errors, [])
        plugin._slic3r_run(ctypes.addressof(common))
        self.assertEqual(errors, ["Python plugin run failed: broken worker"])
        self.assertTrue(keepalive)

    def test_plugin_base_reports_progress_initialization_failure(self):
        errors = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(self.generated.STEP_PRE_INFILL, payload, errors=errors)

        class Plugin(self.api_module.PluginBase):
            def __init__(self):
                super().__init__("test.progress.failure", self_outer.generated.STEP_PRE_INFILL)
                self.setup_called = False

            def progress_message_format(self):
                raise RuntimeError("invalid progress format")

            def setup(self, _address, _count):
                self.setup_called = True

        self_outer = self
        plugin = Plugin()
        plugin._slic3r_setup(ctypes.addressof(common), 1)

        self.assertFalse(plugin.setup_called)
        self.assertEqual(errors, ["Python plugin setup failed: invalid progress format"])
        self.assertTrue(keepalive)

    def test_parallel_helper_rethrows_worker_failure_after_native_callbacks(self):
        calls = []

        class Host:
            @staticmethod
            def slic3r_parallel_for_storage(begin, end, user_data, callback):
                for index in range(int(begin), int(end)):
                    callback(index, ctypes.c_void_p(100 + index), user_data)

        def worker(index, storage):
            calls.append((index, storage))
            if index == 1:
                raise ValueError("worker failed")

        api = types.SimpleNamespace(host=Host())
        with self.assertRaisesRegex(ValueError, "worker failed"):
            self.run_context.parallel_for_storage_with_progress(api, 0, 3, None, None, worker)
        self.assertEqual(calls, [(0, 100), (1, 101)])

    def test_parallel_helper_releases_chained_tracebacks_before_scratch_destruction(self):
        releases = []

        class Host:
            scratch_alive = False

            def slic3r_parallel_for_storage(self, begin, end, user_data, callback):
                for index in range(int(begin), int(end)):
                    self.scratch_alive = True
                    callback(index, ctypes.c_void_p(100 + index), user_data)
                    self.scratch_alive = False

        host = Host()

        class ScratchOwned:
            def __del__(self):
                releases.append(host.scratch_alive)

        def worker(_index, _storage):
            scratch_object = ScratchOwned()
            try:
                raise ValueError("inner failure")
            except ValueError as inner:
                raise RuntimeError("outer failure") from inner

        api = types.SimpleNamespace(host=host)
        with self.assertRaisesRegex(RuntimeError, "outer failure"):
            self.run_context.parallel_for_storage_with_progress(api, 0, 1, None, None, worker)
        gc.collect()

        self.assertEqual(releases, [True])


if __name__ == "__main__":
    unittest.main()
