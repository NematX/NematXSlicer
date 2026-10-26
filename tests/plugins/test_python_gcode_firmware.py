"""Behavior tests for the Python GCODE_FIRMWARE provider and consumer adapters."""

import ctypes
import gc
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
import weakref


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
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    try:
        return int(handle)
    except (TypeError, ValueError):
        return int(ctypes.cast(handle, ctypes.c_void_p).value or 0)


def native_fixture_path():
    """Locate the CMake-built native firmware callback probe."""
    names = (
        "python_gcode_firmware_native_fixture.dll",
        "libpython_gcode_firmware_native_fixture.so",
        "libpython_gcode_firmware_native_fixture.dylib",
    )
    directories = (
        ROOT / "build/tests/plugins/Debug",
        ROOT / "build/tests/plugins/Release",
        ROOT / "build/tests/plugins/RelWithDebInfo",
        ROOT / "build/tests/plugins",
    )
    for directory in directories:
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


class PythonGCodeFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "gcode_firmware_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.reader_service = importlib.import_module("slic3r_gcode_reader_service")
        cls.firmware = importlib.import_module("steps.gcode_firmware")
        cls.thumbnail = importlib.import_module("steps.gcode_thumbnail")
        cls.datatree = importlib.import_module("slic3r_datatree_views")
        cls.extrusion = importlib.import_module("slic3r_extrusion_views")
        cls.plan = importlib.import_module("slic3r_printing_plan_views")
        cls.execution = importlib.import_module("steps.execution")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_reader_identity_and_custom_range_constants_are_generated(self):
        """Keep generated Python Reader identifiers aligned with the C API version."""
        self.assertEqual(self.generated.SLIC3R_PLUGIN_API_GCODE_READER_MINOR, 3)
        self.assertEqual(self.generated.RAW_GCODE_READER_MARKER_SEAM, 1)
        self.assertEqual(self.generated.RAW_GCODE_READER_MARKER_CUSTOM_GCODE, 4)
        self.assertNotIn("source", dict(self.generated.RawGcodeReaderEvent._fields_))
        self.assertGreater(
            self.generated.RAW_GCODE_READER_ITEM_READER_ID,
            self.generated.RAW_GCODE_READER_ITEM_LINE_NUMBER,
        )
        self.assertEqual(
            self.generated.RAW_GCODE_READER_ITEM_CUSTOM_BEGIN,
            0x80000000,
        )

    def test_reader_marker_is_typed_and_event_only(self):
        generated = self.generated
        items = (generated.RawGcodeReaderEventItem * 2)(
            generated.RawGcodeReaderEventItem(
                generated.RAW_GCODE_READER_ITEM_MARKER,
                0,
                generated.RAW_GCODE_READER_MARKER_PAUSE_PRINT,
            ),
            generated.RawGcodeReaderEventItem(
                generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0
            ),
        )
        event = generated.RawGcodeReaderEvent(
            generated.RAW_GCODE_READER_EVENT_MARKER, items, len(items)
        )
        copied = self.reader_service._copied_items(event, event.kind)
        self.assertEqual(copied[0][2], generated.RAW_GCODE_READER_MARKER_PAUSE_PRINT)

        with self.assertRaisesRegex(ValueError, "complete.*event-local marker"):
            self.reader_service._copied_items(event)

        line_only = (generated.RawGcodeReaderEventItem * 1)(
            generated.RawGcodeReaderEventItem(
                generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0
            )
        )
        missing = generated.RawGcodeReaderEvent(
            generated.RAW_GCODE_READER_EVENT_MARKER, line_only, len(line_only)
        )
        with self.assertRaisesRegex(ValueError, "needs a marker type"):
            self.reader_service._copied_items(missing, missing.kind)

    def test_reader_diagnostic_is_typed_and_event_only(self):
        generated = self.generated
        for diagnostic_type in range(
            generated.RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_COMMAND,
            generated.RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT + 1,
        ):
            items = (generated.RawGcodeReaderEventItem * 2)(
                generated.RawGcodeReaderEventItem(
                    generated.RAW_GCODE_READER_ITEM_DIAGNOSTIC_TYPE,
                    0,
                    diagnostic_type,
                ),
                generated.RawGcodeReaderEventItem(
                    generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0
                ),
            )
            event = generated.RawGcodeReaderEvent(
                generated.RAW_GCODE_READER_EVENT_DIAGNOSTIC, items, len(items)
            )
            copied = self.reader_service._copied_items(event, event.kind)
            self.assertEqual(copied[0][2], diagnostic_type)

        with self.assertRaisesRegex(ValueError, "complete.*event-local diagnostic"):
            self.reader_service._copied_items(event)

        line_only = (generated.RawGcodeReaderEventItem * 1)(
            generated.RawGcodeReaderEventItem(
                generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0
            )
        )
        missing = generated.RawGcodeReaderEvent(
            generated.RAW_GCODE_READER_EVENT_DIAGNOSTIC,
            line_only,
            len(line_only),
        )
        with self.assertRaisesRegex(ValueError, "needs a diagnostic type"):
            self.reader_service._copied_items(missing, missing.kind)

        items[0].target = 1
        with self.assertRaisesRegex(ValueError, "diagnostic"):
            self.reader_service._copied_items(event, event.kind)
        items[0].target = 0
        items[0].value = 1.5
        with self.assertRaisesRegex(ValueError, "diagnostic"):
            self.reader_service._copied_items(event, event.kind)
        items[0].value = (
            generated.RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT + 1
        )
        with self.assertRaisesRegex(ValueError, "declared diagnostic type"):
            self.reader_service._copied_items(event, event.kind)

    def test_reader_service_routes_through_the_generic_orchestrator(self):
        seen = {}
        generated = self.generated

        class Host:
            def orchestrator_find_plugin(self, _orchestrator, plugin_id):
                seen["id"] = plugin_id.decode("utf-8")
                return 21 if seen["id"] == "gcode.reader.test" else 0

            @staticmethod
            def orchestrator_plugin_step(_plugin):
                return generated.GCODE_READER

            def orchestrator_execute_plugin(
                self, _orchestrator, _plugin, _print, payloads, payload_count
            ):
                self_outer.assertEqual(payload_count, 1)
                payload = ctypes.cast(
                    payloads[0].data, ctypes.POINTER(generated.RunCtxGcodeReader)
                ).contents
                chunks = []
                while True:
                    chunk = generated.RawGcodeReaderChunk()
                    result = generated.RawGcodeReaderResult()
                    payload.get_next_chunk(
                        payload.callback_context,
                        ctypes.byref(chunk),
                        ctypes.byref(result),
                    )
                    self_outer.assertEqual(
                        result.status, generated.RAW_GCODE_READER_STATUS_SUCCESS
                    )
                    if not chunk.bytes and chunk.byte_count == 0:
                        break
                    chunks.append(ctypes.string_at(chunk.bytes, chunk.byte_count))
                seen["text"] = b"".join(chunks)
                result = generated.RawGcodeReaderResult()
                payload.emit_final_state(
                    payload.callback_context,
                    payload.initial_state,
                    ctypes.byref(result),
                )
                self_outer.assertEqual(
                    result.status, generated.RAW_GCODE_READER_STATUS_SUCCESS
                )
                return generated.RAW_PLUGIN_EXECUTION_SUCCESS

        self_outer = self
        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(1), host=Host())
        service = self.firmware.GCodeReaderService(api)
        items = ((self.generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 7.0),)

        self.assertEqual(
            service.read("gcode.reader.test", items, ("G1 ", "X4\n")), items
        )
        self.assertEqual(seen, {"id": "gcode.reader.test", "text": b"G1 X4\n"})

    def test_reader_service_callback_contract_is_absent_from_generated_api(self):
        self.assertFalse(hasattr(self.generated, "RawGcodeReaderService"))
        self.assertFalse(hasattr(self.generated, "GCODE_READER_SERVICE_READ"))
        self.assertNotIn(
            "reader_service",
            dict(self.generated.RunCtxGcodeFirmware._fields_),
        )

    def test_reader_service_rejects_selection_and_completion_errors(self):
        generated = self.generated
        items = ((generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0),)

        class Host:
            selected = ""
            execution_status = generated.RAW_PLUGIN_EXECUTION_SUCCESS

            def orchestrator_find_plugin(self, _orchestrator, plugin_id):
                self.selected = plugin_id.decode("utf-8")
                return 0 if self.selected == "missing" else 31

            def orchestrator_plugin_step(self, _plugin):
                return 999 if self.selected == "wrong.step" else generated.GCODE_READER

            def orchestrator_execute_plugin(
                self, _orchestrator, _plugin, _print, _payloads, _payload_count
            ):
                return self.execution_status

        host = Host()
        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(1), host=host)
        service = self.firmware.GCodeReaderService(api)

        with self.assertRaisesRegex(RuntimeError, "activation set"):
            service.read("missing", items, "")
        with self.assertRaisesRegex(RuntimeError, "not a G-code reader"):
            service.read("wrong.step", items, "")
        with self.assertRaisesRegex(RuntimeError, "without a final state"):
            service.read("valid.reader", items, "")

        host.execution_status = generated.RAW_PLUGIN_EXECUTION_INACTIVE
        with self.assertRaisesRegex(RuntimeError, "no longer active"):
            service.read("valid.reader", items, "")

    def test_reader_service_rejects_premature_and_duplicate_final_states(self):
        generated = self.generated
        items = ((generated.RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0),)

        class Host:
            mode = "premature"

            @staticmethod
            def orchestrator_find_plugin(_orchestrator, _plugin_id):
                return 41

            @staticmethod
            def orchestrator_plugin_step(_plugin):
                return generated.GCODE_READER

            def orchestrator_execute_plugin(
                self, _orchestrator, _plugin, _print, payloads, _payload_count
            ):
                payload = ctypes.cast(
                    payloads[0].data, ctypes.POINTER(generated.RunCtxGcodeReader)
                ).contents
                result = generated.RawGcodeReaderResult()
                if self.mode == "premature":
                    payload.emit_final_state(
                        payload.callback_context,
                        payload.initial_state,
                        ctypes.byref(result),
                    )
                    return generated.RAW_PLUGIN_EXECUTION_SUCCESS

                while True:
                    chunk = generated.RawGcodeReaderChunk()
                    payload.get_next_chunk(
                        payload.callback_context,
                        ctypes.byref(chunk),
                        ctypes.byref(result),
                    )
                    if not chunk.bytes and chunk.byte_count == 0:
                        break
                payload.emit_final_state(
                    payload.callback_context,
                    payload.initial_state,
                    ctypes.byref(result),
                )
                payload.emit_final_state(
                    payload.callback_context,
                    payload.initial_state,
                    ctypes.byref(result),
                )
                return generated.RAW_PLUGIN_EXECUTION_SUCCESS

        host = Host()
        api = types.SimpleNamespace(orchestrator=ctypes.c_void_p(1), host=host)
        service = self.firmware.GCodeReaderService(api)
        with self.assertRaisesRegex(RuntimeError, "before end-of-stream"):
            service.read("reader", items, "G1 X1\n")

        host.mode = "duplicate"
        with self.assertRaisesRegex(RuntimeError, "more than one final state"):
            service.read("reader", items, "G1 X1\n")

    def test_thumbnail_result_is_copied_and_parameters_are_explicit(self):
        pixels = (ctypes.c_uint8 * 8)(*range(8))
        seen = []

        def render(_context, request, result):
            seen.append((request.contents.width, request.contents.height, request.contents.show_bed))
            result.contents.status = self.generated.RAW_GCODE_THUMBNAIL_STATUS_SUCCESS
            result.contents.image.struct_size = ctypes.sizeof(
                self.generated.RawGcodeThumbnailImage
            )
            result.contents.image.pixels = ctypes.cast(
                pixels, ctypes.POINTER(ctypes.c_uint8)
            )
            result.contents.image.pixels_size = len(pixels)
            result.contents.image.width = 2
            result.contents.image.height = 1
            result.contents.image.stride_bytes = 8
            result.contents.image.pixel_format = (
                self.generated.RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8
            )
            result.contents.image.origin = (
                self.generated.RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT
            )

        callback = self.generated.GCODE_THUMBNAIL_RENDER(render)
        raw = self.generated.RawGcodeThumbnailRenderer()
        raw.struct_size = ctypes.sizeof(raw)
        raw.render = callback
        request = self.generated.RawGcodeThumbnailRenderRequest()
        request.struct_size = ctypes.sizeof(request)
        request.width = 2
        request.height = 1
        request.show_bed = True
        wrapped = self.thumbnail.GCodeThumbnailRequest(request)
        view = self.thumbnail.GCodeThumbnailRendererView(ctypes.pointer(raw))
        first = view.render(wrapped)
        pixels[0] = 99
        self.assertEqual(first.pixels, bytes(range(8)))
        self.assertEqual(first.origin, self.generated.RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT)
        self.assertEqual(seen, [(2, 1, 1)])
        self.assertIsNone(self.thumbnail.GCodeThumbnailRendererView(None).render(wrapped))

    def test_thumbnail_request_owns_its_ctypes_snapshot_and_validates_custom_pose(self):
        raw = self.generated.RawGcodeThumbnailRenderRequest()
        raw.struct_size = ctypes.sizeof(raw)
        raw.width = 32
        raw.height = 16
        raw.custom_scale = 1.0
        request = self.thumbnail.GCodeThumbnailRequest(raw)
        raw.width = 99
        self.assertEqual(request.width, 32)

        invalid = self.generated.RawGcodeThumbnailRenderRequest()
        invalid.struct_size = ctypes.sizeof(invalid)
        invalid.height = 16
        with self.assertRaisesRegex(ValueError, "render request is invalid"):
            self.thumbnail.GCodeThumbnailRequest(invalid)

        custom = self.generated.RawGcodeThumbnailRenderRequest()
        custom.struct_size = ctypes.sizeof(custom)
        custom.width = 32
        custom.height = 16
        custom.camera = self.generated.RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM
        custom.camera_up.z = 1.0
        custom.custom_scale = 1.0
        self.thumbnail.GCodeThumbnailRequest(custom)  # Reserved camera fields are ignored.

    def test_thumbnail_encoder_copies_bytes_contains_errors_and_destroys_once(self):
        destroyed = []
        thumbnail = self.thumbnail

        class Session(thumbnail.GCodeThumbnailEncoderSession):
            def encode(self, request):
                if request.width == 7:
                    raise RuntimeError("python codec failed")
                if request.width == 8:
                    return b""
                return bytes((request.width, 0, request.height))

            def __del__(self):
                destroyed.append(1)

        session = Session()
        reference = weakref.ref(session)
        instance = thumbnail.make_gcode_thumbnail_encoder_instance(session)
        view = thumbnail.GCodeThumbnailEncoderView(ctypes.pointer(instance))
        raw = self.generated.RawGcodeThumbnailRenderRequest()
        raw.struct_size = ctypes.sizeof(raw)
        raw.width = 4
        raw.height = 3
        raw.custom_scale = 1.0
        request = thumbnail.GCodeThumbnailRequest(raw)
        self.assertEqual(view.encode(request), b"\x04\x00\x03")
        raw.width = 7
        with self.assertRaisesRegex(RuntimeError, "python codec failed"):
            view.encode(thumbnail.GCodeThumbnailRequest(raw))
        raw.width = 8
        self.assertIsNone(view.encode(thumbnail.GCodeThumbnailRequest(raw)))
        with self.assertRaisesRegex(ValueError, "already been consumed"):
            thumbnail.make_gcode_thumbnail_encoder_instance(session)

        table = ctypes.cast(
            instance.vtable, ctypes.POINTER(thumbnail.RawGcodeThumbnailEncoderVTable)
        )
        del session
        gc.collect()
        self.assertIsNotNone(reference())
        table.contents.destroy(instance.session)
        instance.session = None
        instance.vtable = None
        gc.collect()
        self.assertIsNone(reference())
        self.assertEqual(destroyed, [1])

    def test_owned_encoder_uses_orchestrator_and_cleans_failed_publication(self):
        import copy
        destroyed = []
        thumbnail = self.thumbnail
        generated = self.generated

        class Session(thumbnail.GCodeThumbnailEncoderSession):
            def encode(self, request): return b"copied"
            def __del__(self): destroyed.append(True)

        class Orchestrator:
            def __init__(self): self.status = generated.RAW_PLUGIN_EXECUTION_SUCCESS
            def find_plugin(self, identifier):
                self.identifier = identifier
                return types.SimpleNamespace(valid=lambda: True, step=lambda: int(generated.GCODE_THUMBNAIL))
            def select_plugin(self, config, step, group):
                self.group = group
                return self.find_plugin("selected")
            def execute_one(self, plugin, payload, print):
                payload.instance = thumbnail.make_gcode_thumbnail_encoder_instance(Session())
                return self.status

        orch = Orchestrator()
        print_view = types.SimpleNamespace(c_handle=lambda: ctypes.c_void_p(1), config=lambda: None)
        raw = generated.RawGcodeThumbnailRenderRequest()
        raw.struct_size = ctypes.sizeof(raw)
        raw.width = raw.height = 4
        request = thumbnail.GCodeThumbnailRequest(raw)
        with thumbnail.GCodeThumbnailEncoder(orch, print_view, None, "JPG") as encoder:
            self.assertEqual(orch.identifier, "JPG")
            self.assertEqual(encoder.encode(request), b"copied")
            with self.assertRaises(TypeError): copy.copy(encoder)
            with self.assertRaises(TypeError): copy.deepcopy(encoder)
        encoder.close()
        self.assertEqual(len(destroyed), 1)
        with self.assertRaises(RuntimeError): encoder.encode(request)
        orch.status = generated.RAW_PLUGIN_EXECUTION_PLUGIN_ERROR
        with self.assertRaises(RuntimeError): thumbnail.GCodeThumbnailEncoder(orch, print_view, None)
        gc.collect()
        self.assertEqual(orch.group, "thumbnail_encoder_plugin")
        self.assertEqual(len(destroyed), 2)

    def test_thumbnail_context_borrows_inputs_and_transfers_one_session(self):
        thumbnail = self.thumbnail

        class Session(thumbnail.GCodeThumbnailEncoderSession):
            def encode(self, _request):
                return b"block"

        payload = self.generated.RunCtxGcodeThumbnail()
        payload.print = ctypes.c_void_p(21)
        common = self.generated.PluginRunContext()
        common.step = self.generated.GCODE_THUMBNAIL
        common.data = ctypes.addressof(payload)
        context = thumbnail.GCodeThumbnailContext.from_gcode_thumbnail_run_context(
            self.api, ctypes.addressof(common)
        )
        self.assertEqual(context.print().address, 21)
        self.assertFalse(context.renderer().valid())
        context.publish(Session())
        with self.assertRaisesRegex(ValueError, "already owns"):
            context.publish(Session())
        table = ctypes.cast(
            payload.instance.vtable,
            ctypes.POINTER(thumbnail.RawGcodeThumbnailEncoderVTable),
        )
        table.contents.destroy(payload.instance.session)

    def test_thumbnail_wrappers_declare_minimal_contracts(self):
        header_attribute = "__slic3r_abi_headers__"
        self.assertIn(
            "slic3r_gcode_thumbnail.h",
            getattr(self.thumbnail.GCodeThumbnailRendererView, header_attribute),
        )
        self.assertIn(
            "slic3r_gcode_thumbnail.h",
            getattr(self.thumbnail.GCodeThumbnailEncoderView, header_attribute),
        )
        self.assertIn(
            "steps/slic3r_step_gcode_thumbnail.h",
            getattr(self.thumbnail.GCodeThumbnailContext, header_attribute),
        )

    def setUp(self):
        self.api = types.SimpleNamespace(host=types.SimpleNamespace())

    def destroy_instance(self, instance):
        vtable = ctypes.cast(
            instance.vtable, ctypes.POINTER(self.firmware.RawGcodeFirmwareVTable)
        )
        vtable.contents.destroy(instance.session)
        instance.session = None
        instance.vtable = None

    def complete_session(self, *, fail_at=None):
        calls = []
        firmware = self.firmware

        class Session(firmware.GCodeFirmwareSession):
            def reader_plugin_id(self):
                return "gcode.reader.test"

            def begin_print(self, value, plan):
                calls.append(("begin_print", value.address))
                return "print\n"

            def begin_group(self, value):
                calls.append(("begin_group", value.address))
                return "group\n"

            def begin_layer(self, value):
                calls.append(("begin_layer", value.address))
                return "layer\n"

            def begin_tool_group(self, value):
                calls.append(("begin_tool_group", value.address))
                return "tool\n"

            def write_extrusion(self, value):
                calls.append(("write_extrusion", value.address))
                return "A\0B"

            def write_event(self, value):
                calls.append(("write_event", value.address))
                if fail_at == "write_event":
                    raise RuntimeError("event failed")
                return "event\n"

            def end_tool_group(self):
                calls.append(("end_tool_group", None))
                return "end tool\n"

            def end_layer(self):
                calls.append(("end_layer", None))
                return "end layer\n"

            def end_group(self):
                calls.append(("end_group", None))
                return "end group\n"

            def end_print(self):
                calls.append(("end_print", None))
                return "end print\n"

        self_outer = self
        return Session(), calls

    def test_complete_callback_order_and_embedded_nul_text(self):
        session, calls = self.complete_session()
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        try:
            view = self.firmware.GCodeFirmwareView(self.api, instance)
            self.assertEqual(view.reader_plugin_id(), "gcode.reader.test")
            self.assertEqual(
                view.begin_print(
                    self.datatree.Print(self.api, 11),
                    self.plan.PrintingPlan(self.api, 12),
                ),
                "print\n",
            )
            self.assertEqual(view.begin_group(self.plan.PrintingGroup(self.api, 12)), "group\n")
            self.assertEqual(view.begin_layer(self.plan.PrintingLayerGroup(self.api, 13)), "layer\n")
            self.assertEqual(view.begin_tool_group(self.plan.PrintingToolGroup(self.api, 14)), "tool\n")
            self.assertEqual(view.write_extrusion(self.plan.PrintingExtrusion(self.api, 15)), "A\0B")
            self.assertEqual(view.write_event(self.extrusion.ExtrusionEntity(self.api, 16)), "event\n")
            self.assertEqual(view.end_tool_group(), "end tool\n")
            self.assertEqual(view.end_layer(), "end layer\n")
            self.assertEqual(view.end_group(), "end group\n")
            self.assertEqual(view.end_print(), "end print\n")
        finally:
            self.destroy_instance(instance)

        self.assertEqual(
            calls,
            [
                ("begin_print", 11),
                ("begin_group", 12),
                ("begin_layer", 13),
                ("begin_tool_group", 14),
                ("write_extrusion", 15),
                ("write_event", 16),
                ("end_tool_group", None),
                ("end_layer", None),
                ("end_group", None),
                ("end_print", None),
            ],
        )

    def test_exception_is_contained_and_diagnostic_lives_until_next_call(self):
        session, _calls = self.complete_session(fail_at="write_event")
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        published = self.firmware._published_sessions[address(instance.session)]
        view = self.firmware.GCodeFirmwareView(self.api, instance)
        try:
            with self.assertRaisesRegex(RuntimeError, "event failed"):
                view.write_event(self.extrusion.ExtrusionEntity(self.api, 16))
            self.assertIsNotNone(published.error)
            self.assertEqual(published.error.value, b"event failed")
            self.assertEqual(view.end_print(), "end print\n")
            self.assertIsNone(published.error)
        finally:
            self.destroy_instance(instance)
        self.assertIsNone(published.output)
        self.assertIsNone(published.error)

    def test_publication_transfers_unique_session_and_destroys_once(self):
        destroyed = []
        firmware = self.firmware

        class Session(firmware.GCodeFirmwareSession):
            def begin_print(self, _value, _plan): return ""
            def begin_group(self, _value): return ""
            def begin_layer(self, _value): return ""
            def begin_tool_group(self, _value): return ""
            def write_extrusion(self, _value): return ""
            def write_event(self, _value): return ""
            def end_tool_group(self): return ""
            def end_layer(self): return ""
            def end_group(self): return ""
            def end_print(self): return ""
            def __del__(self): destroyed.append(1)

        self_outer = self
        payload = self.generated.RunCtxGcodeFirmware()
        payload.print = ctypes.c_void_p(21)
        common = self.generated.PluginRunContext()
        common.step = self.generated.GCODE_FIRMWARE
        common.data = ctypes.addressof(payload)
        context = firmware.GCodeFirmwareContext.from_gcode_firmware_run_context(
            self.api, ctypes.addressof(common)
        )
        session = Session()
        reference = weakref.ref(session)
        context.publish_gcode_firmware(session)
        with self.assertRaisesRegex(ValueError, "already owns"):
            context.publish_gcode_firmware(session)
        with self.assertRaisesRegex(ValueError, "already been consumed"):
            firmware.make_gcode_firmware_instance(session, self.api)

        del session
        gc.collect()
        self.assertIsNotNone(reference())
        self.destroy_instance(payload.instance)
        gc.collect()
        self.assertIsNone(reference())
        self.assertEqual(destroyed, [1])

    def test_gcode_context_returns_borrowed_firmware_view(self):
        session, _calls = self.complete_session()
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        payload = self.generated.RunCtxGenerateGcode()
        payload.print = ctypes.c_void_p(1)
        payload.plan = ctypes.c_void_p(2)
        payload.firmware = ctypes.pointer(instance)
        payload.output_path = b"out.gcode"

        @self.generated.GCODE_PUBLISH_READER_SOURCE_PATH
        def publish_reader_source(_context, _path):
            return 1

        payload.publish_reader_source_path = publish_reader_source
        common = self.generated.PluginRunContext()
        common.step = self.generated.STEP_GCODE
        common.data = ctypes.addressof(payload)
        try:
            context = self.execution.GCodeContext.from_gcode_run_context(
                self.api, ctypes.addressof(common)
            )
            self.assertEqual(context.gcode_firmware().end_print(), "end print\n")
        finally:
            self.destroy_instance(instance)

    def test_script_processor_copies_text_and_invalidates_old_context(self):
        output = ctypes.create_string_buffer(b"script\0result")
        config_address = 401

        def prepare(_context, _script_type, _producer):
            return config_address

        def process(_context, _script, _extruder, result):
            result.contents.status = self.generated.RAW_GCODE_SCRIPT_STATUS_SUCCESS
            result.contents.text = ctypes.cast(output, ctypes.c_char_p)
            result.contents.text_size = len(b"script\0result")

        prepare_callback = self.generated.GCODE_SCRIPT_PREPARE(prepare)
        process_callback = self.generated.GCODE_SCRIPT_PROCESS(process)
        raw = self.generated.RawGcodeScriptProcessor(
            ctypes.sizeof(self.generated.RawGcodeScriptProcessor),
            ctypes.c_void_p(99),
            prepare_callback,
            process_callback,
        )
        processor = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )
        first = processor.prepare(self.generated.GCODE_SCRIPT_TYPE_START_GCODE)
        self.assertEqual(first.process("start", 0), "script\0result")
        second = processor.prepare(self.generated.GCODE_SCRIPT_TYPE_END_GCODE)
        with self.assertRaisesRegex(RuntimeError, "invalidated"):
            first.process("stale", 0)
        self.assertEqual(second.process("end", 1), "script\0result")

    def test_script_processor_invalidation_is_shared_between_views(self):
        output = ctypes.create_string_buffer(b"shared")

        def prepare(_context, script_type, _producer):
            return 500 + int(script_type)

        def process(_context, _script, _extruder, result):
            result.contents.status = self.generated.RAW_GCODE_SCRIPT_STATUS_SUCCESS
            result.contents.text = ctypes.cast(output, ctypes.c_char_p)
            result.contents.text_size = len(b"shared")

        prepare_callback = self.generated.GCODE_SCRIPT_PREPARE(prepare)
        process_callback = self.generated.GCODE_SCRIPT_PROCESS(process)
        raw = self.generated.RawGcodeScriptProcessor(
            ctypes.sizeof(self.generated.RawGcodeScriptProcessor),
            ctypes.c_void_p(99),
            prepare_callback,
            process_callback,
        )
        first_view = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )
        second_view = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )

        stale = first_view.prepare(self.generated.GCODE_SCRIPT_TYPE_START_GCODE)
        current = second_view.prepare(self.generated.GCODE_SCRIPT_TYPE_END_GCODE)

        with self.assertRaisesRegex(RuntimeError, "invalidated"):
            stale.process("stale", 0)
        self.assertEqual(current.process("current", 0), "shared")

    def test_invalid_instances_and_non_string_results_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.firmware.GCodeFirmwareView(
                self.api, self.generated.RawGcodeFirmwareInstance()
            )

        session, _calls = self.complete_session()
        session.end_print = lambda: b"bytes are not text"
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        try:
            with self.assertRaisesRegex(RuntimeError, "must return str"):
                self.firmware.GCodeFirmwareView(self.api, instance).end_print()
        finally:
            self.destroy_instance(instance)

    def test_output_and_error_survive_native_thread_state_detach(self):
        fixture_path = native_fixture_path()
        if fixture_path is None:
            self.skipTest("python_gcode_firmware_native_fixture has not been built")

        fixture = ctypes.CDLL(str(fixture_path))
        fixture.python_gcode_firmware_native_probe_create.argtypes = [
            ctypes.POINTER(self.generated.RawGcodeFirmwareInstance)
        ]
        fixture.python_gcode_firmware_native_probe_create.restype = ctypes.c_void_p
        fixture.python_gcode_firmware_native_probe_status.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_firmware_native_probe_status.restype = ctypes.c_int32
        for name in ("copy_text", "copy_error"):
            callback = getattr(fixture, f"python_gcode_firmware_native_probe_{name}")
            callback.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_char),
                ctypes.c_uint32,
            ]
            callback.restype = ctypes.c_uint32
        fixture.python_gcode_firmware_native_probe_destroy.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_firmware_native_probe_destroy.restype = None

        for fails, expected_status, expected in (
            (False, self.generated.RAW_GCODE_FIRMWARE_STATUS_SUCCESS, b"native\0text"),
            (True, self.generated.RAW_GCODE_FIRMWARE_STATUS_ERROR, b"native firmware failure"),
        ):
            with self.subTest(fails=fails):
                session, _calls = self.complete_session()
                if fails:
                    session.end_print = lambda: (_ for _ in ()).throw(
                        RuntimeError("native firmware failure")
                    )
                else:
                    session.end_print = lambda: "native\0text"
                instance = self.firmware.make_gcode_firmware_instance(session, self.api)
                probe = None
                try:
                    probe = fixture.python_gcode_firmware_native_probe_create(
                        ctypes.byref(instance)
                    )
                    self.assertTrue(probe)
                    self.assertEqual(
                        fixture.python_gcode_firmware_native_probe_status(probe),
                        expected_status,
                    )
                    output = ctypes.create_string_buffer(128)
                    copy = (
                        fixture.python_gcode_firmware_native_probe_copy_error
                        if fails
                        else fixture.python_gcode_firmware_native_probe_copy_text
                    )
                    length = copy(probe, output, len(output))
                    self.assertEqual(length, len(expected))
                    self.assertEqual(output.raw[:length], expected)
                finally:
                    if probe:
                        fixture.python_gcode_firmware_native_probe_destroy(probe)
                    self.destroy_instance(instance)

    def test_thumbnail_bytes_and_error_survive_native_thread_state_detach(self):
        fixture_path = native_fixture_path()
        if fixture_path is None:
            self.skipTest("python_gcode_firmware_native_fixture has not been built")

        fixture = ctypes.CDLL(str(fixture_path))
        fixture.python_gcode_thumbnail_native_probe_create.argtypes = [
            ctypes.POINTER(self.generated.RawGcodeThumbnailEncoderInstance),
            ctypes.POINTER(self.generated.RawGcodeThumbnailRenderRequest),
        ]
        fixture.python_gcode_thumbnail_native_probe_create.restype = ctypes.c_void_p
        fixture.python_gcode_thumbnail_native_probe_status.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_thumbnail_native_probe_status.restype = ctypes.c_int32
        for name in ("copy_bytes", "copy_error"):
            callback = getattr(fixture, f"python_gcode_thumbnail_native_probe_{name}")
            callback.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_char),
                ctypes.c_uint32,
            ]
            callback.restype = ctypes.c_uint32
        fixture.python_gcode_thumbnail_native_probe_destroy.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_thumbnail_native_probe_destroy.restype = None

        request = self.generated.RawGcodeThumbnailRenderRequest()
        request.struct_size = ctypes.sizeof(request)
        request.width = 4
        request.height = 3
        request.custom_scale = 1.0
        thumbnail = self.thumbnail

        for fails, expected_status, expected in (
            (False, self.generated.RAW_GCODE_THUMBNAIL_STATUS_SUCCESS,
             b"native\0thumbnail"),
            (True, self.generated.RAW_GCODE_THUMBNAIL_STATUS_ERROR,
             b"native thumbnail failure"),
        ):
            with self.subTest(fails=fails):
                class Session(thumbnail.GCodeThumbnailEncoderSession):
                    def encode(self, _request):
                        if fails:
                            raise RuntimeError("native thumbnail failure")
                        return b"native\0thumbnail"

                instance = thumbnail.make_gcode_thumbnail_encoder_instance(Session())
                probe = None
                try:
                    probe = fixture.python_gcode_thumbnail_native_probe_create(
                        ctypes.byref(instance), ctypes.byref(request)
                    )
                    self.assertTrue(probe)
                    self.assertEqual(
                        fixture.python_gcode_thumbnail_native_probe_status(probe),
                        expected_status,
                    )
                    output = ctypes.create_string_buffer(128)
                    copy = (
                        fixture.python_gcode_thumbnail_native_probe_copy_error
                        if fails
                        else fixture.python_gcode_thumbnail_native_probe_copy_bytes
                    )
                    length = copy(probe, output, len(output))
                    self.assertEqual(length, len(expected))
                    self.assertEqual(output.raw[:length], expected)
                finally:
                    if probe:
                        fixture.python_gcode_thumbnail_native_probe_destroy(probe)
                    table = ctypes.cast(
                        instance.vtable,
                        ctypes.POINTER(thumbnail.RawGcodeThumbnailEncoderVTable),
                    )
                    table.contents.destroy(instance.session)


if __name__ == "__main__":
    unittest.main()
