#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Provider and consumer wrappers for the ``GCODE_THUMBNAIL`` service.

The host renderer owns its RGBA8 memory. :class:`GCodeThumbnailRendererView`
therefore copies pixels before returning to plugin code. Encoder sessions own
their latest bytes and diagnostics until the next callback; the acquiring
consumer destroys the published instance exactly once when it closes the service.
"""

from __future__ import annotations

import ctypes
import threading
from dataclasses import dataclass

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    GCODE_THUMBNAIL,
    GCODE_THUMBNAIL_ENCODE,
    GCODE_THUMBNAIL_ENCODER_DESTROY,
    PluginRunContext,
    RAW_GCODE_THUMBNAIL_STATUS_ERROR,
    RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_THUMBNAIL_STATUS_SKIPPED,
    RAW_GCODE_THUMBNAIL_STATUS_SUCCESS,
    RAW_GCODE_THUMBNAIL_STATUS_UNSET,
    RAW_GCODE_THUMBNAIL_BACKGROUND_SOLID,
    RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM,
    RAW_GCODE_THUMBNAIL_COLOR_BY_FILAMENT,
    RAW_GCODE_THUMBNAIL_FRAMING_CUSTOM,
    RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT,
    RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8,
    RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE,
    RawGcodeThumbnailEncodeResult,
    RawGcodeThumbnailEncoderInstance,
    RawGcodeThumbnailImage,
    RawGcodeThumbnailRenderRequest,
    RawGcodeThumbnailRenderResult,
    RawGcodeThumbnailRenderer,
    RunCtxGcodeThumbnail,
)
from slic3r_datatree_views import Print


def _address(value) -> int:
    if value is None:
        return 0
    if isinstance(value, ctypes.c_void_p):
        return int(value.value or 0)
    return int(ctypes.cast(value, ctypes.c_void_p).value or 0)


def _raw_pointer(structure, field_name: str) -> int:
    field = getattr(type(structure), field_name)
    return int(
        ctypes.c_void_p.from_address(ctypes.addressof(structure) + field.offset).value
        or 0
    )


@dataclass(frozen=True)
class GCodeThumbnailRequest:
    """Owned Python snapshot of one firmware render request.

    The contained ctypes value is independent from the firmware callback's
    temporary storage, but remains mutable like every ctypes structure.
    """

    raw: RawGcodeThumbnailRenderRequest

    def __post_init__(self) -> None:
        if not isinstance(self.raw, RawGcodeThumbnailRenderRequest):
            raise TypeError("raw must be a RawGcodeThumbnailRenderRequest")
        # Keep an independent byte-for-byte value copy so firmware callbacks may
        # reuse or overwrite their C result immediately after this returns.
        request = RawGcodeThumbnailRenderRequest.from_buffer_copy(bytes(self.raw))
        object.__setattr__(self, "raw", request)
        reserved = tuple(int(value) for value in request.reserved)
        if (
            int(request.struct_size) < ctypes.sizeof(RawGcodeThumbnailRenderRequest)
            or not 0 < int(request.width) <= 2048
            or not 0 < int(request.height) <= 2048
            or not 0 <= int(request.camera) <= int(RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM)
            or not 0 <= int(request.projection) <= int(RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE)
            or not 0 <= int(request.framing) <= int(RAW_GCODE_THUMBNAIL_FRAMING_CUSTOM)
            or not 0 <= int(request.color_mode) <= int(RAW_GCODE_THUMBNAIL_COLOR_BY_FILAMENT)
            or not 0 <= int(request.background) <= int(RAW_GCODE_THUMBNAIL_BACKGROUND_SOLID)
            or int(request.show_bed) not in (0, 1)
            or int(request.printable_only) not in (0, 1)
            or int(request.parts_only) not in (0, 1)
            or int(request.with_format_tag) not in (0, 1)
            or any(reserved)
        ):
            raise ValueError("the G-code thumbnail render request is invalid")
        # Camera geometry is reserved: the current host always uses auto framing.

    @property
    def width(self) -> int:
        return int(self.raw.width)

    @property
    def height(self) -> int:
        return int(self.raw.height)


@dataclass(frozen=True)
class GCodeThumbnailImage:
    """Owned RGBA8 image copied from the short-lived host render result."""

    pixels: bytes
    width: int
    height: int
    stride_bytes: int
    origin: int


@abi_requires_headers("slic3r_gcode_thumbnail.h")
class GCodeThumbnailRendererView:
    """Borrow a renderer while copying every returned image immediately."""

    def __init__(self, renderer=None) -> None:
        self._renderer = (
            ctypes.cast(renderer, ctypes.POINTER(RawGcodeThumbnailRenderer))
            if _address(renderer)
            else None
        )
        if self._renderer:
            raw = self._renderer.contents
            if (
                int(raw.struct_size) < ctypes.sizeof(RawGcodeThumbnailRenderer)
                or not raw.render
            ):
                raise ValueError("the G-code thumbnail renderer is incomplete")

    def valid(self) -> bool:
        return bool(self._renderer)

    def render(self, request: GCodeThumbnailRequest) -> GCodeThumbnailImage | None:
        if not isinstance(request, GCodeThumbnailRequest):
            raise TypeError("request must be a GCodeThumbnailRequest")
        if not self._renderer:
            return None
        result = RawGcodeThumbnailRenderResult()
        result.struct_size = ctypes.sizeof(result)
        result.image.struct_size = ctypes.sizeof(RawGcodeThumbnailImage)
        renderer = self._renderer.contents
        renderer.render(renderer.context, ctypes.byref(request.raw), ctypes.byref(result))
        if int(result.status) == int(RAW_GCODE_THUMBNAIL_STATUS_SKIPPED):
            return None
        if int(result.status) != int(RAW_GCODE_THUMBNAIL_STATUS_SUCCESS):
            message = result.error_message.decode("utf-8", errors="replace") if result.error_message else "thumbnail rendering failed"
            raise RuntimeError(message)
        image = result.image
        minimum_stride = int(image.width) * 4
        required_size = (
            (int(image.height) - 1) * int(image.stride_bytes) + minimum_stride
            if int(image.height)
            else 0
        )
        if (
            int(image.struct_size) < ctypes.sizeof(RawGcodeThumbnailImage)
            or not image.pixels
            or int(image.width) != request.width
            or int(image.height) != request.height
            or int(image.stride_bytes) < minimum_stride
            or int(image.pixel_format) != int(RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8)
            or not 0 <= int(image.origin) <= int(RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT)
            or required_size > int(image.pixels_size)
        ):
            raise RuntimeError("the host returned an invalid RGBA8 thumbnail")
        return GCodeThumbnailImage(
            ctypes.string_at(image.pixels, required_size),
            int(image.width),
            int(image.height),
            int(image.stride_bytes),
            int(image.origin),
        )


@abi_requires_headers("slic3r_gcode_thumbnail.h")
class GCodeThumbnailEncoderSession:
    """Base class for one encoder instance owned by a single export."""

    def encode(self, request: GCodeThumbnailRequest) -> bytes:
        """Return one complete file block; ``b''`` skips an optional image."""
        raise NotImplementedError


class RawGcodeThumbnailEncoderVTable(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("destroy", GCODE_THUMBNAIL_ENCODER_DESTROY),
        ("encode", GCODE_THUMBNAIL_ENCODE),
    ]


class _PublishedEncoder:
    def __init__(self, session: GCodeThumbnailEncoderSession) -> None:
        self.session = session
        self.lock = threading.RLock()
        self.bytes = None
        self.error = None


_published_lock = threading.RLock()
_published: dict[int, _PublishedEncoder] = {}


def _destroy_encoder(session_address) -> None:
    try:
        with _published_lock:
            _published.pop(_address(session_address), None)
    except BaseException:
        return


def _encode(session_address, request_pointer, result_pointer) -> None:
    if not result_pointer:
        return
    try:
        result = result_pointer.contents
        if int(result.struct_size) < ctypes.sizeof(RawGcodeThumbnailEncodeResult):
            return
        result.status = RAW_GCODE_THUMBNAIL_STATUS_UNSET
        result.bytes = None
        result.bytes_size = 0
        result.error_message = None
        with _published_lock:
            owner = _published.get(_address(session_address))
        if owner is None or not request_pointer:
            result.status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT
            return
        with owner.lock:
            owner.bytes = None
            owner.error = None
            try:
                if int(request_pointer.contents.struct_size) < ctypes.sizeof(RawGcodeThumbnailRenderRequest):
                    raise ValueError("the thumbnail request is truncated")
                encoded = owner.session.encode(GCodeThumbnailRequest(request_pointer.contents))
                if not isinstance(encoded, bytes):
                    raise TypeError("thumbnail encode() must return bytes")
                if not encoded:
                    result.status = RAW_GCODE_THUMBNAIL_STATUS_SKIPPED
                    return
                owner.bytes = (ctypes.c_uint8 * len(encoded)).from_buffer_copy(encoded)
                result.bytes = ctypes.cast(owner.bytes, ctypes.POINTER(ctypes.c_uint8))
                result.bytes_size = len(encoded)
                result.status = RAW_GCODE_THUMBNAIL_STATUS_SUCCESS
            except BaseException as exc:
                owner.error = ctypes.create_string_buffer(
                    (str(exc) or exc.__class__.__name__).encode("utf-8", errors="replace")
                )
                result.status = RAW_GCODE_THUMBNAIL_STATUS_ERROR
                result.error_message = ctypes.cast(owner.error, ctypes.c_char_p)
    except BaseException:
        return


_destroy_callback = GCODE_THUMBNAIL_ENCODER_DESTROY(_destroy_encoder)
_encode_callback = GCODE_THUMBNAIL_ENCODE(_encode)
_vtable = RawGcodeThumbnailEncoderVTable(
    ctypes.sizeof(RawGcodeThumbnailEncoderVTable),
    _destroy_callback,
    _encode_callback,
)


def make_gcode_thumbnail_encoder_instance(
    session: GCodeThumbnailEncoderSession,
) -> RawGcodeThumbnailEncoderInstance:
    """Transfer a fresh Python session to a host-owned C instance."""
    if not isinstance(session, GCodeThumbnailEncoderSession):
        raise TypeError("session must derive from GCodeThumbnailEncoderSession")
    if getattr(session, "_slic3r_thumbnail_consumed", False):
        raise ValueError("the thumbnail encoder session has already been consumed")
    session._slic3r_thumbnail_consumed = True
    owner = _PublishedEncoder(session)
    key = id(owner)
    with _published_lock:
        _published[key] = owner
    return RawGcodeThumbnailEncoderInstance(
        ctypes.sizeof(RawGcodeThumbnailEncoderInstance),
        ctypes.c_void_p(key),
        ctypes.cast(ctypes.pointer(_vtable), ctypes.c_void_p),
    )


@abi_requires_headers("slic3r_gcode_thumbnail.h")
class GCodeThumbnailEncoderView:
    """Borrow an encoder and copy its returned bytes before the next call."""

    def __init__(self, instance=None) -> None:
        self._instance = (
            ctypes.cast(instance, ctypes.POINTER(RawGcodeThumbnailEncoderInstance))
            if _address(instance)
            else None
        )
        if self._instance and self._vtable() is None:
            raise ValueError("the G-code thumbnail encoder instance is incomplete")

    def _vtable(self):
        if not self._instance:
            return None
        raw = self._instance.contents
        if not raw.session or not raw.vtable or int(raw.struct_size) < ctypes.sizeof(RawGcodeThumbnailEncoderInstance):
            return None
        table = ctypes.cast(raw.vtable, ctypes.POINTER(RawGcodeThumbnailEncoderVTable))
        if int(table.contents.struct_size) < ctypes.sizeof(RawGcodeThumbnailEncoderVTable) or not table.contents.destroy or not table.contents.encode:
            return None
        return table

    def valid(self) -> bool:
        return self._vtable() is not None

    def c_handle(self):
        """Return the borrowed C instance pointer for one synchronous callback."""
        return self._instance

    def encode(self, request: GCodeThumbnailRequest) -> bytes | None:
        if not self.valid():
            return None
        result = RawGcodeThumbnailEncodeResult()
        result.struct_size = ctypes.sizeof(result)
        self._vtable().contents.encode(
            self._instance.contents.session,
            ctypes.byref(request.raw),
            ctypes.byref(result),
        )
        if int(result.status) == int(RAW_GCODE_THUMBNAIL_STATUS_SKIPPED):
            return None
        if int(result.status) != int(RAW_GCODE_THUMBNAIL_STATUS_SUCCESS):
            message = result.error_message.decode("utf-8", errors="replace") if result.error_message else "thumbnail encoding failed"
            raise RuntimeError(message)
        address = _raw_pointer(result, "bytes")
        if not address and int(result.bytes_size):
            raise RuntimeError("the thumbnail encoder returned missing bytes")
        return ctypes.string_at(address, int(result.bytes_size)) if address else b""


@abi_requires_headers("steps/slic3r_step_gcode_thumbnail.h")
class GCodeThumbnailContext:
    """Borrow a service payload and publish one encoder session."""

    def __init__(self, api, common: PluginRunContext, payload: RunCtxGcodeThumbnail) -> None:
        self.api = api
        self.common = common
        self.payload = payload

    @classmethod
    def from_gcode_thumbnail_run_context(cls, api, address: int):
        if not address:
            return None
        common = PluginRunContext.from_address(int(address))
        if int(common.step) != int(GCODE_THUMBNAIL) or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxGcodeThumbnail)).contents
        return cls(api, common, payload)

    def renderer(self) -> GCodeThumbnailRendererView:
        return GCodeThumbnailRendererView(self.payload.renderer)

    def print(self) -> Print:
        """Borrow the immutable Print whose scene and settings are rendered."""
        if not self.payload.print:
            raise ValueError("the thumbnail context has no Print")
        return Print(self.api, self.payload.print)

    def publish(self, session: GCodeThumbnailEncoderSession) -> None:
        if self.payload.instance.session or self.payload.instance.vtable:
            raise ValueError("the thumbnail payload already owns a session")
        self.payload.instance = make_gcode_thumbnail_encoder_instance(session)


__all__ = [
    "GCodeThumbnailContext",
    "GCodeThumbnailEncoderSession",
    "GCodeThumbnailEncoderView",
    "GCodeThumbnailImage",
    "GCodeThumbnailRendererView",
    "GCodeThumbnailRequest",
    "RawGcodeThumbnailEncoderVTable",
    "make_gcode_thumbnail_encoder_instance",
]


@abi_requires_headers("steps/slic3r_step_gcode_thumbnail.h", "slic3r_orchestrator.h")
class GCodeThumbnailEncoder:
    """Own a service instance acquired through the public orchestrator.

    Keep the Print, renderer and orchestrator alive until close(). Calls are
    serial. Explicit ids never fall back. Copies are forbidden; use with.
    """
    def __init__(self, orchestrator, print, renderer, provider_id=""):
        """Acquire the explicit encoder, or use the print's setting if omitted; invalid choices raise ValueError before execution."""
        from slic3r_api_generated import RAW_PLUGIN_EXECUTION_SUCCESS
        self._instance = None
        self._keepalive = (orchestrator, print, renderer)
        config = print.config() if not provider_id else None
        requested_id = provider_id
        if not provider_id:
            option = config.get("thumbnails_format")
            requested_id = option.serialize() if option is not None else ""
        plugin = (orchestrator.find_plugin(provider_id) if provider_id else
                  orchestrator.select_plugin(config, GCODE_THUMBNAIL, "thumbnail_encoder_plugin"))
        if not provider_id and not plugin.valid() and requested_id:
            raise ValueError("GCODE_THUMBNAIL provider '" + requested_id + "' is not active")
        if not plugin.valid() or plugin.step() != int(GCODE_THUMBNAIL):
            raise ValueError("Unknown GCODE_THUMBNAIL provider: " + requested_id if requested_id else
                             "Invalid GCODE_THUMBNAIL provider selection")
        payload = RunCtxGcodeThumbnail()
        payload.print = print.c_handle()
        payload.renderer = renderer._renderer if renderer else None
        try:
            status = orchestrator.execute_one(plugin, payload, print=print)
            self._instance = RawGcodeThumbnailEncoderInstance.from_buffer_copy(bytes(payload.instance))
            if status != int(RAW_PLUGIN_EXECUTION_SUCCESS):
                raise RuntimeError("thumbnail provider execution failed: " + str(status))
            GCodeThumbnailEncoderView(ctypes.pointer(self._instance))
        except BaseException:
            self._instance = RawGcodeThumbnailEncoderInstance.from_buffer_copy(bytes(payload.instance))
            self.close()
            raise

    def encode(self, request):
        if self._instance is None:
            raise RuntimeError("thumbnail encoder is closed")
        return GCodeThumbnailEncoderView(ctypes.pointer(self._instance)).encode(request)

    def close(self):
        instance, self._instance = self._instance, None
        if instance is not None and instance.session and instance.vtable:
            table = ctypes.cast(instance.vtable, ctypes.POINTER(RawGcodeThumbnailEncoderVTable)).contents
            if table.struct_size >= RawGcodeThumbnailEncoderVTable.destroy.offset + ctypes.sizeof(ctypes.c_void_p) and table.destroy:
                table.destroy(instance.session)
        self._keepalive = None

    def __enter__(self):
        if self._instance is None:
            raise RuntimeError("thumbnail encoder is closed")
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        if getattr(self, "_instance", None) is not None:
            self.close()

    def __copy__(self):
        raise TypeError("thumbnail encoders cannot be copied")

    def __deepcopy__(self, memo):
        raise TypeError("thumbnail encoders cannot be copied")

__all__.append("GCodeThumbnailEncoder")
