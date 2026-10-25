///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeThumbnailViews_hpp_
#define slic3r_GCodeThumbnailViews_hpp_

#include <algorithm>
#include <array>
#include <charconv>
#include <map>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_gcode_thumbnail.h"
#include "../OrchestratorViews.hpp"
#include "../DataTreeViews.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_thumbnail.h"

/*
The renderer view asks the host for pixels; an encoder session consumes those
pixels and returns the selected format's bytes. The provider adapter implements
the C callbacks for a C++ encoder, while the consumer view invokes an acquired
session. These are separate roles even when they participate in the same export.

Acquire the encoder, send a render/encode request, consume or copy its result,
then release the session. Do not save a returned byte pointer for another call:
the session may replace the buffer. The renderer's pixels stay host-owned.

G-code thumbnail C++ adapters
=============================

The renderer view borrows host-owned RGBA8 pixels for one synchronous call.
The encoder adapter owns one provider session per export and retains its last
byte block, its last compressed image (returned by the optional encode_image
callback) and its diagnostic. Each buffer is invalidated by the next call of
its own callback. The writer-facing view copies neither ownership nor
allocations across the C ABI.
*/

namespace slic3r_api {

/* Reject malformed requests before either a host renderer or provider sees
   them. Custom vectors are intentionally ignored for predefined cameras, so
   callers may leave those fields zero-initialized. */
inline void validate_gcode_thumbnail_render_request(
    const raw_gcode_thumbnail_render_request &request)
{
    const bool modes_valid =
        int(request.camera) >= int(RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC) &&
        request.camera <= RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM &&
        int(request.projection) >= int(RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC) &&
        request.projection <= RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE &&
        int(request.framing) >= int(RAW_GCODE_THUMBNAIL_FRAMING_AUTO) &&
        request.framing <= RAW_GCODE_THUMBNAIL_FRAMING_CUSTOM &&
        int(request.color_mode) >= int(RAW_GCODE_THUMBNAIL_COLOR_UNIFORM) &&
        request.color_mode <= RAW_GCODE_THUMBNAIL_COLOR_BY_FILAMENT &&
        int(request.background) >= int(RAW_GCODE_THUMBNAIL_BACKGROUND_TRANSPARENT) &&
        request.background <= RAW_GCODE_THUMBNAIL_BACKGROUND_SOLID;
    const bool flags_valid = request.show_bed <= 1 && request.printable_only <= 1 &&
                             request.parts_only <= 1 && request.with_format_tag <= 1;
    const bool reserved_valid = std::all_of(
        std::begin(request.reserved), std::end(request.reserved),
        [](uint32_t value) { return value == 0; });
    if (request.struct_size < sizeof(raw_gcode_thumbnail_render_request) ||
        request.width == 0 || request.height == 0 || request.width > 2048 ||
        request.height > 2048 || !modes_valid || !flags_valid || !reserved_valid)
        throw std::invalid_argument("The G-code thumbnail render request is invalid.");

}

class GCodeThumbnailImageView
{
public:
    GCodeThumbnailImageView() = default;
    explicit GCodeThumbnailImageView(const raw_gcode_thumbnail_image &image) : m_image(image) {}

    const uint8_t *pixels() const { return m_image.pixels; }
    uint64_t pixels_size() const { return m_image.pixels_size; }
    uint32_t width() const { return m_image.width; }
    uint32_t height() const { return m_image.height; }
    uint32_t stride_bytes() const { return m_image.stride_bytes; }
    raw_gcode_thumbnail_origin origin() const { return m_image.origin; }

private:
    raw_gcode_thumbnail_image m_image = {};
};

struct GCodeThumbnailRenderResult
{
    raw_gcode_thumbnail_status status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    GCodeThumbnailImageView image;
};

class GCodeThumbnailRendererView
{
public:
    explicit GCodeThumbnailRendererView(const raw_gcode_thumbnail_renderer *renderer = nullptr) :
        m_renderer(renderer)
    {}

    bool valid() const { return m_renderer != nullptr; }
    // Return the borrowed C renderer (null when unavailable) for services that take the C pointer.
    const raw_gcode_thumbnail_renderer *handle() const { return m_renderer; }

    GCodeThumbnailRenderResult render(const raw_gcode_thumbnail_render_request &request) const
    {
        validate_gcode_thumbnail_render_request(request);
        if (m_renderer == nullptr)
            return {RAW_GCODE_THUMBNAIL_STATUS_SKIPPED, {}};
        if (m_renderer->struct_size < sizeof(raw_gcode_thumbnail_renderer) ||
            m_renderer->render == nullptr)
            throw std::invalid_argument("The G-code thumbnail renderer is incomplete.");

        raw_gcode_thumbnail_render_result result = {};
        result.struct_size = sizeof(result);
        result.image.struct_size = sizeof(result.image);
        m_renderer->render(m_renderer->context, &request, &result);
        if (result.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
            return {result.status, {}};
        if (result.status != RAW_GCODE_THUMBNAIL_STATUS_SUCCESS)
            throw std::runtime_error(result.error_message != nullptr ? result.error_message :
                                     "The host could not render the G-code thumbnail.");
        validate_image(result.image, request);
        return {result.status, GCodeThumbnailImageView(result.image)};
    }

private:
    static void validate_image(const raw_gcode_thumbnail_image &image,
                               const raw_gcode_thumbnail_render_request &request)
    {
        const uint64_t minimum_stride = uint64_t(image.width) * 4u;
        const uint64_t required_size = image.height == 0 ? 0 :
            uint64_t(image.stride_bytes) * (image.height - 1u) + minimum_stride;
        if (image.struct_size < sizeof(raw_gcode_thumbnail_image) || image.pixels == nullptr ||
            image.width != request.width || image.height != request.height ||
            image.pixel_format != RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8 ||
            image.origin > RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT ||
            image.stride_bytes < minimum_stride || required_size > image.pixels_size)
            throw std::runtime_error("The host returned an invalid RGBA8 thumbnail image.");
    }

    const raw_gcode_thumbnail_renderer *m_renderer = nullptr;
};

// One compressed image: the bytes of a file in `format`, with the size of the image they hold.
struct GCodeThumbnailEncodedImage
{
    raw_gcode_thumbnail_encoded_format format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET;
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> bytes;
};

class GCodeThumbnailEncoderSession
{
public:
    virtual ~GCodeThumbnailEncoderSession() = default;

    /* Return a complete file block. An empty vector means that the host had no
       renderer and this optional image was skipped. */
    virtual std::vector<uint8_t> encode(const raw_gcode_thumbnail_render_request &request) = 0;

    /* Tell whether encode_image() is implemented. The adapter reads it once, when it builds the
       C instance: a session that returns false publishes no encode_image callback. */
    virtual bool supports_image() const noexcept { return false; }

    /* Return the compressed image of the request, upright and without framing. Empty bytes mean that the host had
       no renderer and this optional image was skipped. Only called when supports_image() is true. */
    virtual GCodeThumbnailEncodedImage encode_image(const raw_gcode_thumbnail_render_request &request)
    {
        (void)request;
        throw std::logic_error("This G-code thumbnail encoder cannot produce a raw image.");
    }
};

namespace detail {

struct GCodeThumbnailEncoderHolder
{
    std::unique_ptr<GCodeThumbnailEncoderSession> session;
    std::vector<uint8_t> bytes;
    // The last compressed image; its bytes back the pointer returned by encode_image until the next call.
    GCodeThumbnailEncodedImage image;
    std::string error;
    std::mutex mutex;
};

inline void destroy_thumbnail_encoder(void *opaque) noexcept
{
    delete static_cast<GCodeThumbnailEncoderHolder *>(opaque);
}

inline void encode_thumbnail(void *opaque,
                             const raw_gcode_thumbnail_render_request *request,
                             raw_gcode_thumbnail_encode_result *result) noexcept
{
    if (result == nullptr || result->struct_size < sizeof(raw_gcode_thumbnail_encode_result))
        return;
    result->status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    result->bytes = nullptr;
    result->bytes_size = 0;
    result->error_message = nullptr;
    if (opaque == nullptr || request == nullptr ||
        request->struct_size < sizeof(raw_gcode_thumbnail_render_request)) {
        result->status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT;
        return;
    }

    GCodeThumbnailEncoderHolder &holder = *static_cast<GCodeThumbnailEncoderHolder *>(opaque);
    std::lock_guard<std::mutex> lock(holder.mutex);
    holder.bytes.clear();
    holder.error.clear();
    try {
        holder.bytes = holder.session->encode(*request);
        result->bytes = holder.bytes.empty() ? nullptr : holder.bytes.data();
        result->bytes_size = holder.bytes.size();
        result->status = holder.bytes.empty() ? RAW_GCODE_THUMBNAIL_STATUS_SKIPPED :
                                                RAW_GCODE_THUMBNAIL_STATUS_SUCCESS;
    } catch (const std::invalid_argument &error) {
        holder.error = error.what();
        result->status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT;
    } catch (const std::exception &error) {
        holder.error = error.what();
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
    } catch (...) {
        holder.error = "The thumbnail encoder failed with an unknown exception.";
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
    }
    if (result->status != RAW_GCODE_THUMBNAIL_STATUS_SUCCESS &&
        result->status != RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
        result->error_message = holder.error.c_str();
}

// C callback of encode_image: same containment of exceptions as encode_thumbnail(). Bytes and diagnostic stay in the holder
// (valid until the next call on the session); a session that returns no byte reports SKIPPED.
inline void encode_thumbnail_image(void *opaque,
                                   const raw_gcode_thumbnail_render_request *request,
                                   raw_gcode_thumbnail_encoded_image_result *result) noexcept
{
    if (result == nullptr || result->struct_size < sizeof(raw_gcode_thumbnail_encoded_image_result))
        return;
    result->status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    result->format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET;
    result->width = 0;
    result->height = 0;
    result->bytes = nullptr;
    result->bytes_size = 0;
    result->error_message = nullptr;
    if (opaque == nullptr || request == nullptr ||
        request->struct_size < sizeof(raw_gcode_thumbnail_render_request)) {
        result->status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT;
        return;
    }

    GCodeThumbnailEncoderHolder &holder = *static_cast<GCodeThumbnailEncoderHolder *>(opaque);
    std::lock_guard<std::mutex> lock(holder.mutex);
    holder.image = GCodeThumbnailEncodedImage();
    holder.error.clear();
    try {
        holder.image = holder.session->encode_image(*request);
        if (holder.image.bytes.empty()) {
            result->status = RAW_GCODE_THUMBNAIL_STATUS_SKIPPED;
        } else {
            result->format = holder.image.format;
            result->width = holder.image.width;
            result->height = holder.image.height;
            result->bytes = holder.image.bytes.data();
            result->bytes_size = holder.image.bytes.size();
            result->status = RAW_GCODE_THUMBNAIL_STATUS_SUCCESS;
        }
    } catch (const std::invalid_argument &error) {
        holder.error = error.what();
        result->status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT;
    } catch (const std::exception &error) {
        holder.error = error.what();
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
    } catch (...) {
        holder.error = "The thumbnail encoder failed with an unknown exception.";
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
    }
    if (result->status != RAW_GCODE_THUMBNAIL_STATUS_SUCCESS &&
        result->status != RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
        result->error_message = holder.error.c_str();
}

// The vtable of a session that implements encode_image() and the one of a session that does not (null callback).
inline const raw_gcode_thumbnail_encoder_vtable &thumbnail_encoder_vtable(bool with_image)
{
    static const raw_gcode_thumbnail_encoder_vtable with_image_vtable = {
        sizeof(raw_gcode_thumbnail_encoder_vtable),
        &destroy_thumbnail_encoder,
        &encode_thumbnail,
        &encode_thumbnail_image
    };
    static const raw_gcode_thumbnail_encoder_vtable text_only_vtable = {
        sizeof(raw_gcode_thumbnail_encoder_vtable),
        &destroy_thumbnail_encoder,
        &encode_thumbnail,
        nullptr
    };
    return with_image ? with_image_vtable : text_only_vtable;
}

} // namespace detail

inline raw_gcode_thumbnail_encoder_instance make_gcode_thumbnail_encoder_instance(
    std::unique_ptr<GCodeThumbnailEncoderSession> session)
{
    if (!session)
        throw std::invalid_argument("A G-code thumbnail encoder session is required.");
    const bool with_image = session->supports_image();
    detail::GCodeThumbnailEncoderHolder *holder = new detail::GCodeThumbnailEncoderHolder();
    holder->session = std::move(session);
    return {sizeof(raw_gcode_thumbnail_encoder_instance), holder,
            &detail::thumbnail_encoder_vtable(with_image)};
}

struct GCodeThumbnailEncodedBlock
{
    raw_gcode_thumbnail_status status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    std::string_view bytes;
};

// A compressed image borrowed from an encoder until its next call; `bytes` is empty when the optional image was skipped.
struct GCodeThumbnailEncodedImageBlock
{
    raw_gcode_thumbnail_status status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    raw_gcode_thumbnail_encoded_format format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET;
    uint32_t width = 0;
    uint32_t height = 0;
    std::string_view bytes;
};

class GCodeThumbnailEncoderView
{
public:
    explicit GCodeThumbnailEncoderView(
        const raw_gcode_thumbnail_encoder_instance *instance = nullptr) : m_instance(instance)
    {
        if (m_instance != nullptr)
            validate();
    }

    bool valid() const { return m_instance != nullptr; }

    // Tell whether the encoder can return compressed images (its vtable carries a non-null encode_image).
    bool supports_images() const
    {
        return m_instance != nullptr && m_instance->vtable->encode_image != nullptr;
    }

    // Ask the encoder for the compressed image of the request. A missing encoder or a skipped image give SKIPPED. An
    // encoder without encode_image throws. A successful answer is checked: known format, non-empty bytes, and the
    // size of the request; anything else throws, whatever the provider.
    GCodeThumbnailEncodedImageBlock encode_image(
        const raw_gcode_thumbnail_render_request &request) const
    {
        validate_gcode_thumbnail_render_request(request);
        if (m_instance == nullptr)
            return {RAW_GCODE_THUMBNAIL_STATUS_SKIPPED, RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET, 0, 0, {}};
        if (!supports_images())
            throw std::runtime_error("The G-code thumbnail encoder does not support raw images.");
        raw_gcode_thumbnail_encoded_image_result result = {};
        result.struct_size = sizeof(result);
        m_instance->vtable->encode_image(m_instance->session, &request, &result);
        if (result.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
            return {result.status, RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_UNSET, 0, 0, {}};
        if (result.status != RAW_GCODE_THUMBNAIL_STATUS_SUCCESS)
            throw std::runtime_error(result.error_message != nullptr ? result.error_message :
                                     "The G-code thumbnail encoder failed.");
        if (result.format != RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_PNG &&
            result.format != RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG &&
            result.format != RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI)
            throw std::runtime_error("The G-code thumbnail encoder returned an unknown image format.");
        if (result.bytes == nullptr || result.bytes_size == 0)
            throw std::runtime_error("The G-code thumbnail encoder returned no image bytes.");
        if (result.width != request.width || result.height != request.height)
            throw std::runtime_error("The G-code thumbnail encoder returned an image of unexpected dimensions.");
        return {result.status, result.format, result.width, result.height,
                std::string_view(reinterpret_cast<const char *>(result.bytes), size_t(result.bytes_size))};
    }

    GCodeThumbnailEncodedBlock encode(
        const raw_gcode_thumbnail_render_request &request) const
    {
        validate_gcode_thumbnail_render_request(request);
        if (m_instance == nullptr)
            return {RAW_GCODE_THUMBNAIL_STATUS_SKIPPED, {}};
        raw_gcode_thumbnail_encode_result result = {};
        result.struct_size = sizeof(result);
        m_instance->vtable->encode(m_instance->session, &request, &result);
        if (result.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
            return {result.status, {}};
        if (result.status != RAW_GCODE_THUMBNAIL_STATUS_SUCCESS)
            throw std::runtime_error(result.error_message != nullptr ? result.error_message :
                                     "The G-code thumbnail encoder failed.");
        if (result.bytes_size != 0 && result.bytes == nullptr)
            throw std::runtime_error("The G-code thumbnail encoder returned missing bytes.");
        return {result.status,
                result.bytes_size == 0 ? std::string_view() :
                std::string_view(reinterpret_cast<const char *>(result.bytes),
                                 size_t(result.bytes_size))};
    }

private:
    void validate() const
    {
        if (m_instance->struct_size < sizeof(raw_gcode_thumbnail_encoder_instance) ||
            m_instance->session == nullptr || m_instance->vtable == nullptr ||
            m_instance->vtable->struct_size < sizeof(raw_gcode_thumbnail_encoder_vtable) ||
            m_instance->vtable->destroy == nullptr || m_instance->vtable->encode == nullptr)
            throw std::invalid_argument("The G-code thumbnail encoder instance is incomplete.");
    }

    const raw_gcode_thumbnail_encoder_instance *m_instance = nullptr;
};

// An owned request belongs to its consumer, not to the host writer. An empty
// provider_id selects the exclusive group; an explicit id never falls back.
struct GCodeThumbnailRequest {
    raw_gcode_thumbnail_render_request render = {};
    std::string provider_id;
    raw_gcode_thumbnail_placement placement = RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE;
    bool required = false;
};

// Own one service session. The orchestrator, Print and renderer must outlive it.
// The provider is the explicit provider_id, or the one that `config` selects: the caller
// passes the configuration copy it exports with, never the Print's own. Encoders read no
// setting; everything they need is in the render request. Calls are serial; copy encoded
// bytes before another call. No copy or move may leave a borrowed view pointing at another
// owner's instance record.
class GCodeThumbnailEncoder {
public:
    // Acquire an owned service session using borrowed host/Print/config/renderer inputs; report missing providers or execution failures by throwing.
    GCodeThumbnailEncoder(const OrchestratorView &, const Print &, const Config &config,
                          const raw_gcode_thumbnail_renderer *, const std::string &provider_id = {});
    ~GCodeThumbnailEncoder();
    GCodeThumbnailEncoder(const GCodeThumbnailEncoder &) = delete;
    GCodeThumbnailEncoder &operator=(const GCodeThumbnailEncoder &) = delete;
    GCodeThumbnailEncoderView view() const { return GCodeThumbnailEncoderView(&m_instance); }
private:
    void close() noexcept;
    raw_gcode_thumbnail_encoder_instance m_instance = {};
};

// Build the standard requests once from config. Provider ids are serialized
// strings, never codec enum indices. BIQU always precedes all file content.
std::vector<GCodeThumbnailRequest> default_gcode_thumbnail_requests(const Config &config);

// Acquire service instances through the public orchestrator and copy output
// into three owned blocks indexed by placement. Failure aborts the caller;
// optional absent renderers skip images, required images fail explicitly.
// Encoders are selected with `config`, the caller's configuration copy.
std::array<std::string, 3> encode_gcode_thumbnails(
    const OrchestratorView &, const Print &, const Config &config,
    const raw_gcode_thumbnail_renderer *, const std::vector<GCodeThumbnailRequest> &requests);

// Render requested thumbnails as compressed images and return owned bytes; unsupported providers and required skipped requests throw.
std::vector<GCodeThumbnailEncodedImage> encode_gcode_thumbnail_images(
    const OrchestratorView &, const Print &, const Config &config,
    const raw_gcode_thumbnail_renderer *, const std::vector<GCodeThumbnailRequest> &requests);

// Return an owned encoder without rendering and set incompatibility to empty for raw-image support or its diagnostic otherwise; borrowed host, Print, config and renderer inputs must outlive the session, and acquisition failures throw.
std::unique_ptr<GCodeThumbnailEncoder> acquire_gcode_thumbnail_image_encoder(
    const OrchestratorView &, const Print &, const Config &config,
    const raw_gcode_thumbnail_renderer *, const std::string &provider_id, std::string &incompatibility);

// Header-only definitions keep external consumers on the public C ABI.
namespace detail {
// Decode an RGB setting, preserving the supplied default when malformed.
inline raw_gcode_thumbnail_color parse_thumbnail_color(const std::string &, raw_gcode_thumbnail_color);
inline raw_gcode_thumbnail_color parse_thumbnail_color(const std::string &text,
                                                 raw_gcode_thumbnail_color fallback)
{
    if (text.size() != 7 || text.front() != '#')
        return fallback;
    raw_gcode_thumbnail_color result = fallback;
    const char *first = text.data() + 1;
    const char *last = text.data() + text.size();
    uint32_t value = 0;
    const std::from_chars_result parsed = std::from_chars(first, last, value, 16);
    if (parsed.ec != std::errc() || parsed.ptr != last)
        return fallback;
    result.red = uint8_t((value >> 16) & 0xffu);
    result.green = uint8_t((value >> 8) & 0xffu);
    result.blue = uint8_t(value & 0xffu);
    result.alpha = 255;
    return result;
}

} // namespace detail
inline std::vector<GCodeThumbnailRequest> default_gcode_thumbnail_requests(const Config &config)
{
    std::vector<GCodeThumbnailRequest> requests;
    const ConfigOption selected = config.get("thumbnails_format");
    const std::string provider = selected.valid() ? selected.serialize() : "PNG";
    const raw_gcode_thumbnail_placement placement = provider == "BIQU" ? RAW_GCODE_THUMBNAIL_FILE_PREFIX :
        (config.bool_or_default("thumbnails_end_file", false) ? RAW_GCODE_THUMBNAIL_FILE_SUFFIX :
                                                             RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE);

    if (!config.has("thumbnails"))
        return requests;
    const ConfigOption sizes = config.get("thumbnails");
    for (uint32_t idx = 0; idx < sizes.size(); ++idx) {
        const ConfigPoint size = sizes.get_point(idx);
        if (size.x <= 0.0 || size.y <= 0.0)
            continue;
        if (!std::isfinite(size.x) || !std::isfinite(size.y) ||
            std::floor(size.x) != size.x || std::floor(size.y) != size.y ||
            size.x > double(UINT32_MAX) || size.y > double(UINT32_MAX))
            throw std::invalid_argument("Thumbnail dimensions must be finite positive integers.");

        raw_gcode_thumbnail_render_request request = {};
        request.struct_size = sizeof(request);
        request.width = uint32_t(size.x);
        request.height = uint32_t(size.y);
        // Reserved camera fields describe the only view implemented by the host.
        request.camera = RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC;
        request.projection = RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC;
        request.framing = RAW_GCODE_THUMBNAIL_FRAMING_AUTO;
        request.color_mode = raw_gcode_thumbnail_color_mode(
            config.enum_or_default("thumbnail_color_mode", RAW_GCODE_THUMBNAIL_COLOR_BY_FILAMENT));
        request.background = raw_gcode_thumbnail_background(
            config.enum_or_default("thumbnail_background", RAW_GCODE_THUMBNAIL_BACKGROUND_TRANSPARENT));
        request.show_bed = config.bool_or_default("thumbnails_with_bed", true) ? 1 : 0;
        request.printable_only = 1;
        request.parts_only = config.bool_or_default("thumbnails_with_support", false) ? 0 : 1;
        request.with_format_tag = config.bool_or_default("thumbnails_tag_format", false) ? 1 : 0;
        request.uniform_color = detail::parse_thumbnail_color(
            config.string_or_default("thumbnails_color", "#018aff"), {1, 138, 255, 255});
        request.background_color = detail::parse_thumbnail_color(
            config.string_or_default("thumbnail_background_color", "#ffffff"), {255, 255, 255, 255});
        request.camera_position = {1.0, -1.0, 1.0};
        request.camera_target = {0.0, 0.0, 0.0};
        request.camera_up = {0.0, 0.0, 1.0};
        request.custom_scale = 1.0;
        requests.push_back({request, {}, placement, false});
    }
    return requests;
}


// Select a provider from the borrowed config or explicit ID and own its session until destruction; borrowed host, Print, config and renderer must outlive the session, and missing providers or failed execution throw without retaining a partial session.
inline GCodeThumbnailEncoder::GCodeThumbnailEncoder(const OrchestratorView &orchestrator,
    const Print &print, const Config &config, const raw_gcode_thumbnail_renderer *renderer,
    const std::string &provider_id)
{
    std::string requested_id = provider_id;
    PluginView plugin;
    if (provider_id.empty()) {
        if (config.has("thumbnails_format"))
            requested_id = config.get("thumbnails_format").serialize();
        plugin = orchestrator.select_plugin(config.handle(), GCODE_THUMBNAIL, "thumbnail_encoder_plugin");
    } else {
        plugin = orchestrator.find_plugin(provider_id);
    }
    if (provider_id.empty() && !plugin.valid() && !requested_id.empty())
        throw std::invalid_argument("GCODE_THUMBNAIL provider '" + requested_id + "' is not active");
    if (!plugin.valid() || plugin.step() != GCODE_THUMBNAIL)
        throw std::invalid_argument(requested_id.empty() ? "No active G-code thumbnail encoder." :
                                    "Unknown GCODE_THUMBNAIL provider: " + requested_id);
    run_ctx_gcode_thumbnail payload = {};
    payload.print = print.handle();
    payload.renderer = renderer;
    try {
        const raw_plugin_execution_status status = orchestrator.execute(plugin, print.handle(), payload);
        m_instance = payload.instance;
        if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
            throw std::runtime_error("Thumbnail provider '" + plugin.id() + "' failed: " + std::to_string(int(status)));
        (void)view();
    } catch (...) { m_instance = payload.instance; close(); throw; }
}
inline GCodeThumbnailEncoder::~GCodeThumbnailEncoder() { close(); }
inline void GCodeThumbnailEncoder::close() noexcept
{
    if (m_instance.session && m_instance.vtable &&
        m_instance.vtable->struct_size >= offsetof(raw_gcode_thumbnail_encoder_vtable, destroy) + sizeof(gcode_thumbnail_encoder_destroy_fn) &&
        m_instance.vtable->destroy)
        m_instance.vtable->destroy(m_instance.session);
    m_instance = {};
}
inline std::array<std::string, 3> encode_gcode_thumbnails(const OrchestratorView &orchestrator,
    const Print &print, const Config &config, const raw_gcode_thumbnail_renderer *renderer,
    const std::vector<GCodeThumbnailRequest> &requests)
{
    std::array<std::string, 3> blocks;
    std::map<std::string, std::unique_ptr<GCodeThumbnailEncoder>> encoders;
    for (const GCodeThumbnailRequest &request : requests) {
        validate_gcode_thumbnail_render_request(request.render);
        if (int(request.placement) < 0 || int(request.placement) > 2)
            throw std::invalid_argument("Invalid thumbnail placement.");
        std::unique_ptr<GCodeThumbnailEncoder> &encoder = encoders[request.provider_id];
        if (!encoder) encoder.reset(new GCodeThumbnailEncoder(orchestrator, print, config, renderer, request.provider_id));
        const GCodeThumbnailEncodedBlock block = encoder->view().encode(request.render);
        if (block.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED) {
            if (request.required) throw std::runtime_error("A required thumbnail has no renderer.");
        } else if (!block.bytes.empty()) {
            blocks[size_t(request.placement)].append(block.bytes.data(), block.bytes.size());
        }
    }
    return blocks;
}

// Return an owned encoder after inspecting its capability without rendering; set incompatibility to empty for raw-image support or its diagnostic otherwise, and borrow host, Print, config and renderer for the session lifetime; acquisition failures throw.
inline std::unique_ptr<GCodeThumbnailEncoder> acquire_gcode_thumbnail_image_encoder(
    const OrchestratorView &orchestrator, const Print &print, const Config &config,
    const raw_gcode_thumbnail_renderer *renderer, const std::string &provider_id, std::string &incompatibility)
{
    std::unique_ptr<GCodeThumbnailEncoder> encoder(new GCodeThumbnailEncoder(orchestrator, print, config, renderer, provider_id));
    incompatibility.clear();
    if (!encoder->view().supports_images()) {
        const std::string provider = provider_id.empty() ?
            (config.has("thumbnails_format") ? config.get("thumbnails_format").serialize() : "PNG") : provider_id;
        incompatibility = "Thumbnail provider '" + provider + "' does not support raw images.";
    }
    return encoder;
}

// Validate and acquire each requested provider before rendering any image, then copy successful borrowed bytes into owned results; unsupported providers and required skipped requests throw.
inline std::vector<GCodeThumbnailEncodedImage> encode_gcode_thumbnail_images(
    const OrchestratorView &orchestrator, const Print &print, const Config &config,
    const raw_gcode_thumbnail_renderer *renderer,
    const std::vector<GCodeThumbnailRequest> &requests)
{
    std::vector<GCodeThumbnailEncodedImage> images;
    if (requests.empty())
        return images;

    std::map<std::string, std::unique_ptr<GCodeThumbnailEncoder>> encoders;
    for (const GCodeThumbnailRequest &request : requests) {
        validate_gcode_thumbnail_render_request(request.render);
        std::map<std::string, std::unique_ptr<GCodeThumbnailEncoder>>::iterator encoder =
            encoders.find(request.provider_id);
        if (encoder == encoders.end()) {
            std::string incompatibility;
            std::unique_ptr<GCodeThumbnailEncoder> acquired = acquire_gcode_thumbnail_image_encoder(
                orchestrator, print, config, renderer, request.provider_id, incompatibility);
            if (!incompatibility.empty())
                throw std::runtime_error(incompatibility);
            encoder = encoders.emplace(request.provider_id, std::move(acquired)).first;
        }
    }

    for (const GCodeThumbnailRequest &request : requests) {
        const GCodeThumbnailEncodedImageBlock block = encoders.find(request.provider_id)->second->view().encode_image(request.render);
        if (block.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED) {
            if (request.required)
                throw std::runtime_error("A required raw G-code thumbnail was skipped.");
            continue;
        }

        GCodeThumbnailEncodedImage image;
        image.format = block.format;
        image.width = block.width;
        image.height = block.height;
        image.bytes.assign(reinterpret_cast<const uint8_t *>(block.bytes.data()),
                           reinterpret_cast<const uint8_t *>(block.bytes.data()) + block.bytes.size());
        images.push_back(std::move(image));
    }
    return images;
}

} // namespace slic3r_api
#endif
