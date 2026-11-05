///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BuiltinGCodeThumbnails.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/beast/core/detail/base64.hpp>
#include <boost/format.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_thumbnail.h"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"

/*
Built-in G-code thumbnail encoders
==================================

Four provider subclasses share settings through their base and choose the codec passed
to the existing GCodeThumbnails::compress_thumbnail() function. A session asks
the host for one RGBA8 image, normalizes strided/top-origin input to the legacy
ThumbnailData layout, compresses it, and creates the complete textual block.
The writer receives opaque bytes and therefore contains no format branches.
The session also returns the compressed image alone (encode_image(): bytes, format, size) for PNG, JPG and
QOI, which share the render, orientation and compression steps with the framed text block. BIQU is text only and
publishes no image callback.

Representative direct calls (branches are not a timeline):

    BuiltinThumbnailPluginBase::run_impl()
        |-- plugin_ctx_as_gcode_thumbnail()
        `-- make_gcode_thumbnail_encoder_instance()

    BuiltinThumbnailEncoderSession::encode()
        |-- validate_gcode_thumbnail_render_request()
        |-- m_renderer.render()
        |-- frame_image()
        `-- copy_image()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Thumbnail {
namespace {

const char *const k_no_dependencies[] = {nullptr};

const key_value_string_pair_t k_background_values[] = {
    {"transparent", "Transparent"}, {"solid", "Solid color"}
};

// Every built-in provider declares the same settings. The two background
// definitions belong to the exclusive group, so any active provider can publish
// them; no provider depends on PNG being active.
const raw_used_config_key k_shared_setting_keys[] = {
    {"thumbnail_background", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_color_mode", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_end_file", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_bed", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_support", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_tag_format", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
const char *const k_defined_setting_keys[] = {"thumbnail_background", "thumbnail_background_color"};

// Publish identical shared background definitions and stable UI fragments.
void register_shared_settings(orchestrator_handle *orchestrator);
// Register a closed enum in the printer preset through the public C contract.
void register_enum_setting(orchestrator_handle *, const char *, const char *, const char *,
                           const char *, const key_value_string_pair_t *, uint32_t);

void register_enum_setting(orchestrator_handle *orchestrator,
                           const char *key,
                           const char *label,
                           const char *tooltip,
                           const char *default_value,
                           const key_value_string_pair_t *values,
                           uint32_t value_count)
{
    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = key;
    definition.type = RAW_CO_ENUM;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    definition.printer_technology = RAW_PT_FFF;
    definition.category = RAW_OPTION_CATEGORY_FIRMWARE;
    definition.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
    definition.invalidates_step = STEP_NONE;
    definition.label = label;
    definition.tooltip = tooltip;
    definition.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    definition.default_serialized_value = default_value;
    definition.enum_def.value_label_pairs = {values, value_count};
    if (orchestrator_create_option_def(orchestrator, &definition) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error(std::string("Could not register thumbnail setting '") + key + "'.");
}

void register_shared_settings(orchestrator_handle *orchestrator)
{
    register_enum_setting(orchestrator, "thumbnail_background", "Thumbnail background",
        "Selects a transparent or solid thumbnail background.", "transparent",
        k_background_values, uint32_t(std::size(k_background_values)));

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "thumbnail_background_color";
    definition.type = RAW_CO_STRING;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    definition.printer_technology = RAW_PT_FFF;
    definition.category = RAW_OPTION_CATEGORY_FIRMWARE;
    definition.gui_type = RAW_GUI_TYPE_COLOR;
    definition.invalidates_step = STEP_NONE;
    definition.label = "Thumbnail background color";
    definition.tooltip = "Color used when the thumbnail background is solid.";
    definition.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    definition.default_serialized_value = "#ffffff";
    if (orchestrator_create_option_def(orchestrator, &definition) != OPTION_DEF_ERROR_OK)
        throw std::runtime_error("Could not register thumbnail background color.");

    // Camera/projection/framing controls are intentionally absent until the
    // host renderer implements them. The C request fields remain reserved.
    orchestrator_add_ui_fragment(orchestrator, "printer_fff.ui", "thumbnail_encoder_plugin",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:insert$beforesetting$thumbnails:thumbnails_format\n", 1);
    orchestrator_add_ui_fragment(
        orchestrator, "printer_fff.ui", "thumbnail_render_settings",
        "page:General:printer\n"
        "group:Firmware\n"
        "\tsetting:thumbnail_background\n"
        "\tsetting:thumbnail_background_color\n",
        1);
}

class BuiltinThumbnailEncoderSession final : public GCodeThumbnailEncoderSession
{
public:
    BuiltinThumbnailEncoderSession(GCodeThumbnailRendererView renderer,
                                   Slic3r::GCodeThumbnailsFormat format) :
        m_renderer(renderer), m_format(format)
    {}

    std::vector<uint8_t> encode(const raw_gcode_thumbnail_render_request &request) override;

    // Every codec except BIQU (text lines, not an image file) can return its compressed image.
    bool supports_image() const noexcept override { return m_format != Slic3r::GCodeThumbnailsFormat::BIQU; }
    GCodeThumbnailEncodedImage encode_image(const raw_gcode_thumbnail_render_request &request) override;

private:
    Slic3r::ThumbnailData copy_image(const GCodeThumbnailImageView &image) const;
    // Compress with the codec of this session; throw when the codec returns nothing.
    std::unique_ptr<Slic3r::GCodeThumbnails::CompressedImageBuffer> compress_image(
        const Slic3r::ThumbnailData &image) const;
    std::vector<uint8_t> frame_image(const Slic3r::ThumbnailData &image,
                                     bool with_format_tag) const;

    GCodeThumbnailRendererView m_renderer;
    Slic3r::GCodeThumbnailsFormat m_format;
};

Slic3r::ThumbnailData BuiltinThumbnailEncoderSession::copy_image(
    const GCodeThumbnailImageView &image) const
{
    Slic3r::ThumbnailData copy;
    copy.set(image.width(), image.height());
    const size_t row_size = size_t(image.width()) * 4u;
    for (uint32_t output_row = 0; output_row < image.height(); ++output_row) {
        const uint32_t source_row = image.origin() == RAW_GCODE_THUMBNAIL_ORIGIN_BOTTOM_LEFT ?
            output_row : image.height() - output_row - 1;
        std::memcpy(copy.pixels.data() + size_t(output_row) * row_size,
                    image.pixels() + size_t(source_row) * image.stride_bytes(), row_size);
    }
    return copy;
}

std::unique_ptr<Slic3r::GCodeThumbnails::CompressedImageBuffer> BuiltinThumbnailEncoderSession::compress_image(
    const Slic3r::ThumbnailData &image) const
{
    std::unique_ptr<Slic3r::GCodeThumbnails::CompressedImageBuffer> compressed =
        Slic3r::GCodeThumbnails::compress_thumbnail(image, m_format);
    if (!compressed || compressed->data == nullptr || compressed->size == 0)
        throw std::runtime_error("The thumbnail codec returned no data.");
    return compressed;
}

std::vector<uint8_t> BuiltinThumbnailEncoderSession::frame_image(
    const Slic3r::ThumbnailData &image,
    bool with_format_tag) const
{
    static constexpr size_t MaximumBase64RowLength = 78;
    std::unique_ptr<Slic3r::GCodeThumbnails::CompressedImageBuffer> compressed = compress_image(image);

    std::string output;
    const std::string tag = with_format_tag && !compressed->tag().empty() ?
        std::string(compressed->tag()) : std::string(Slic3r::GCodeThumbnails::EMPTY_TAG);
    if (m_format == Slic3r::GCodeThumbnailsFormat::BIQU) {
        std::ostringstream header;
        header << ';' << std::setfill('0') << std::hex << std::setw(4) << image.width
               << std::setw(4) << image.height << '\n';
        output += header.str();
        const char *data = static_cast<const char *>(compressed->data);
        const size_t text_size = data[compressed->size - 1] == '\0' ?
            compressed->size - 1 : compressed->size;
        output.append(data, text_size);
        output += "; bigtree thumbnail end\n";
    } else {
        std::string encoded;
        encoded.resize(boost::beast::detail::base64::encoded_size(compressed->size));
        encoded.resize(boost::beast::detail::base64::encode(
            encoded.data(), compressed->data, compressed->size));
        output += (boost::format("\n;\n; %s begin %dx%d %d\n") % tag %
                   image.width % image.height % encoded.size()).str();
        while (!encoded.empty()) {
            const size_t count = std::min(MaximumBase64RowLength, encoded.size());
            output += "; " + encoded.substr(0, count) + "\n";
            encoded.erase(0, count);
        }
    }
    output += (boost::format("; %s end\n;\n") % tag).str();
    return std::vector<uint8_t>(output.begin(), output.end());
}

std::vector<uint8_t> BuiltinThumbnailEncoderSession::encode(
    const raw_gcode_thumbnail_render_request &request)
{
    validate_gcode_thumbnail_render_request(request);

    const GCodeThumbnailRenderResult rendered = m_renderer.render(request);
    if (rendered.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
        return {};
    return frame_image(copy_image(rendered.image), request.with_format_tag != 0);
}

// Render the image, compress it with the session's codec and return the file bytes without any text framing; an
// optional image without renderer gives an empty result. The format is the codec's (never BIQU: see supports_image()).
GCodeThumbnailEncodedImage BuiltinThumbnailEncoderSession::encode_image(
    const raw_gcode_thumbnail_render_request &request)
{
    validate_gcode_thumbnail_render_request(request);

    const GCodeThumbnailRenderResult rendered = m_renderer.render(request);
    if (rendered.status == RAW_GCODE_THUMBNAIL_STATUS_SKIPPED)
        return {};
    const Slic3r::ThumbnailData image = copy_image(rendered.image);
    const std::unique_ptr<Slic3r::GCodeThumbnails::CompressedImageBuffer> compressed = compress_image(image);

    GCodeThumbnailEncodedImage result;
    switch (m_format) {
    case Slic3r::GCodeThumbnailsFormat::JPG: result.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG; break;
    case Slic3r::GCodeThumbnailsFormat::QOI: result.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI; break;
    default: result.format = RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_PNG; break;
    }
    result.width = image.width;
    result.height = image.height;
    const uint8_t *data = static_cast<const uint8_t *>(compressed->data);
    result.bytes.assign(data, data + compressed->size);
    return result;
}

// Common provider behavior: settings, group membership and session ownership.
// Subclasses select a codec; the session continues to share image processing.
class BuiltinThumbnailPluginBase : public PluginBase
{
public:
    explicit BuiltinThumbnailPluginBase(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
protected:
    virtual Slic3r::GCodeThumbnailsFormat format() const noexcept = 0;
private:
    const char *exclusive_group_impl() const noexcept override { return "thumbnail_encoder_plugin"; }
    slicing_step_t step_impl() const noexcept override { return GCODE_THUMBNAIL; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_shared_setting_keys), std::end(k_shared_setting_keys), keys);
        return int32_t(std::size(k_shared_setting_keys));
    }
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_setting_keys), std::end(k_defined_setting_keys), keys);
        return int32_t(std::size(k_defined_setting_keys));
    }
    void inilialize_impl(storage_handle *) const override { register_shared_settings(m_orchestrator); }
    // Publish a fresh session which borrows the caller's renderer.
    void run_impl(const plugin_run_context *) const override;
};

void BuiltinThumbnailPluginBase::run_impl(const plugin_run_context *run_context) const
{
    // The published encoder session intentionally outlives this callback and
    // therefore cannot borrow its run context. Stop before creating that
    // session; the writer performs the remaining checks around each encode.
    throw_if_cancelled(run_context);
    run_ctx_gcode_thumbnail *context = plugin_ctx_as_gcode_thumbnail(run_context);
    if (context == nullptr || context->print == nullptr)
        throw std::invalid_argument("Thumbnail encoder received an invalid run context.");
    std::unique_ptr<GCodeThumbnailEncoderSession> session(
        new BuiltinThumbnailEncoderSession(GCodeThumbnailRendererView(context->renderer),
                                           format()));
    throw_if_cancelled(run_context);
    context->instance = make_gcode_thumbnail_encoder_instance(std::move(session));
}

// PNG provider identity; shared settings and session publication come from the base.
class PNGThumbnailPlugin final : public BuiltinThumbnailPluginBase
{
public:
    using BuiltinThumbnailPluginBase::BuiltinThumbnailPluginBase;
private:
    const char *id_impl() const noexcept override { return "PNG"; }
    const char *name_impl() const noexcept override { return "PNG"; }
    const char *description_impl() const noexcept override { return "Encodes standard PNG G-code thumbnail blocks."; }
    // Return the common encoder execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return PNG's preference number. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    Slic3r::GCodeThumbnailsFormat format() const noexcept override { return Slic3r::GCodeThumbnailsFormat::PNG; }
};

// JPEG provider identity; shared settings and session publication come from the base.
class JPEGThumbnailPlugin final : public BuiltinThumbnailPluginBase
{
public:
    using BuiltinThumbnailPluginBase::BuiltinThumbnailPluginBase;
private:
    const char *id_impl() const noexcept override { return "JPG"; }
    const char *name_impl() const noexcept override { return "JPEG"; }
    const char *description_impl() const noexcept override { return "Encodes compact JPEG G-code thumbnail blocks."; }
    // Return the common encoder execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return JPEG's preference number. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    Slic3r::GCodeThumbnailsFormat format() const noexcept override { return Slic3r::GCodeThumbnailsFormat::JPG; }
};

// QOI provider identity; shared settings and session publication come from the base.
class QOIThumbnailPlugin final : public BuiltinThumbnailPluginBase
{
public:
    using BuiltinThumbnailPluginBase::BuiltinThumbnailPluginBase;
private:
    const char *id_impl() const noexcept override { return "QOI"; }
    const char *name_impl() const noexcept override { return "QOI"; }
    const char *description_impl() const noexcept override { return "Encodes low-memory QOI G-code thumbnail blocks."; }
    // Return the common encoder execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return QOI's preference number. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 20; }
    Slic3r::GCodeThumbnailsFormat format() const noexcept override { return Slic3r::GCodeThumbnailsFormat::QOI; }
};

// BIQU provider identity; shared settings and session publication come from the base.
class BIQUThumbnailPlugin final : public BuiltinThumbnailPluginBase
{
public:
    using BuiltinThumbnailPluginBase::BuiltinThumbnailPluginBase;
private:
    const char *id_impl() const noexcept override { return "BIQU"; }
    const char *name_impl() const noexcept override { return "BIQU"; }
    const char *description_impl() const noexcept override { return "Encodes BIQU RGB565 hexadecimal G-code thumbnail blocks."; }
    // Return the common encoder execution priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 0; }
    // Return BIQU's preference number. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 30; }
    Slic3r::GCodeThumbnailsFormat format() const noexcept override { return Slic3r::GCodeThumbnailsFormat::BIQU; }
};

} // namespace

void register_builtin_gcode_thumbnail_plugins(orchestrator_handle *orchestrator)
{
    static std::vector<std::unique_ptr<BuiltinThumbnailPluginBase>> plugins;
    if (plugins.empty()) {
        plugins.emplace_back(new PNGThumbnailPlugin(orchestrator));
        plugins.emplace_back(new JPEGThumbnailPlugin(orchestrator));
        plugins.emplace_back(new QOIThumbnailPlugin(orchestrator));
        plugins.emplace_back(new BIQUThumbnailPlugin(orchestrator));
    }
    for (const std::unique_ptr<BuiltinThumbnailPluginBase> &plugin : plugins)
        orchestrator_register_plugin(orchestrator, plugin->c_instance());
}

}}} // namespace slic3r_api::GCodeGeneration::Thumbnail
