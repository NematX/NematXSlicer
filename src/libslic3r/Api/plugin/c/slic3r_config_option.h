///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_config_option_h_
#define slic3r_config_option_h_

#define SLIC3R_PLUGIN_API_CONFIG_OPTION_MAJOR 3u
#define SLIC3R_PLUGIN_API_CONFIG_OPTION_MINOR 0u

#include <stddef.h>
#include <stdint.h>

#include "slic3r_config_option_type.h"
#include "slic3r_def.h"
#include "slic3r_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct c_float_or_percent
{
    double  value;
    uint8_t percent;
} c_float_or_percent;

SLIC3R_HOST_API double c_float_or_percent_get_effective_value(const c_float_or_percent *me, double ratio_over);
SLIC3R_HOST_API double c_float_or_percent_get_float(const c_float_or_percent *me);
SLIC3R_HOST_API int32_t c_float_or_percent_is_percent(const c_float_or_percent *me);
SLIC3R_HOST_API int32_t c_float_or_percent_equal(const c_float_or_percent *lhs, const c_float_or_percent *rhs);
SLIC3R_HOST_API int32_t c_float_or_percent_less(const c_float_or_percent *lhs, const c_float_or_percent *rhs);

typedef struct graph_data_handle graph_data_handle;

/*
Interpolation mode used between consecutive graph points. NONE is returned
when a graph handle is null or contains an unsupported native mode.
*/
typedef enum raw_graph_type {
    RAW_GRAPH_TYPE_NONE = 0,
    RAW_GRAPH_TYPE_SQUARE,
    RAW_GRAPH_TYPE_LINEAR,
    RAW_GRAPH_TYPE_SPLINE,
} raw_graph_type;

/* Return the number of usable points in this borrowed graph; null returns 0. */
SLIC3R_HOST_API uint32_t         graph_data_size(const graph_data_handle *me);

/*
Copy one usable graph point into caller-owned storage. Graph coordinates are
unscaled doubles. Return 1 on success. A null graph, null output, invalid graph
or out-of-range index returns 0 and resets a non-null output to (0, 0).
*/
SLIC3R_HOST_API int32_t graph_data_get_point(const graph_data_handle *me, uint32_t index, c_vec2d *out);

/* Return the graph interpolation mode, or RAW_GRAPH_TYPE_NONE on failure. */
SLIC3R_HOST_API raw_graph_type graph_data_type(const graph_data_handle *me);

/* Evaluate the graph at unscaled X; a null graph returns 0.0. */
SLIC3R_HOST_API double         graph_interpolate(const graph_data_handle *me, double x);

/* Evaluate the host's inverse interpolation at unscaled Y; null returns 0.0. */
SLIC3R_HOST_API double         graph_inverse_interpolate(const graph_data_handle *me, double y);

/* Return nonzero when the graph's usable point range and mode are valid. */
SLIC3R_HOST_API int32_t        graph_validate(const graph_data_handle *me);

enum config_option_flags
{
    FCO_PHONY = 1,
    FCO_EXTRUDER_ARRAY = 1 << 1,
    FCO_PLACEHOLDER_TEMP = 1 << 2,
    FCO_ENABLED = 1 << 3,
    FCO_CAN_DISABLED = 1 << 4,
};

typedef struct config_option_handle config_option_handle;
typedef struct config_option_vector_handle config_option_vector_handle;
typedef struct config_handle config_handle;

/*
Resolve one numeric option through the host ConfigBase ratio chain.

extruder_id selects vector values used directly or through ratio_over.
EXTRUDER_ID_UNKNOWN cannot select a vector item and therefore makes vector
resolution fail. The function returns zero when the id is invalid, the key is
absent or disabled, the vector has no usable item, or the value cannot be
represented or resolved. No C++ exception crosses the plugin ABI.
*/
SLIC3R_HOST_API int32_t config_get_computed_value(const config_handle *me,
                                                  const char *key,
                                                  extruder_index_t extruder_id,
                                                  double *value_out);

SLIC3R_HOST_API config_option_type config_option_type_get(const config_option_handle *me);
SLIC3R_HOST_API uint32_t           config_option_flags_get(const config_option_handle *me);
SLIC3R_HOST_API uint32_t           config_option_size(const config_option_handle *me);
SLIC3R_HOST_API int32_t            config_option_is_scalar(const config_option_handle *me);
SLIC3R_HOST_API int32_t            config_option_is_vector(const config_option_handle *me);
SLIC3R_HOST_API int32_t            config_option_is_enabled(const config_option_handle *me, int32_t idx);
SLIC3R_HOST_API int32_t            config_option_set_enabled(config_option_handle *me, int32_t enabled, int32_t idx);
SLIC3R_HOST_API int32_t            config_option_can_be_disabled(const config_option_handle *me);
SLIC3R_HOST_API int32_t            config_option_set_can_be_disabled(config_option_handle *me, int32_t force_disabled);
SLIC3R_HOST_API int32_t            config_option_is_phony(const config_option_handle *me);
SLIC3R_HOST_API int32_t            config_option_set_phony(config_option_handle *me, int32_t phony);
SLIC3R_HOST_API int32_t            config_option_equals(const config_option_handle *me, const config_option_handle *other);
SLIC3R_HOST_API int32_t            config_option_less(const config_option_handle *me, const config_option_handle *other);
SLIC3R_HOST_API void               config_option_copy(config_option_handle *dst, const config_option_handle *src);

/* Serialize a borrowed option and return the full byte length excluding the terminator.
Pass NULL/0 to query the length. A writable buffer receives at most max_size-1
bytes and a terminator. Empty values and NULL handles return 0. A serialization
failure (for example a non-finite floating-point value) returns UINT32_MAX and
clears a writable buffer; no exception escapes. */
SLIC3R_HOST_API uint32_t config_option_serialize(const config_option_handle *me, char *out, uint32_t max_size);
SLIC3R_HOST_API int32_t config_option_deserialize(config_option_handle *me, const char *str, int32_t append);

SLIC3R_HOST_API int32_t            config_option_get_int(const config_option_handle *me, uint32_t idx);
SLIC3R_HOST_API double             config_option_get_float(const config_option_handle *me, uint32_t idx);
SLIC3R_HOST_API c_float_or_percent config_option_get_float_or_percent(const config_option_handle *me, uint32_t idx);
SLIC3R_HOST_API int32_t            config_option_get_bool(const config_option_handle *me, uint32_t idx);
SLIC3R_HOST_API uint32_t           config_option_get_string(const config_option_handle *me, uint32_t idx, char *out, uint32_t max_size);
/*
Borrow one GraphData payload from a graph option.

For scalar graph options, idx is ignored. For vector graph options, idx selects
the vector item. The returned pointer is owned by the config option and remains
valid only while that option is not modified.
*/
SLIC3R_HOST_API const graph_data_handle *config_option_get_graph(const config_option_handle *me, uint32_t idx);
SLIC3R_HOST_API void               config_option_set_int(config_option_handle *me, int32_t value, uint32_t idx);
SLIC3R_HOST_API void               config_option_set_float(config_option_handle *me, double value, uint32_t idx);
SLIC3R_HOST_API void               config_option_set_float_or_percent(config_option_handle *me, c_float_or_percent value, uint32_t idx);
SLIC3R_HOST_API void               config_option_set_bool(config_option_handle *me, int32_t value, uint32_t idx);
SLIC3R_HOST_API void               config_option_set_string(config_option_handle *me, const char *value, uint32_t idx);

SLIC3R_HOST_API config_option_vector_handle *config_option_vector_cast_mutable(config_option_handle *me);
SLIC3R_HOST_API const config_option_vector_handle *config_option_vector_cast(const config_option_handle *me);
SLIC3R_HOST_API uint32_t config_option_vector_size(const config_option_vector_handle *me);
SLIC3R_HOST_API int32_t config_option_vector_empty(const config_option_vector_handle *me);
/*
Serialize one existing vector item. The return value follows the same two-call
buffer contract as config_option_serialize(). Invalid handles or indices return
zero; a serialization failure returns UINT32_MAX.
*/
SLIC3R_HOST_API uint32_t config_option_vector_serialize_at(
    const config_option_vector_handle *me, int32_t idx, char *out, uint32_t max_size);

/*
Replace one existing vector item from its serialized representation. Parsing is
performed before mutation, so invalid text leaves the option unchanged. Return
1 on success; null arguments, invalid indices or text return 0.
*/
SLIC3R_HOST_API int32_t config_option_vector_deserialize_at(
    config_option_vector_handle *me, int32_t idx, const char *str);
SLIC3R_HOST_API void   config_option_vector_resize(
    config_option_vector_handle *me, uint32_t new_size, const config_option_handle *default_value);
SLIC3R_HOST_API void   config_option_vector_clear(config_option_vector_handle *me);
SLIC3R_HOST_API int32_t config_option_vector_is_extruder_size(const config_option_vector_handle *me);
SLIC3R_HOST_API void    config_option_vector_set_is_extruder_size(config_option_vector_handle *me, int32_t is_extruder_size);

#ifdef __cplusplus
}
#endif

#endif // slic3r_config_option_h_
