/* Compile the public fan/timing records as C without any host or other SDK
   header. Export their layout to the C++ tests to catch language ABI drift. */
#include "libslic3r/Api/plugin/c/slic3r_fan_policy.h"
#include "libslic3r/Api/plugin/c/slic3r_printing_layer_time.h"
#include <stddef.h>

size_t fan_policy_c_layout(unsigned int field)
{
    switch (field) {
    case 0: return sizeof(c_fan_policy_context);
    case 1: return sizeof(c_fan_role_reference);
    case 2: return offsetof(c_fan_policy_context, roles);
    case 3: return sizeof(c_printing_layer_time_property);
    case 4: return offsetof(c_printing_layer_time_property, origin);
    default: return RAW_FAN_ROLE_COUNT;
    }
}
