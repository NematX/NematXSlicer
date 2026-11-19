# Shared host SDK linkage for the plugin and the focused API harness.
if(NOT SUPERSLICER_SOURCE_DIR)
    message(FATAL_ERROR "Set SUPERSLICER_SOURCE_DIR to the host source checkout.")
endif()
if(NOT IS_ABSOLUTE "${SUPERSLICER_SOURCE_DIR}")
    message(FATAL_ERROR "SUPERSLICER_SOURCE_DIR must be an absolute path: ${SUPERSLICER_SOURCE_DIR}")
endif()
if(NOT EXISTS "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/GeometryViews.hpp")
    message(FATAL_ERROR
        "SUPERSLICER_SOURCE_DIR must contain the plugin SDK; missing "
        "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/GeometryViews.hpp")
endif()
if(NOT SUPERSLICER_HOST_LIBRARY)
    message(FATAL_ERROR "Set SUPERSLICER_HOST_LIBRARY to the linkable host library for this target.")
endif()
if(NOT IS_ABSOLUTE "${SUPERSLICER_HOST_LIBRARY}")
    message(FATAL_ERROR "SUPERSLICER_HOST_LIBRARY must be an absolute path: ${SUPERSLICER_HOST_LIBRARY}")
endif()
if(NOT EXISTS "${SUPERSLICER_HOST_LIBRARY}")
    message(FATAL_ERROR "Host library not found: ${SUPERSLICER_HOST_LIBRARY}")
endif()

# Unix hosts commonly expose a static libslic3r archive. Callers must provide
# its transitive libraries explicitly; the host source tree is not guessed here.
add_library(nematx_host_sdk INTERFACE)
target_include_directories(nematx_host_sdk INTERFACE "${SUPERSLICER_SOURCE_DIR}/src")
target_link_libraries(nematx_host_sdk INTERFACE
    "${SUPERSLICER_HOST_LIBRARY}"
    ${SUPERSLICER_HOST_LINK_LIBRARIES})
