cmake_minimum_required(VERSION 3.16)
project(xy_plugin_sdk_only LANGUAGES CXX)

# This intentionally independent project cannot inherit the slicer's include
# directories or PCH. Compile a sandbox containing only the public SDK and the
# plugin's private sources: accidental host includes become compiler errors.
get_filename_component(_slicer_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_sandbox "${CMAKE_CURRENT_BINARY_DIR}/sandbox")
file(COPY "${_slicer_root}/src/libslic3r/Api/plugin"
     DESTINATION "${_sandbox}/libslic3r/Api")
set(_plugin_sources)
foreach(_name XYSizeCompensation XYSizeCompensationHelpers ElephantFootCompensation HoleSizeCompensation)
    foreach(_extension cpp hpp)
        configure_file("${_slicer_root}/src/libslic3r/Plugins/${_name}.${_extension}"
                       "${_sandbox}/libslic3r/Plugins/${_name}.${_extension}" COPYONLY)
    endforeach()
    list(APPEND _plugin_sources "${_sandbox}/libslic3r/Plugins/${_name}.cpp")
endforeach()
add_library(xy_plugin_sdk_only OBJECT ${_plugin_sources})
target_compile_features(xy_plugin_sdk_only PRIVATE cxx_std_17)
target_include_directories(xy_plugin_sdk_only PRIVATE "${_sandbox}")
if(MSVC)
    target_compile_options(xy_plugin_sdk_only PRIVATE /EHsc /MP2 /FS)
endif()
