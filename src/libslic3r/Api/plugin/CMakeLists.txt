cmake_minimum_required(VERSION 3.16)

# This directory may also be built on its own from a copy of the public SDK.
# No host C++ headers, precompiled header or host compile definitions are needed.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(slic3r_plugin_sdk LANGUAGES CXX)
endif()

set(_sdk_sources
    cpp/FanPolicy.cpp
    cpp/HoleCompensation.cpp
    cpp/PrintHelpers.cpp
    cpp/gcode/BuiltinGCodeReaderBase.cpp
    cpp/gcode/DefaultExtruder.cpp
    cpp/gcode/DefaultGCodeFirmwareSession.cpp
    cpp/gcode/ExtrusionAxisState.cpp
    cpp/gcode/FanState.cpp
    cpp/gcode/Gantry.cpp
    cpp/gcode/GCodeFormatter.cpp
    cpp/gcode/GCodeLineParser.cpp
    cpp/gcode/GCodePreamble.cpp
    cpp/gcode/HeaterState.cpp
    cpp/gcode/MachineEnvelope.cpp
    cpp/gcode/PressureAdvanceState.cpp
    cpp/gcode/Printer.cpp
    cpp/gcode/SingleAccelerationRegisterGCodeFirmwareSession.cpp
    cpp/gcode/Tool.cpp
)
add_library(slic3r_plugin_sdk_cpp_objects OBJECT ${_sdk_sources})
add_library(slic3r_plugin_sdk_cpp STATIC)
# Build the helpers once. External plugins get an archive; the host incorporates
# these same objects beside its C ABI implementation. Keeping both in one host
# archive avoids circular archive dependencies with Unix linkers.
target_link_libraries(slic3r_plugin_sdk_cpp PUBLIC slic3r_plugin_sdk_cpp_objects)
add_library(slic3r::plugin_sdk_cpp ALIAS slic3r_plugin_sdk_cpp)
target_compile_features(slic3r_plugin_sdk_cpp_objects PUBLIC cxx_std_17)
set_target_properties(slic3r_plugin_sdk_cpp_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)
get_filename_component(_sdk_include_root "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
target_include_directories(slic3r_plugin_sdk_cpp_objects PUBLIC "${_sdk_include_root}")

# Both dependencies are header-only and private: consumers need only SDK headers
# and the host C ABI import library, not Boost libraries or host internals.
if(TARGET boost_headeronly)
    target_link_libraries(slic3r_plugin_sdk_cpp_objects PRIVATE boost_headeronly)
else()
    find_package(Boost REQUIRED)
    target_include_directories(slic3r_plugin_sdk_cpp_objects SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
endif()
find_path(SLIC3R_SDK_FAST_FLOAT_INCLUDE_DIR fast_float/fast_float.h HINTS "${_sdk_include_root}")
if(NOT SLIC3R_SDK_FAST_FLOAT_INCLUDE_DIR)
    message(FATAL_ERROR "The plugin SDK needs the fast_float headers (set SLIC3R_SDK_FAST_FLOAT_INCLUDE_DIR).")
endif()
target_include_directories(slic3r_plugin_sdk_cpp_objects SYSTEM PRIVATE "${SLIC3R_SDK_FAST_FLOAT_INCLUDE_DIR}")
if(MSVC)
    target_compile_options(slic3r_plugin_sdk_cpp_objects PRIVATE /EHsc /utf-8 /bigobj)
endif()

# The host uses the very same implementation, without compiling these files a
# second time into libslic3r. External projects simply add_subdirectory this SDK.
set(SLIC3R_PLUGIN_SDK_CPP_SOURCES "")
foreach(_source IN LISTS _sdk_sources)
    list(APPEND SLIC3R_PLUGIN_SDK_CPP_SOURCES "Api/plugin/${_source}")
endforeach()
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(SLIC3R_PLUGIN_SDK_CPP_SOURCES "${SLIC3R_PLUGIN_SDK_CPP_SOURCES}" PARENT_SCOPE)
endif()
