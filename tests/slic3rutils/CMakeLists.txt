get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests
    ${_TEST_NAME}_tests_main.cpp
    slic3r_jobs_tests.cpp
    slic3r_version_tests.cpp
    slic3r_arrangejob_tests.cpp
    )

# mold linker for successful linking needs also to link TBB library and link it before libslic3r.
target_link_libraries(${_TEST_NAME}_tests test_common TBB::tbb TBB::tbbmalloc libslic3r_gui libslic3r)
if (MSVC)
    target_link_libraries(${_TEST_NAME}_tests Setupapi.lib)
endif ()
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

if (WIN32)
    prusaslicer_copy_dlls(${_TEST_NAME}_tests)
endif()

# catch_discover_tests(${_TEST_NAME}_tests TEST_PREFIX "${_TEST_NAME}: ")
set(_catch_args "exclude:[NotWorking];-s")
list(APPEND _catch_args "${CATCH_EXTRA_ARGS}")
add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests ${_catch_args})

# This executable exercises only the GUI translation helpers, without the GUI application or core.
if(UNIX)
    set(wxWidgets_USE_UNICODE ON)
    set(wxWidgets_USE_STATIC ${SLIC3R_STATIC})
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(wxWidgets_CONFIG_OPTIONS "--toolkit=gtk${SLIC3R_GTK}")
endif()
find_package(wxWidgets 3.2 MODULE REQUIRED COMPONENTS base)
add_executable(host_i18n_tests host_i18n_tests.cpp)
target_include_directories(host_i18n_tests PRIVATE ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src ${PROJECT_BINARY_DIR}/src/libslic3r)
target_compile_definitions(host_i18n_tests PRIVATE wxUSE_GUI=0 "$<$<CONFIG:Debug>:${wxWidgets_DEFINITIONS_DEBUG}>")
# I18N.hpp exposes config definitions; provide their header dependencies without linking the core.
target_link_libraries(host_i18n_tests PRIVATE boost_headeronly libcereal TBB::tbb wxWidgets::wxWidgets)
set_property(TARGET host_i18n_tests PROPERTY FOLDER "tests")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME host_printbed_shader_resources COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_printbed_shader.py)
set(_identity_output ${CMAKE_CURRENT_BINARY_DIR}/identity-checks/$<CONFIG>)
add_test(NAME host_identity_resources COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_host_identity.py --output-dir ${_identity_output} --cmake ${CMAKE_COMMAND})
add_test(NAME host_identity_i18n COMMAND host_i18n_tests ${_identity_output}/catalogues)
set_tests_properties(host_identity_resources PROPERTIES FIXTURES_SETUP host_identity_catalogues)
set_tests_properties(host_identity_i18n PROPERTIES FIXTURES_REQUIRED host_identity_catalogues)

add_executable(host_config_tests host_config_tests.cpp)
target_include_directories(host_config_tests PRIVATE ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src ${PROJECT_BINARY_DIR}/src/libslic3r)
target_link_libraries(host_config_tests PRIVATE test_common TBB::tbb TBB::tbbmalloc libslic3r)
set_property(TARGET host_config_tests PROPERTY FOLDER "tests")
if(WIN32)
    prusaslicer_copy_dlls(host_config_tests)
endif()
add_test(NAME host_identity_config COMMAND host_config_tests)
set_tests_properties(host_identity_config PROPERTIES ENVIRONMENT "HOST_CONFIG_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/config-checks/$<CONFIG>")

# Execute the complete filament script without linking the GUI application or core.
add_executable(host_simple_script_tests host_simple_script_tests.cpp)
target_include_directories(host_simple_script_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(host_simple_script_tests PRIVATE test_common angelscript)
set_property(TARGET host_simple_script_tests PROPERTY FOLDER "tests")
add_test(NAME host_simple_filament_scripts COMMAND host_simple_script_tests
    "${PROJECT_SOURCE_DIR}/resources/ui_layout/default/filament.as"
    "${PROJECT_SOURCE_DIR}/resources/ui_layout/default/filament.ui")
