get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

add_executable(${_TEST_NAME}_tests
    rest_server_tests.cpp
)

target_link_libraries(${_TEST_NAME}_tests
    test_common
    slic3r_rest
    boost_libs
)

if (WIN32)
    target_compile_definitions(${_TEST_NAME}_tests PRIVATE _WIN32_WINNT=0x0601)
endif ()

set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

if (WIN32)
    prusaslicer_copy_dlls(${_TEST_NAME}_tests)
endif ()

add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests ${CATCH_EXTRA_ARGS})

# Launching finalizes the process-global engine; transport tests intentionally
# exercise the uninitialized engine, so they must run in another process.
add_executable(rest_launch_tests rest_launch_tests.cpp)
target_compile_definitions(rest_launch_tests PRIVATE
    SLIC3R_REST_SOURCE_DIRECTORY=R"\(${CMAKE_SOURCE_DIR}\)"
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)")
target_link_libraries(rest_launch_tests test_common slic3r_rest slic3r_rest_engine boost_libs)
if (WIN32)
    target_compile_definitions(rest_launch_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_launch_tests)
endif ()
set_property(TARGET rest_launch_tests PROPERTY FOLDER "tests")
add_test(rest_launch_tests rest_launch_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_deployment_tests rest_deployment_tests.cpp)
target_compile_definitions(rest_deployment_tests PRIVATE
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)"
    SLIC3R_REST_SOURCE_DIRECTORY=R"\(${CMAKE_SOURCE_DIR}\)")
target_link_libraries(rest_deployment_tests test_common slic3r_rest slic3r_rest_engine boost_libs)
if (WIN32)
    target_compile_definitions(rest_deployment_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_deployment_tests)
endif ()
set_property(TARGET rest_deployment_tests PROPERTY FOLDER "tests")
add_test(rest_deployment_tests rest_deployment_tests ${CATCH_EXTRA_ARGS})
set_tests_properties(rest_deployment_tests PROPERTIES TIMEOUT 600)
add_test(NAME rest_standalone_password_stdin COMMAND ${CMAKE_COMMAND}
    -DEXECUTABLE=$<TARGET_FILE:slic3r_rest_password> -P ${CMAKE_CURRENT_SOURCE_DIR}/check_password_stdin.cmake)
set_tests_properties(rest_standalone_password_stdin PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE)

add_executable(rest_profiles_tests
    rest_profile_review_tests.cpp
    rest_profile_subset_tests.cpp
    rest_profiles_tests.cpp
)

target_compile_definitions(rest_profiles_tests PRIVATE
    SLIC3R_REST_SOURCE_DIRECTORY=R"\(${CMAKE_SOURCE_DIR}\)"
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)"
)

target_link_libraries(rest_profiles_tests
    test_common
    slic3r_rest
    slic3r_rest_engine
    boost_libs
)

if (WIN32)
    target_compile_definitions(rest_profiles_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_profiles_tests)
endif ()

set_property(TARGET rest_profiles_tests PROPERTY FOLDER "tests")
add_test(rest_profiles_tests rest_profiles_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_models_tests
    custom_gcodes_tests.cpp
    cut_tests.cpp
    entity_settings_tests.cpp
    file_names_tests.cpp
    paint_tests.cpp
    placement_tests.cpp
    rest_entity_settings_tests.cpp
    rest_models_tests.cpp
    rest_custom_gcodes_tests.cpp
    rest_cut_tests.cpp
    rest_paint_tests.cpp
    rest_placement_tests.cpp
    rest_scene_edit_tests.cpp
    scene_edit_tests.cpp
    transform_tests.cpp
    viewer_json_tests.cpp
)

target_compile_definitions(rest_models_tests PRIVATE
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)"
)

target_link_libraries(rest_models_tests
    test_common
    slic3r_rest
    slic3r_rest_engine
    boost_libs
)

if (WIN32)
    target_compile_definitions(rest_models_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_models_tests)
endif ()

set_property(TARGET rest_models_tests PROPERTY FOLDER "tests")
add_test(rest_models_tests rest_models_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_settings_tests
    rest_settings_tests.cpp
)

target_compile_definitions(rest_settings_tests PRIVATE
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)"
)

target_link_libraries(rest_settings_tests
    test_common
    slic3r_rest
    slic3r_rest_engine
    boost_libs
)

if (WIN32)
    target_compile_definitions(rest_settings_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_settings_tests)
endif ()

set_property(TARGET rest_settings_tests PROPERTY FOLDER "tests")
add_test(rest_settings_tests rest_settings_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_engine_tests
    engine_tests.cpp
)

target_compile_definitions(rest_engine_tests PRIVATE
    SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)"
)

target_link_libraries(rest_engine_tests
    test_common
    slic3r_rest
    slic3r_rest_engine
)

if (WIN32)
    target_compile_definitions(rest_engine_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_engine_tests)
endif ()

set_property(TARGET rest_engine_tests PROPERTY FOLDER "tests")
add_test(rest_engine_tests rest_engine_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_session_tests
    session_tests.cpp
)

target_link_libraries(rest_session_tests
    test_common
    slic3r_rest_session
)

if (WIN32)
    target_compile_definitions(rest_session_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_session_tests)
endif ()

set_property(TARGET rest_session_tests PROPERTY FOLDER "tests")
add_test(rest_session_tests rest_session_tests ${CATCH_EXTRA_ARGS})

add_executable(rest_auth_tests
    auth_tests.cpp
)

target_link_libraries(rest_auth_tests
    test_common
    slic3r_rest_auth
    boost_libs
)

if (WIN32)
    target_compile_definitions(rest_auth_tests PRIVATE _WIN32_WINNT=0x0601)
    prusaslicer_copy_dlls(rest_auth_tests)
endif ()

set_property(TARGET rest_auth_tests PROPERTY FOLDER "tests")
add_test(rest_auth_tests rest_auth_tests ${CATCH_EXTRA_ARGS})

set_tests_properties(
    rest_server_tests
    rest_launch_tests
    rest_profiles_tests
    rest_settings_tests
    rest_engine_tests
    rest_session_tests
    rest_auth_tests
    PROPERTIES TIMEOUT 600)

set_tests_properties(rest_models_tests PROPERTIES TIMEOUT 600)

# Separate real slicing and workflow suites so their time budgets are independent.
foreach(_REST_SUITE IN ITEMS rest_slice rest_e2e)
    add_executable(${_REST_SUITE}_tests ${_REST_SUITE}_tests.cpp)
    target_compile_definitions(${_REST_SUITE}_tests PRIVATE
        SLIC3R_REST_RESOURCE_DIRECTORY=R"\(${SLIC3R_BUILD_RESOURCES_DIR}\)")
    target_link_libraries(${_REST_SUITE}_tests test_common slic3r_rest slic3r_rest_engine boost_libs)
    if (WIN32)
        target_compile_definitions(${_REST_SUITE}_tests PRIVATE _WIN32_WINNT=0x0601)
        prusaslicer_copy_dlls(${_REST_SUITE}_tests)
    endif ()
    set_property(TARGET ${_REST_SUITE}_tests PROPERTY FOLDER "tests")
    add_test(${_REST_SUITE}_tests ${_REST_SUITE}_tests ${CATCH_EXTRA_ARGS})
endforeach()
# Liveness guards, not performance thresholds: allow concurrent external builds.
set_tests_properties(rest_slice_tests rest_e2e_tests PROPERTIES TIMEOUT 1800)
# Real-engine suites already exercise concurrent slices internally. Avoid CPU
# oversubscription between executables while transport/settings tests run in parallel.
set_tests_properties(rest_engine_tests rest_slice_tests rest_e2e_tests
    PROPERTIES RESOURCE_LOCK rest_slicing)

# These commands exercise the actual executable/DLL dispatch, not an injected main.
add_test(NAME rest_standalone_help COMMAND ${CMAKE_COMMAND}
    -DEXECUTABLE=$<TARGET_FILE:slic3r_rest_server> -P ${CMAKE_CURRENT_SOURCE_DIR}/check_rest_help.cmake)
set_tests_properties(rest_standalone_help PROPERTIES TIMEOUT 600)
add_test(NAME rest_standalone_password_usage COMMAND ${CMAKE_COMMAND}
    -DEXECUTABLE=$<TARGET_FILE:slic3r_rest_password> -P ${CMAKE_CURRENT_SOURCE_DIR}/check_password_usage.cmake)
set_tests_properties(rest_standalone_password_usage PROPERTIES TIMEOUT 600)
if (TARGET Slic3r)
    if (WIN32)
        add_test(NAME rest_launcher_help COMMAND ${CMAKE_COMMAND}
            -DEXECUTABLE=$<TARGET_FILE:Slic3r_app_rest> -P ${CMAKE_CURRENT_SOURCE_DIR}/check_rest_help.cmake)
        set(_rest_console_target Slic3r_app_console)
    else ()
        add_test(NAME rest_launcher_help COMMAND ${CMAKE_COMMAND}
            -DEXECUTABLE=$<TARGET_FILE_DIR:Slic3r>/${SLIC3R_APP_CMD}-rest
            -P ${CMAKE_CURRENT_SOURCE_DIR}/check_rest_help.cmake)
        set(_rest_console_target Slic3r)
    endif ()
    add_test(NAME rest_app_help COMMAND ${CMAKE_COMMAND}
        -DEXECUTABLE=$<TARGET_FILE:${_rest_console_target}> -DMODE=--rest-server
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_rest_help.cmake)
    set_tests_properties(rest_app_help PROPERTIES TIMEOUT 600)
    add_test(NAME rest_app_password_usage COMMAND ${CMAKE_COMMAND}
        -DEXECUTABLE=$<TARGET_FILE:${_rest_console_target}> -DMODE=--rest-hash-password
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_password_usage.cmake)
    set_tests_properties(rest_launcher_help PROPERTIES TIMEOUT 600)
    set_tests_properties(rest_app_password_usage PROPERTIES TIMEOUT 600)
    add_test(NAME rest_app_password_stdin COMMAND ${CMAKE_COMMAND}
        -DEXECUTABLE=$<TARGET_FILE:${_rest_console_target}> -DMODE=--rest-hash-password
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_password_stdin.cmake)
    set_tests_properties(rest_app_password_stdin PROPERTIES TIMEOUT 600 LABELS rest_app RUN_SERIAL TRUE)
    set_tests_properties(rest_launcher_help rest_app_help rest_app_password_usage PROPERTIES LABELS rest_app)
endif ()
