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
    rest_engine_tests
    rest_session_tests
    rest_auth_tests
    PROPERTIES TIMEOUT 120)
