# REST server targets are kept separate from the GUI so they can be built headless.

# Provided by the deps/+Argon2 project; only REST builds require it.
find_package(Argon2 CONFIG REQUIRED)

# copy_directory_if_different avoids rewriting the whole 90 MB resource tree on
# every incremental REST build; keep the unconditional copy on older CMake.
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.26)
    set(_slic3r_rest_copy_directory copy_directory_if_different)
else ()
    set(_slic3r_rest_copy_directory copy_directory)
endif ()

add_custom_target(slic3r_rest_resources
    COMMAND "${CMAKE_COMMAND}" -E ${_slic3r_rest_copy_directory}
        "${SLIC3R_RESOURCES_DIR}" "${SLIC3R_BUILD_RESOURCES_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory
        "${SLIC3R_BUILD_RESOURCES_DIR}/plugins"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${SLIC3R_GENERATED_DEFAULT_ACTIVATED_FILE}"
        "${SLIC3R_BUILD_RESOURCES_DIR}/plugins/default_activated.ini"
    COMMENT "Preparing headless REST resources"
    VERBATIM
)

add_library(slic3r_rest_auth STATIC
    auth.cpp
    auth.hpp
)

target_include_directories(slic3r_rest_auth
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(slic3r_rest_auth PRIVATE boost_libs Argon2::argon2)
target_compile_definitions(slic3r_rest_auth PRIVATE NOMINMAX)
if (WIN32)
    target_link_libraries(slic3r_rest_auth PRIVATE bcrypt.lib)
endif ()
set_property(TARGET slic3r_rest_auth PROPERTY FOLDER "rest_server")

add_library(slic3r_rest STATIC
    file_names.cpp
    file_names.hpp
    multipart_upload.cpp
    multipart_upload.hpp
    profiles.cpp
    profiles.hpp
    rest_server.cpp
    rest_server.hpp
    settings_catalog.cpp
    settings_codec.cpp
    settings.hpp
    settings_internal.hpp
    temporary_file.cpp
    temporary_file.hpp
    transform.cpp
    transform.hpp
)

target_include_directories(slic3r_rest
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(slic3r_rest PRIVATE boost_libs slic3r_rest_auth slic3r_rest_session libslic3r)
target_compile_definitions(slic3r_rest PRIVATE NOMINMAX)

add_executable(slic3r_rest_password
    password_tool.cpp
)
target_link_libraries(slic3r_rest_password PRIVATE slic3r_rest_auth boost_libs)
set_property(TARGET slic3r_rest_password PROPERTY FOLDER "rest_server")

if (WIN32)
    target_compile_definitions(slic3r_rest PRIVATE _WIN32_WINNT=0x0601)
endif ()

set_property(TARGET slic3r_rest PROPERTY FOLDER "rest_server")

add_library(slic3r_rest_engine STATIC
    engine.cpp
    engine.hpp
)

target_include_directories(slic3r_rest_engine
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(slic3r_rest_engine PUBLIC libslic3r)
add_dependencies(slic3r_rest_engine slic3r_rest_resources)

if (WIN32)
    target_compile_definitions(slic3r_rest_engine PRIVATE _WIN32_WINNT=0x0601)
endif ()

set_property(TARGET slic3r_rest_engine PROPERTY FOLDER "rest_server")

add_library(slic3r_rest_session STATIC
    session.cpp
    session.hpp
)

target_include_directories(slic3r_rest_session
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(slic3r_rest_session PUBLIC libslic3r)

if (WIN32)
    target_compile_definitions(slic3r_rest_session PRIVATE _WIN32_WINNT=0x0601)
endif ()

set_property(TARGET slic3r_rest_session PROPERTY FOLDER "rest_server")

add_executable(slic3r_rest_server
    main.cpp
)

target_link_libraries(slic3r_rest_server PRIVATE slic3r_rest slic3r_rest_engine)
set_property(TARGET slic3r_rest_server PROPERTY FOLDER "rest_server")
