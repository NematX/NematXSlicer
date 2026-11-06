# Argon2 is pulled only when the REST server is enabled; the option is forwarded
# to the dependency build by deps/autobuild.cmake.
if (SLIC3R_BUILD_REST_SERVER)
    add_cmake_project(Argon2
        URL https://github.com/P-H-C/phc-winner-argon2/archive/refs/tags/20190702.tar.gz
        URL_HASH SHA256=daf972a89577f8772602bf2eb38b6a3dd3d922bf5724d45e7f9589b5e830442c
        PATCH_COMMAND
            ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
            ${CMAKE_CURRENT_LIST_DIR}/Config.cmake.in
            .
    )
endif ()
