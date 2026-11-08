cmake_minimum_required(VERSION 3.12)

project(Argon2 LANGUAGES C VERSION 1.0.0)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

add_library(argon2 STATIC
    src/argon2.c
    src/blake2/blake2b.c
    src/core.c
    src/encoding.c
    src/ref.c
)

target_compile_definitions(argon2 PRIVATE ARGON2_NO_THREADS)
target_include_directories(argon2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
    TARGETS argon2
    EXPORT Argon2Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES include/argon2.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(_argon2_package_directory ${CMAKE_INSTALL_LIBDIR}/cmake/Argon2)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/Argon2ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/Argon2Config.cmake"
    INSTALL_DESTINATION ${_argon2_package_directory}
)

install(
    EXPORT Argon2Targets
    FILE Argon2Targets.cmake
    NAMESPACE Argon2::
    DESTINATION ${_argon2_package_directory}
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/Argon2Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/Argon2ConfigVersion.cmake"
    DESTINATION ${_argon2_package_directory}
)
