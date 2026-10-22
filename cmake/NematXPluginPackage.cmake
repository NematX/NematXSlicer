include(CMakeParseArguments)

function(nematx_plugin_platform_arch output_variable)
    if(WIN32)
        set(target_os windows)
    elseif(APPLE)
        set(target_os macos)
    elseif(UNIX)
        set(target_os linux)
    else()
        message(FATAL_ERROR "Unsupported target operating system for plugin packaging.")
    endif()

    if(CMAKE_OSX_ARCHITECTURES)
        set(architectures ${CMAKE_OSX_ARCHITECTURES})
    elseif(CMAKE_GENERATOR_PLATFORM)
        set(architectures "${CMAKE_GENERATOR_PLATFORM}")
    elseif(CMAKE_CXX_COMPILER_ARCHITECTURE_ID)
        set(architectures "${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}")
    elseif(CMAKE_SYSTEM_PROCESSOR)
        set(architectures "${CMAKE_SYSTEM_PROCESSOR}")
    else()
        message(FATAL_ERROR "Could not determine the target architecture from the CMake toolchain.")
    endif()

    set(normalized_architectures)
    foreach(architecture IN LISTS architectures)
        string(REGEX REPLACE ",.*$" "" architecture "${architecture}")
        string(TOLOWER "${architecture}" architecture)
        if(architecture MATCHES "^(x64|x86_64|amd64|x86-64)$")
            list(APPEND normalized_architectures x86_64)
        elseif(architecture MATCHES "^(arm64|aarch64)$")
            list(APPEND normalized_architectures arm64)
        else()
            message(FATAL_ERROR "Unsupported target architecture for plugin packaging: ${architecture}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES normalized_architectures)
    if(APPLE AND NOT "${normalized_architectures}" STREQUAL "arm64")
        message(FATAL_ERROR "This plugin package supports macOS ARM64 only; got ${normalized_architectures}.")
    endif()
    list(JOIN normalized_architectures "_" architecture_slug)
    set(${output_variable} "${target_os}-${architecture_slug}" PARENT_SCOPE)
endfunction()

function(nematx_package_plugin)
    cmake_parse_arguments(PKG "" "TARGET;PLUGIN_ID;PACKAGE_ID;PACKAGE_VERSION;NAME;DESCRIPTION;DATADIR;ACTIVATION_SCRIPT;ABI_SCRIPT;DESCRIPTION_TEMPLATE;VERSION_TEMPLATE;SLICER_VERSION;PLUGIN_REPOSITORY_DIR;DEFAULT_ACTIVATED_FILE;MODEL;CONFIG;DEBUGGER_COMMAND;DEBUGGER_WORKING_DIRECTORY;OUTPUT_SUBDIR" "" ${ARGN})

    foreach(required_arg
            TARGET PLUGIN_ID PACKAGE_ID PACKAGE_VERSION NAME DESCRIPTION DATADIR
            ACTIVATION_SCRIPT ABI_SCRIPT DESCRIPTION_TEMPLATE VERSION_TEMPLATE
            SLICER_VERSION DEFAULT_ACTIVATED_FILE)
        if(NOT PKG_${required_arg})
            message(FATAL_ERROR "nematx_package_plugin requires ${required_arg}")
        endif()
    endforeach()

    if(NOT TARGET ${PKG_TARGET})
        message(FATAL_ERROR "nematx_package_plugin target does not exist: ${PKG_TARGET}")
    endif()

    nematx_plugin_platform_arch(platform_arch)
    set(package_output_base "${CMAKE_BINARY_DIR}/package/${platform_arch}/$<CONFIG>")
    set(package_output_dir "${package_output_base}")
    if(PKG_OUTPUT_SUBDIR)
        set(package_output_dir "${package_output_dir}/${PKG_OUTPUT_SUBDIR}")
    endif()
    set(metadata_dir "${CMAKE_BINARY_DIR}/metadata/${PKG_TARGET}")
    set(installed_dir "${PKG_DATADIR}/plugins/${PKG_PACKAGE_ID}")
    set(package_name "${PKG_PACKAGE_ID}_${PKG_PACKAGE_VERSION}_${PKG_SLICER_VERSION}.zip")
    set(package_archive "${package_output_base}/${package_name}")

    file(MAKE_DIRECTORY "${metadata_dir}")
    set(PLUGIN_ID "${PKG_PACKAGE_ID}")
    set(PLUGIN_NAME "${PKG_NAME}")
    set(PLUGIN_DESCRIPTION "${PKG_DESCRIPTION}")
    set(PLUGIN_PACKAGE_VERSION "${PKG_PACKAGE_VERSION}")
    set(SUPERSLICER_VERSION_FULL "${PKG_SLICER_VERSION}")
    configure_file("${PKG_DESCRIPTION_TEMPLATE}" "${metadata_dir}/description.ini" @ONLY)
    configure_file("${PKG_VERSION_TEMPLATE}" "${metadata_dir}/version-base.ini" @ONLY)

    set_target_properties(${PKG_TARGET} PROPERTIES
        PREFIX ""
        OUTPUT_NAME "plugin"
        RUNTIME_OUTPUT_DIRECTORY "${package_output_dir}"
        LIBRARY_OUTPUT_DIRECTORY "${package_output_dir}"
    )
    if(MSVC)
        target_link_options(${PKG_TARGET} PRIVATE "$<$<CONFIG:RelWithDebInfo>:/DEBUG>")
        set_target_properties(${PKG_TARGET} PROPERTIES
            PDB_OUTPUT_DIRECTORY "${package_output_dir}"
        )
    endif()

    if(MSVC AND PKG_DEBUGGER_COMMAND)
        set_target_properties(${PKG_TARGET} PROPERTIES
            VS_DEBUGGER_COMMAND "${PKG_DEBUGGER_COMMAND}"
            VS_DEBUGGER_WORKING_DIRECTORY "${PKG_DEBUGGER_WORKING_DIRECTORY}"
        )
        if(PKG_CONFIG AND PKG_MODEL)
            get_filename_component(model_filename "${PKG_MODEL}" NAME)
            set(staged_model "${PKG_DATADIR}/${model_filename}")
            set_target_properties(${PKG_TARGET} PROPERTIES
                VS_DEBUGGER_COMMAND_ARGUMENTS
                    "--datadir \"${PKG_DATADIR}\" --export-gcode --load \"${PKG_CONFIG}\" --output \"${PKG_DATADIR}/debugger-output.gcode\" \"${staged_model}\""
            )
        endif()
    endif()

    set(pdb_stage_command)
    if(MSVC)
        list(APPEND pdb_stage_command
            COMMAND "${CMAKE_COMMAND}"
                "-DSOURCE=$<TARGET_PDB_FILE:${PKG_TARGET}>"
                "-DDESTINATION=${installed_dir}/plugin.pdb"
                -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/copy_if_exists.cmake"
        )
    endif()

    set_property(TARGET ${PKG_TARGET} APPEND PROPERTY LINK_DEPENDS
        "${PKG_ACTIVATION_SCRIPT}"
        "${PKG_ABI_SCRIPT}"
        "${PKG_DESCRIPTION_TEMPLATE}"
        "${PKG_VERSION_TEMPLATE}"
    )
    if(MSVC)
        set_property(TARGET ${PKG_TARGET} APPEND PROPERTY LINK_DEPENDS
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/copy_if_exists.cmake")
    endif()

    set(repository_commands)
    if(PKG_PLUGIN_REPOSITORY_DIR)
        list(APPEND repository_commands
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${PKG_PLUGIN_REPOSITORY_DIR}"
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${package_archive}"
                "${PKG_PLUGIN_REPOSITORY_DIR}/${package_name}"
        )
    endif()

    add_custom_command(TARGET ${PKG_TARGET} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${metadata_dir}/description.ini"
            "$<TARGET_FILE_DIR:${PKG_TARGET}>/description.ini"
        COMMAND "${Python3_EXECUTABLE}" "${PKG_ABI_SCRIPT}"
            --base "${metadata_dir}/version-base.ini"
            --binary "$<TARGET_FILE:${PKG_TARGET}>"
            --output "$<TARGET_FILE_DIR:${PKG_TARGET}>/version.ini"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${package_output_base}"
        COMMAND "${CMAKE_COMMAND}" -E tar cf "${package_archive}"
            --format=zip -- "$<TARGET_FILE_NAME:${PKG_TARGET}>" description.ini version.ini
        ${repository_commands}
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${installed_dir}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:${PKG_TARGET}>"
            "${installed_dir}/$<TARGET_FILE_NAME:${PKG_TARGET}>"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE_DIR:${PKG_TARGET}>/description.ini"
            "${installed_dir}/description.ini"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE_DIR:${PKG_TARGET}>/version.ini"
            "${installed_dir}/version.ini"
        ${pdb_stage_command}
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${PKG_DATADIR}/plugins"
        COMMAND "${Python3_EXECUTABLE}" "${PKG_ACTIVATION_SCRIPT}"
            --defaults "${PKG_DEFAULT_ACTIVATED_FILE}"
            --output "${PKG_DATADIR}/plugins/activated.ini"
            --plugin-id "${PKG_PLUGIN_ID}"
            --package-id "${PKG_PACKAGE_ID}"
            --plugin-version "${PKG_PACKAGE_VERSION}"
            --slicer-version "${PKG_SLICER_VERSION}"
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PKG_TARGET}>"
        COMMENT "Packaging ${PKG_PACKAGE_ID} and staging its isolated datadir"
        VERBATIM
    )

    if(PKG_MODEL)
        get_filename_component(model_filename "${PKG_MODEL}" NAME)
        add_custom_command(TARGET ${PKG_TARGET} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${PKG_MODEL}"
                "${PKG_DATADIR}/${model_filename}"
            VERBATIM
        )
    endif()
endfunction()
