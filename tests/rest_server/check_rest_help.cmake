# Assert both a successful process exit and REST help text; a regex alone can
# make CTest ignore a nonzero exit code when the expected output is present.
if (DEFINED MODE)
    execute_process(COMMAND "${EXECUTABLE}" "${MODE}" --help RESULT_VARIABLE result
        OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 300)
else ()
    execute_process(COMMAND "${EXECUTABLE}" --help RESULT_VARIABLE result
        OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 300)
endif ()
get_filename_component(program "${EXECUTABLE}" NAME)
string(FIND "${output}" "${program}" program_position)
if (DEFINED MODE)
    string(FIND "${output}" "${MODE}" mode_position)
else ()
    set(mode_position 0)
endif ()
if (NOT result STREQUAL "0" OR NOT output MATCHES "--config" OR
    program_position EQUAL -1 OR mode_position EQUAL -1)
    message(FATAL_ERROR "Unexpected REST help result: ${result}\nstdout: ${output}\nstderr: ${error}")
endif ()
