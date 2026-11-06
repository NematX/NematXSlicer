# Check the real wrapper's exit code and stderr together. PASS_REGULAR_EXPRESSION
# combined with WILL_FAIL can invert the regex success rather than the exit code.
if (DEFINED MODE)
    execute_process(COMMAND "${EXECUTABLE}" "${MODE}" RESULT_VARIABLE result
        OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 300)
else ()
    execute_process(COMMAND "${EXECUTABLE}" RESULT_VARIABLE result
        OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 300)
endif ()
get_filename_component(program "${EXECUTABLE}" NAME)
string(FIND "${error}" "${program}" program_position)
if (DEFINED MODE)
    string(FIND "${error}" "${MODE}" mode_position)
else ()
    set(mode_position 0)
endif ()
if (NOT result STREQUAL "1" OR NOT error MATCHES "usage: .*--username NAME" OR
    NOT output STREQUAL "" OR program_position EQUAL -1 OR mode_position EQUAL -1)
    message(FATAL_ERROR "Unexpected password usage result: ${result}\nstdout: ${output}\nstderr: ${error}")
endif ()
