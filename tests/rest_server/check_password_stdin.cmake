# A pipe/file is the supported noninteractive credential input; never use argv.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/password-stdin-test.txt" "test-secret\n")
set(args --password-stdin --username TestUser)
if (DEFINED MODE)
    list(PREPEND args "${MODE}")
endif ()
execute_process(COMMAND "${EXECUTABLE}" ${args}
    INPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/password-stdin-test.txt"
    OUTPUT_VARIABLE output ERROR_VARIABLE error RESULT_VARIABLE result TIMEOUT 300)
file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/password-stdin-test.txt")
if (NOT result STREQUAL "0" OR NOT error STREQUAL "" OR output MATCHES "test-secret")
    message(FATAL_ERROR "Password stdin command failed (${result}): ${error}")
endif ()
string(JSON hash GET "${output}" users TestUser password_hash)
if (NOT hash MATCHES "^\\$argon2id\\$")
    message(FATAL_ERROR "Expected an Argon2id account document")
endif ()
