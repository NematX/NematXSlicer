if(NOT DEFINED SOURCE OR NOT DEFINED DESTINATION)
    message(FATAL_ERROR "copy_if_exists.cmake requires SOURCE and DESTINATION.")
endif()

if(EXISTS "${SOURCE}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${SOURCE}" "${DESTINATION}"
        RESULT_VARIABLE copy_result
    )
    if(NOT copy_result EQUAL 0)
        message(FATAL_ERROR "Could not copy ${SOURCE} to ${DESTINATION}: ${copy_result}")
    endif()
endif()
