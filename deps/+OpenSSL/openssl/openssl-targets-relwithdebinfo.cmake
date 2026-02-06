#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

if(WIN32)
	set(_lib_extension ".lib")
	message("import releaseWdeb openssl on windows with libssl${_lib_extension}")
else()
	set(_lib_extension ".a")
	message("import releaseWdeb openssl on linux/macos with libssl${_lib_extension}")
endif()

# Import target "OpenSSL::SSL" for configuration "RelWithDebInfo"
set_property(TARGET OpenSSL::SSL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(OpenSSL::SSL PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libssl${_lib_extension}"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenSSL::SSL )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenSSL::SSL "${_IMPORT_PREFIX}/lib/libssl${_lib_extension}" )

# Import target "OpenSSL::Crypto" for configuration "RelWithDebInfo"
set_property(TARGET OpenSSL::Crypto APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(OpenSSL::Crypto PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libcrypto${_lib_extension}"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenSSL::Crypto )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenSSL::Crypto "${_IMPORT_PREFIX}/lib/libcrypto${_lib_extension}" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
