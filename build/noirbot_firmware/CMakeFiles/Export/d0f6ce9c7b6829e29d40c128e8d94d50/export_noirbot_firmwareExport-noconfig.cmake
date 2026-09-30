#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "noirbot_firmware::noirbot_firmware" for configuration ""
set_property(TARGET noirbot_firmware::noirbot_firmware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(noirbot_firmware::noirbot_firmware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libnoirbot_firmware.so"
  IMPORTED_SONAME_NOCONFIG "libnoirbot_firmware.so"
  )

list(APPEND _cmake_import_check_targets noirbot_firmware::noirbot_firmware )
list(APPEND _cmake_import_check_files_for_noirbot_firmware::noirbot_firmware "${_IMPORT_PREFIX}/lib/libnoirbot_firmware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
