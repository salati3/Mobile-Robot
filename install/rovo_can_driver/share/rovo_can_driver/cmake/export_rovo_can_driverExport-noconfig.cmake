#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rovo_can_driver::rovo_can_driver" for configuration ""
set_property(TARGET rovo_can_driver::rovo_can_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rovo_can_driver::rovo_can_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librovo_can_driver.so"
  IMPORTED_SONAME_NOCONFIG "librovo_can_driver.so"
  )

list(APPEND _cmake_import_check_targets rovo_can_driver::rovo_can_driver )
list(APPEND _cmake_import_check_files_for_rovo_can_driver::rovo_can_driver "${_IMPORT_PREFIX}/lib/librovo_can_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
