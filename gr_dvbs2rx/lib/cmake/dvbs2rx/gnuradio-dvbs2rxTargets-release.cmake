#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gnuradio::gnuradio-dvbs2rx" for configuration "Release"
set_property(TARGET gnuradio::gnuradio-dvbs2rx APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(gnuradio::gnuradio-dvbs2rx PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgnuradio-dvbs2rx.so.1.4.0.0"
  IMPORTED_SONAME_RELEASE "libgnuradio-dvbs2rx.so.1.4.0"
  )

list(APPEND _cmake_import_check_targets gnuradio::gnuradio-dvbs2rx )
list(APPEND _cmake_import_check_files_for_gnuradio::gnuradio-dvbs2rx "${_IMPORT_PREFIX}/lib/libgnuradio-dvbs2rx.so.1.4.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
