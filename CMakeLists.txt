cmake_minimum_required( VERSION 3.12 )

function( build_project TARGET )

  set( BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build" CACHE INTERNAL "Project build directory" )
  set( PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "Project root directory" )
  
  include( "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${TARGET}.cmake" )
  configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/cmake/clang_complete.in" "${CMAKE_CURRENT_SOURCE_DIR}/.clang_complete" )
  
endfunction( build_project )

set(CMAKE_VERBOSE_MAKEFILE OFF)
build_project( ${TARGET} )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
