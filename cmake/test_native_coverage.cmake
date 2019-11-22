project( emb_arch_test_native_coverage )

# Headers #
set( TESTS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/tests/inc" CACHE INTERNAL "CMake include directory for tests" )
set( ENV_INCLUDE_DIR "${PROJECT_ROOT_DIR}/env/inc" CACHE INTERNAL "CMake include directory for env templates" )
set( ARRAY_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/array/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( DATA_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/data/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( FUNCTION_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/function/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( ENV_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/env/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( TUPLE_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/tuple/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( TYPE_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/type/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( COMPONENT_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/component/inc" CACHE INTERNAL "CMake include directory for components" )
set( TEST_FW_INCLUDE_DIR "${PROJECT_ROOT_DIR}/tests/Test" CACHE INTERNAL "CMake include directory for test framework" )

# Sources #
file( GLOB TEST_FW_SRC "${PROJECT_ROOT_DIR}/tests/Test/test.cpp" CACHE INTERNAL "CMake source file for test framework" )
file( GLOB TEST_HOOKS_SRC "${PROJECT_ROOT_DIR}/tests/src/hooks/*.cpp" CACHE INTERNAL "CMake source files for hook tests" )
file( GLOB TEST_SEQUENCES_SRC "${PROJECT_ROOT_DIR}/tests/src/sequences/*.cpp" CACHE INTERNAL "CMake source files for sequence tests" )
file( GLOB TEST_FUNCTORS_SRC "${PROJECT_ROOT_DIR}/tests/src/functors/*.cpp" CACHE INTERNAL "CMake source files for functor tests" )
file( GLOB TEST_PORTS_SRC "${PROJECT_ROOT_DIR}/tests/src/port/*.cpp" CACHE INTERNAL "CMake source files for port tests" )
file( GLOB ENV_SRC "${PROJECT_ROOT_DIR}/env/src/*.cpp" CACHE INTERNAL "CMake source files for env templates" )
file( GLOB UTIL_SRC "${PROJECT_ROOT_DIR}/utils/env/src/*.cpp" CACHE INTERNAL "CMake source files for utility templates" )

# Set env #
set( CMAKE_C_COMPILER "/usr/bin/clang" CACHE INTERNAL "CMake native C compiler" )
set( CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE INTERNAL "CMake native CXX compiler" )
set( CMAKE_OBJCOPY_UTIL "/usr/bin/objcopy" CACHE INTERNAL "CMake native objcopy util" )
set( CMAKE_SIZE_UTIL "/usr/bin/size" CACHE INTERNAL "CMake native size util" )
set( CMAKE_OBJDUMP_UTIL "/usr/bin/objdump" CACHE INTERNAL "CMake native objdump util" )
set( CMAKE_GDB_BACKEND "/usr/bin/gdb-multiarch" CHACHE INTERNAL "CMake native gdb backend" )
set( CMAKE_GDB_FRONTEND "/usr/bin/cgdb" CHACHE INTERNAL "CMake native gdb frontend" )

set( COVERAGE_OUT_DIR_NAME "coverage" )
configure_file( "${PROJECT_ROOT_DIR}/cmake/coverage.sh.in" "${PROJECT_ROOT_DIR}/scripts/generated/coverage.sh" )

# Set flags #
set( CMAKE_C_FLAGS "-Og -g3 -pipe -std=gnu11 --coverage -fprofile-arcs -ftest-coverage -fno-inline -fno-inline-small-functions -fno-default-inline -fprofile-generate -fno-elide-constructors" CACHE INTERNAL "CMake C Flags for test native coverage target" )
set( CMAKE_CXX_FLAGS "-Og -g3 -pipe -std=gnu++17 --coverage -fprofile-arcs -ftest-coverage -fno-inline -fno-inline-small-functions -fno-default-inline -fprofile-generate -fno-elide-constructors" CACHE INTERNAL "CMake C Flags for test native coverage target" )
set( CMAKE_EXE_LINKER_FLAGS "-Og -g3 -pipe -std=gnu++17 --coverage -fprofile-arcs -ftest-coverage -fno-inline -fno-inline-small-functions -fno-default-inline -fno-elide-constructors -fprofile-generate -lgcov" CACHE INTERNAL "CMake Linker Flags for test native coverage target" )

include_directories(
  ${TESTS_INCLUDE_DIR}
  ${ENV_INCLUDE_DIR}
  ${ARRAY_UTILS_INCLUDE_DIR}
  ${DATA_UTILS_INCLUDE_DIR}
  ${FUNCTION_UTILS_INCLUDE_DIR}
  ${ENV_UTILS_INCLUDE_DIR}
  ${TUPLE_UTILS_INCLUDE_DIR}
  ${TYPE_UTILS_INCLUDE_DIR}
  ${TEST_FW_INCLUDE_DIR}
  ${COMPONENT_UTILS_INCLUDE_DIR} )

add_executable( ${CMAKE_PROJECT_NAME}.elf
  ${TEST_FW_SRC}
  ${TEST_HOOKS_SRC}
  ${TEST_SEQUENCES_SRC}
  ${TEST_FUNCTORS_SRC}
  ${TEST_PORTS_SRC}
  ${ENV_SRC}
  ${UTIL_SRC} )

target_link_libraries( ${CMAKE_PROJECT_NAME}.elf pthread )
add_custom_command( TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND sh ARGS ${PROJECT_ROOT_DIR}/scripts/generated/coverage.sh )
