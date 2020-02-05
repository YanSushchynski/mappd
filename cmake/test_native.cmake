project( emb_arch_test_native )

# Headers #
set( MAIN_INCLUDE_DIR "${PROJECT_ROOT_DIR}/inc" CACHE INTERNAL "CMake main include directory" )
set( ENV_INCLUDE_DIR "${PROJECT_ROOT_DIR}/env/inc" CACHE INTERNAL "CMake include directory for env templates" )
set( ARRAY_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/array/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( DATA_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/data/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( FUNCTION_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/function/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( ENV_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/env/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( TUPLE_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/tuple/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( TYPE_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/type/inc" CACHE INTERNAL "CMake include directory for util templates" )
set( COMPONENT_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/component/inc" CACHE INTERNAL "CMake include directory for components" )
set( JIT_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/jit/inc" CACHE INTERNAL "CMake include directory for JIT compilation" )
set( DL_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/dl/inc" CACHE INTERNAL "CMake include directory for dynamic shared objects loading" )
set( FNV1A_HASH_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/hash/fnv1a/inc" CACHE INTERNAL "CMake include directory for compile time fnv1a hashing" )
set( SHA256_HASH_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/hash/sha256" CACHE INTERNAL "CMake include directory for compile time sha256 hashing" )
set( STRING_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/string/inc" CACHE INTERNAL "CMake include directory for compile time strings" )
set( NET_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/net/inc" CACHE INTERNAL "CMake include directory for networking" )
set( THREAD_POOL_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/threads/inc" CACHE INTERNAL "CMake include directory for thread pool" )
set( CIDR_UTILS_INCLUDE_DIR "${PROJECT_ROOT_DIR}/utils/libcidr/inc" CACHE INTERNAL "CMake include directory for CIDR manipulating" )

# Sources #
file( GLOB MAIN_SRC "${PROJECT_ROOT_DIR}/src/*.cpp" CACHE INTERNAL "CMake main source files" )
file( GLOB TEST_HOOKS_SRC "${PROJECT_ROOT_DIR}/tests/src/hooks/*.cpp" CACHE INTERNAL "CMake source files for hook tests" )
file( GLOB TEST_SEQUENCES_SRC "${PROJECT_ROOT_DIR}/tests/src/sequences/*.cpp" CACHE INTERNAL "CMake source files for sequence tests" )
file( GLOB TEST_FUNCTORS_SRC "${PROJECT_ROOT_DIR}/tests/src/functors/*.cpp" CACHE INTERNAL "CMake source files for functor tests" )
file( GLOB TEST_PORTS_SRC "${PROJECT_ROOT_DIR}/tests/src/port/*.cpp" CACHE INTERNAL "CMake source files for port tests" )
file( GLOB TEST_ENV_SRC "${PROJECT_ROOT_DIR}/tests/src/env/*.cpp" CACHE INTERNAL "CMake source files for env tests" )
file( GLOB TEST_NET_SRC "${PROJECT_ROOT_DIR}/tests/src/net/*.cpp" CACHE INTERNAL "CMake source files for net tests" )
file( GLOB ENV_SRC "${PROJECT_ROOT_DIR}/env/src/*.cpp" CACHE INTERNAL "CMake source files for env templates" )
file( GLOB ENV_UTIL_SRC "${PROJECT_ROOT_DIR}/utils/env/src/*.cpp" CACHE INTERNAL "CMake source files for utility templates" )
file( GLOB JIT_UTIL_SRC "${PROJECT_ROOT_DIR}/utils/jit/src/*.cpp" CACHE INTERNAL "CMake source files for JIT" )
file( GLOB DL_UTIL_SRC "${PROJECT_ROOT_DIR}/utils/dl/src/*.cpp" CACHE INTERNAL "CMake source files for DL" )
file( GLOB SHA256_HASH_UTIL_SRC "${PROJECT_ROOT_DIR}/utils/hash/sha256/sha256.cpp" CACHE INTERNAL "CMake source files for sha256 hashing" )
file( GLOB CIDR_UTIL_SRC "${PROJECT_ROOT_DIR}/utils/libcidr/src/*.cpp" CACHE INTERNAL "CMake source files LibCIDR" )
file( GLOB TEST_MAIN_SRC "${PROJECT_ROOT_DIR}/tests/src/main.cpp" CACHE INTERNAL "Test main.cpp")

# Set env #
set( CMAKE_C_COMPILER "/usr/bin/clang" CACHE INTERNAL "CMake native C compiler" )
set( CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE INTERNAL "CMake native CXX compiler" )
set( CMAKE_OBJCOPY_UTIL "/usr/bin/objcopy" CACHE INTERNAL "CMake native objcopy util" )
set( CMAKE_SIZE_UTIL "/usr/bin/size" CACHE INTERNAL "CMake native size util" )
set( CMAKE_OBJDUMP_UTIL "/usr/bin/objdump" CACHE INTERNAL "CMake native objdump util" )
set( CMAKE_GDB_BACKEND "/usr/bin/gdb-multiarch" CHACHE INTERNAL "CMake native gdb backend" )

# Set flags #
set( CMAKE_C_FLAGS "-O0 -g -pipe -std=gnu11 -fno-inline" CACHE INTERNAL "CMake C Flags for test native target" )
set( CMAKE_CXX_FLAGS "-O0 -g -pipe -std=gnu++2a -fno-inline" CACHE INTERNAL "CMake C Flags for test native target" )
set( CMAKE_EXE_LINKER_FLAGS "-O0 -g -pipe -std=gnu++2a -fno-inline" CACHE INTERNAL "CMake Linker Flags for test native target" )

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
find_package(OpenSSL REQUIRED)

set(TRAITS_PROTO_DIR "${PROJECT_ROOT_DIR}/env/traits/proto" CACHE INTERNAL "Google protobuf files directory")
file(GLOB TRAITS_PROTO_FILES "${TRAITS_PROTO_DIR}/*.proto" CACHE INTERNAL "Google protobuf files")
set(PROTO_TRAITS_INC_DIR "${PROJECT_ROOT_DIR}/env/traits/inc" CACHE INTERNAL "Google protobuf generated headers directory")

protobuf_generate_cpp(PROTO_SRC PROTO_HEADER ${TRAITS_PROTO_FILES})

add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
add_library(cidr ${CIDR_UTILS_INCLUDE_DIR} ${CIDR_UTIL_SRC})
add_library(sha256 ${SHA256_HASH_UTILS_INCLUDE_DIR} ${SHA256_HASH_UTIL_SRC})

enable_testing()
include_directories(
  ${MAIN_INCLUDE_DIR}
  ${TESTS_INCLUDE_DIR}
  ${ENV_INCLUDE_DIR}
  ${ARRAY_UTILS_INCLUDE_DIR}
  ${DATA_UTILS_INCLUDE_DIR}
  ${FUNCTION_UTILS_INCLUDE_DIR}
  ${ENV_UTILS_INCLUDE_DIR}
  ${TUPLE_UTILS_INCLUDE_DIR}
  ${TYPE_UTILS_INCLUDE_DIR}
  ${COMPONENT_UTILS_INCLUDE_DIR}
  ${JIT_UTILS_INCLUDE_DIR}
  ${DL_UTILS_INCLUDE_DIR}
  ${FNV1A_HASH_UTILS_INCLUDE_DIR}
  ${SHA256_HASH_UTILS_INCLUDE_DIR}
  ${STRING_UTILS_INCLUDE_DIR}
  ${NET_UTILS_INCLUDE_DIR}
  ${PROTO_TRAITS_INC_DIR}
  ${CIDR_UTILS_INCLUDE_DIR}
  ${DL_UTILS_INCLUDE_DIR}
  ${THREAD_POOL_UTILS_INCLUDE_DIR})

add_executable( ${CMAKE_PROJECT_NAME}.elf
  # ${MAIN_SRC}
  ${TEST_HOOKS_SRC}
  ${TEST_SEQUENCES_SRC}
  ${TEST_FUNCTORS_SRC}
  # ${TEST_PORTS_SRC}
  ${TEST_ENV_SRC}
  ${TEST_NET_SRC}
  ${ENV_SRC}
  ${ENV_UTIL_SRC}
  #${JIT_UTIL_SRC}
  ${TEST_MAIN_SRC})

add_test(${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.elf)
target_link_libraries( ${CMAKE_PROJECT_NAME}.elf gtest crypto ssl sha256 cidr pthread proto protobuf dl config++ )
add_custom_command( TARGET proto POST_BUILD COMMAND "cp" ARGS "-rf" "*.pb.h" "${PROTO_TRAITS_INC_DIR}" )
add_custom_command( TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND "openssl" ARGS "genrsa" "-out" "ca_key.key" "2048" )
add_custom_command( TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND "openssl" ARGS "req" "-x509" "-new" "-key" "ca_key.key"
  "-days" "365" "-out" "ca_cert.crt" "-subj" "/C=XX/ST=XXX/L=XXX/O=Mappd Testing/OU=Developers/CN=mappd.localhost" )
