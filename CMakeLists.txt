cmake_minimum_required(VERSION 3.20)
project(can_j1939 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(j1939 INTERFACE)
target_include_directories(j1939 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

option(J1939_BUILD_TESTS "Build unit tests" ON)
option(J1939_BUILD_FREESTANDING_CHECK "Build the no-heap translation unit" ON)

if(J1939_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(j1939_tests tests/test_can_id.cpp tests/test_transport.cpp)
  target_include_directories(j1939_tests PRIVATE tests)
  target_link_libraries(j1939_tests PRIVATE j1939 GTest::gtest_main)
  target_compile_options(j1939_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME j1939_tests COMMAND j1939_tests)
endif()

if(J1939_BUILD_FREESTANDING_CHECK)
  # Proves the library itself compiles with exceptions and RTTI off and links
  # without pulling in the allocator. The symbol check in tools/check_no_heap.sh
  # asserts no operator new or malloc reference survives.
  add_executable(j1939_freestanding tools/freestanding_main.cpp)
  target_link_libraries(j1939_freestanding PRIVATE j1939)
  target_compile_options(j1939_freestanding PRIVATE
    -Wall -Wextra -Wpedantic -Werror
    -fno-exceptions -fno-rtti -fno-threadsafe-statics -Os)
endif()
