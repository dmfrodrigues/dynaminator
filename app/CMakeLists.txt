cmake_minimum_required(VERSION 3.20)

project(simulator)

include_directories("structs/include")

add_executable(simulator main.cpp)

# ################ TESTS ################
file(GLOB_RECURSE TESTS_SRC "test/*.cpp")
find_package(Catch2 REQUIRED)
add_executable(tests ${TESTS_SRC})
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
include(CTest)
include(Catch)
catch_discover_tests(tests)
