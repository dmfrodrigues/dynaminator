cmake_minimum_required(VERSION 3.20)

project(simulator)

include_directories("include")
file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(simulator main.cpp ${SRC})

# ################ TESTS ################
file(GLOB_RECURSE TESTS_SRC "test/*.cpp")
find_package(Catch2 REQUIRED)
add_executable(tests ${TESTS_SRC} ${SRC})
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
include(CTest)
include(Catch)
catch_discover_tests(tests)
