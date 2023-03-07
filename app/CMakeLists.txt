cmake_minimum_required(VERSION 3.13)

project(simulator)

set(CMAKE_CXX_STANDARD 17)

include_directories("include" "test/include" "script/include")

# ################ SIMULATOR ################
file(GLOB_RECURSE SRC "src/*.cpp" "*/src/*.cpp")
add_executable(simulator main.cpp ${SRC})

# ################ TESTS ################
file(GLOB_RECURSE TESTS_SRC "test/main.cpp" "test/test_*.cpp" "test/src/*/*.cpp")
find_package(Catch2 REQUIRED)
add_executable(tests ${TESTS_SRC} ${SRC})
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
include(CTest)
include(Catch)
catch_discover_tests(tests)

# ################ SCRIPT ################
# file(GLOB_RECURSE SRC "src/*.cpp" "*/src/*.cpp")
add_executable(script.cgi script/main.cpp)

install(TARGETS simulator DESTINATION bin)
install(TARGETS script.cgi DESTINATION /usr/lib/cgi-bin)
