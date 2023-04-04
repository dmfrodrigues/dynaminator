cmake_minimum_required(VERSION 3.13)

project(dynaminator LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread -lstdc++fs")

add_compile_options(
    # -g
    -Wall
    # -Weffc++
    # -Wmissing-braces
    # -Wpadded
    # -Winline
    -pedantic -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion
    -Wdisabled-optimization -Wextra -Wfloat-equal -Wformat -Wformat-nonliteral -Wformat-security
    -Wformat-y2k -Wformat=2 -Wimport -Winit-self -Winvalid-pch
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn
    -Wpacked -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wshadow
    -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default
    -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunsafe-loop-optimizations
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-result -Wunused-value
    -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings
    -lstdc++fs

    -Ofast
)

include_directories("include" "test/include" "script/include" "libs/rapidxml" "libs/CTPL" "libs/http-status-codes-cpp")

# ################ SIMULATOR ################
file(GLOB_RECURSE SRC "src/*.cpp" "*/src/*.cpp")
add_executable(dynaminator main.cpp ${SRC})

# ################ TESTS ################
file(GLOB_RECURSE TESTS_SRC "test/main.cpp" "test/test_*.cpp" "test/src/*.cpp" "test/src/*/*.cpp")
find_package(Catch2 REQUIRED)
add_executable(tests ${TESTS_SRC} ${SRC})
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain stdc++fs)
include(CTest)
include(Catch)
catch_discover_tests(tests)

# ################ SCRIPT ################
find_package(nlohmann_json 3.2.0 REQUIRED)
file(GLOB_RECURSE SCRIPT_SRC "script/src/*.cpp")
add_executable(script.cgi script/main.cpp ${SCRIPT_SRC} ${SRC})
target_link_libraries(script.cgi PRIVATE nlohmann_json::nlohmann_json)

# INSTALL
install(TARGETS dynaminator DESTINATION bin)
install(TARGETS script.cgi DESTINATION /usr/lib/cgi-bin)
