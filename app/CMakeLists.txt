cmake_minimum_required(VERSION 3.13)

project(dynaminator)

add_compile_definitions(ASIO_STANDALONE)

if(COVERAGE)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/libs/cmake-modules")
    include(CodeCoverage)
    append_coverage_compiler_flags()
    setup_target_for_coverage_lcov(NAME coverage EXECUTABLE tests --baseDir ../ EXCLUDE "/usr/*")
endif()

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
    # -g
    -Wall
    -pedantic -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion
    -Wdisabled-optimization -Wextra -Wfloat-equal -Wformat -Wformat-nonliteral -Wformat-security
    -Wformat-y2k -Wformat=2 -Wimport -Winit-self -Winvalid-pch
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn
    -Wpacked -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wshadow
    -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default
    -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunsafe-loop-optimizations
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-result -Wunused-value
    -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings
    -Ofast
    # -Weffc++
    # -Wmissing-braces
    # -Wpadded
    # -Winline
)

include_directories("include" "test/include" "libs/rapidxml" "libs/CTPL" "libs/http-status-codes-cpp" "libs/color/src" "libs/cpp-httplib")

find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# ################ SIMULATOR ################
file(GLOB_RECURSE SRC "src/*.cpp" "*/src/*.cpp")
add_executable(dynaminator main.cpp ${SRC})

# ################ TESTS ################
if (NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    file(GLOB_RECURSE TESTS_SRC "test/main.cpp" "test/test_*.cpp" "test/src/*.cpp" "test/src/*/*.cpp")
    find_package(Catch2 REQUIRED)
    add_executable(tests ${TESTS_SRC} ${SRC})
    target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
    include(CTest)
    include(Catch)
    catch_discover_tests(tests)
endif()

# INSTALL
install(TARGETS dynaminator DESTINATION bin)
