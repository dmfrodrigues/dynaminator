#include <unistd.h>

#include <catch2/catch_session.hpp>
#include <iostream>

std::string baseDir = "";
std::string benchmarkDir = "";

int main(int argc, char* argv[]) {
    // Setup
    using namespace Catch::Clara;

    Catch::Session session;

    // Based on https://github.com/catchorg/Catch2/blob/devel/docs/own-main.md#adding-your-own-command-line-options
    Parser cli = session.cli()
               | Opt(baseDir, "baseDir")["-d"]["--baseDir"]("Base data directory")
               | Opt(benchmarkDir, "benchmarkDir")["-b"]["--benchmarkDir"]("Benchmarks data directory");

    session.cli(cli);

    int result = session.run(argc, argv);

    // Clean-up...

    return result;
}
