#include <catch2/catch_session.hpp>

#include <iostream>

std::string baseDir = "";

int main(int argc, char* argv[]) {
    // Setup
    using namespace Catch::Clara;

    Catch::Session session;

    // Based on https://github.com/catchorg/Catch2/blob/devel/docs/own-main.md#adding-your-own-command-line-options
    Parser cli = session.cli()
               | Opt(baseDir, "baseDir")
                     ["-d"]["--baseDir"]("Base directory");

    session.cli(cli);

    std::cerr << "baseDir: " << baseDir << std::endl;

    int result = session.run(argc, argv);

    // Clean-up...

    return result;
}
