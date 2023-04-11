#include <catch2/catch_session.hpp>

#include "Com/Message.hpp"
#include "Com/RunFWSimulation.hpp"

std::string baseDir = "";

int main(int argc, char* argv[]) {
    // Setup
    MESSAGE_REGISTER_MAIN(Com::RunFWSimulation);
    MESSAGE_REGISTER_MAIN(Com::RunFWSimulation::Response);

    using namespace Catch::Clara;

    Catch::Session session;

    // Based on https://github.com/catchorg/Catch2/blob/devel/docs/own-main.md#adding-your-own-command-line-options
    Parser cli = session.cli()
               | Opt(baseDir, "baseDir")
                     ["-d"]["--baseDir"]("Base directory");

    session.cli(cli);

    int result = session.run(argc, argv);

    // Clean-up...

    return result;
}
