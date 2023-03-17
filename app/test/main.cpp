#include <catch2/catch_session.hpp>

#include "network/Message.hpp"
#include "network/RunFWSimulation.hpp"

int main(int argc, char* argv[]) {
    // Setup
    MESSAGE_REGISTER_MAIN(RunFWSimulation);
    MESSAGE_REGISTER_MAIN(RunFWSimulation::Response);

    int result = Catch::Session().run( argc, argv );

    // Clean-up...

    return result;
}
