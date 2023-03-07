#include <catch2/catch_session.hpp>

#include "network/Message.hpp"

int main(int argc, char* argv[]) {
    // Setup

    int result = Catch::Session().run( argc, argv );

    // Clean-up...

    return result;
}
