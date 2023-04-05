#include <catch2/catch_test_macros.hpp>

#include "data/sumo/Network.hpp"

using namespace std;

TEST_CASE("Sumo network - get stats", "[sumonetwork][sumonetwork-stats]") {
    SUMO::Network network = SUMO::Network::loadFromFile("data/network/net.net.xml");

    network.saveStatsToFile("data/out/edgedata-original.xml");
}
