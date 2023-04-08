#include <catch2/catch_test_macros.hpp>

#include "data/sumo/Network.hpp"

using namespace std;

extern string baseDir;

TEST_CASE("Sumo network - get stats", "[sumonetwork][sumonetwork-stats]") {
    SUMO::Network network = SUMO::Network::loadFromFile(baseDir + "data/network/net.net.xml");

    network.saveStatsToFile(baseDir + "data/out/edgedata-original.xml");
}
