#include <catch2/catch_test_macros.hpp>

#include "data/sumo/Network.hpp"
#include "test/utils.hpp"

using namespace std;

TEST_CASE("Sumo network - get stats", "[sumonetwork][sumonetwork-stats]") {
    filesystem::path exePath = getExePath();
    filesystem::path basePath = exePath.parent_path().parent_path();

    SUMO::Network network = SUMO::Network::loadFromFile(basePath.string() + "/data/network/net.net.xml");

    network.saveStatsToFile(basePath.string() + "/data/out/edgedata-original.xml");
}
