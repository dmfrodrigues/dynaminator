#include <catch2/catch_test_macros.hpp>

#include "data/SumoNetwork.hpp"
#include "test/utils.hpp"

using namespace std;

TEST_CASE("Sumo network - get stats", "[sumonetwork][sumonetwork-stats]") {
    filesystem::path exePath = getExePath();
    filesystem::path basePath = exePath.parent_path().parent_path();

    SumoNetwork network = SumoNetwork::loadFromFile(basePath.string() + "/data/network/net.net.xml");

    network.saveStatsToFile(basePath.string() + "/data/out/edgedata-original.xml");
}
