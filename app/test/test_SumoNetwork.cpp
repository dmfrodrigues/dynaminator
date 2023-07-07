#include <catch2/catch_test_macros.hpp>

#include "data/SUMO/Network.hpp"

using namespace std;

extern string baseDir;
extern string benchmarkDir;

TEST_CASE("Sumo network - get stats", "[sumonetwork][sumonetwork-stats]") {
    shared_ptr<SUMO::Network> network = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.net.xml");

    network->saveStatsToFile(baseDir + "data/out/edgedata-original.xml");
}
