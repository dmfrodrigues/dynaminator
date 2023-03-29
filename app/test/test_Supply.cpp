#include <algorithm>
#include <catch2/catch_test_macros.hpp>

#include "static/StaticSolution.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "test/problem/cases.hpp"
#include "test/utils.hpp"

using namespace std;

TEST_CASE("Supply", "[supply][customstaticnetwork]") {
    StaticNetwork *network = getStaticNetworkTestCase1();

    SECTION("Get nodes") {
        vector<StaticNetwork::Node> nodes = network->getNodes();
        sort(nodes.begin(), nodes.end());
        REQUIRE(vector<StaticNetwork::Node>{1, 2, 3} == nodes);
    }

    SECTION("Convert to graph") {
        StaticSolution xn;
        Graph G = network->toGraph(xn);
    }

    delete network;
}

TEST_CASE("BPR", "[bpr]") {
    filesystem::path exePath = getExePath();
    filesystem::path basePath = exePath.parent_path().parent_path();

    SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(basePath.string() + "/data/network/crossroads1/crossroads1.net.xml");
    SumoTAZs sumoTAZs;
    auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
    BPRNetwork *network = get<0>(t);
    const SumoAdapterStatic &adapter = get<1>(t);

    delete network;
}
