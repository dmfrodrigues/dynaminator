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
