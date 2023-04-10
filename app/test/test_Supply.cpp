#include <algorithm>
#include <catch2/catch_test_macros.hpp>

#include "Static/Solution.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "test/problem/cases.hpp"

using namespace std;

TEST_CASE("Supply", "[supply][customstaticnetwork]") {
    Static::Network *network = getStaticNetworkTestCase1();

    SECTION("Get nodes") {
        vector<Static::Network::Node> nodes = network->getNodes();
        sort(nodes.begin(), nodes.end());
        REQUIRE(vector<Static::Network::Node>{1, 2, 3} == nodes);
    }

    SECTION("Convert to graph") {
        Static::SolutionBase xn;
        Graph G = network->toGraph(xn);
    }

    delete network;
}
