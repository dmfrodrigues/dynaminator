#include <algorithm>
#include <catch2/catch_test_macros.hpp>

#include "Static/Solution.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
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
        Alg::Graph G = network->toGraph(xn);
    }

    delete network;
}
