#include <catch2/catch_test_macros.hpp>

#include "test/problem/cases.hpp"

#include "static/StaticSolution.hpp"

#include <algorithm>

using namespace std;

TEST_CASE("Supply", "[supply][customstaticnetwork]"){
    StaticNetwork *network = getStaticNetworkTestCase1();

    SECTION("Get nodes"){
        vector<StaticNetwork::Node> nodes = network->getNodes();
        sort(nodes.begin(), nodes.end());
        REQUIRE(vector<StaticNetwork::Node>{1, 2, 3} == nodes);
    }

    SECTION("Convert to graph"){
        StaticSolution xn;
        Graph G = network->toGraph(xn);
    }

    delete network;
}
