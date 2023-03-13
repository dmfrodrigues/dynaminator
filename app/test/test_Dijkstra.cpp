#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>

#include "data/SumoTAZs.hpp"
#include "shortest-path/Dijkstra.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/BPRNetwork.hpp"

using namespace std;
using Catch::Approx;

const long EDGE_ID_IRRELEVANT = -1;

void testPath(std::vector<Graph::Node> expected, Graph::Path got) {
    if (expected.size() == 0) {
        REQUIRE(1 == got.size());
        REQUIRE(-1 == got.front().id);
        REQUIRE(Graph::NODE_INVALID == got.front().u);
        REQUIRE(Graph::NODE_INVALID == got.front().v);
        REQUIRE(Approx(0).margin(1e-10) == got.front().w);

        return;
    }
    REQUIRE(got.size() == expected.size() - 1);
    auto itExpected = expected.begin();
    auto itGot = got.begin();
    for (; itExpected != expected.end(); ++itExpected, ++itGot) {
        auto itExpectedNext = itExpected;
        ++itExpectedNext;
        REQUIRE(*itExpected == itGot->u);
        REQUIRE(*itExpectedNext == itGot->v);
    }
}

TEST_CASE("Dijkstra's algorithm", "[shortestpath][shortestpath-onemany][dijkstra]") {
    Graph G;
    for (int i = 0; i < 7; ++i) G.addNode(i);
    G.addEdge(EDGE_ID_IRRELEVANT, 0, 1, 1);
    G.addEdge(EDGE_ID_IRRELEVANT, 1, 2, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 0, 3, 5);
    G.addEdge(EDGE_ID_IRRELEVANT, 3, 4, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 2, 3, 1);
    G.addEdge(EDGE_ID_IRRELEVANT, 2, 5, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 4, 5, 3);
    G.addEdge(EDGE_ID_IRRELEVANT, 5, 6, 4);

    SECTION("Start 0") {
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->initialize(&G, 0);
        shortestPath->run();

        testPath({0}, shortestPath->getPath(0));
        testPath({0, 1}, shortestPath->getPath(1));
        testPath({0, 1, 2}, shortestPath->getPath(2));
        testPath({0, 1, 2, 3}, shortestPath->getPath(3));
        testPath({0, 1, 2, 3, 4}, shortestPath->getPath(4));
        testPath({0, 1, 2, 5}, shortestPath->getPath(5));
        testPath({0, 1, 2, 5, 6}, shortestPath->getPath(6));

        REQUIRE(Approx(0).margin(1e-10) == shortestPath->getPathWeight(0));
        REQUIRE(Approx(1).margin(1e-10) == shortestPath->getPathWeight(1));
        REQUIRE(Approx(3).margin(1e-10) == shortestPath->getPathWeight(2));
        REQUIRE(Approx(4).margin(1e-10) == shortestPath->getPathWeight(3));
        REQUIRE(Approx(6).margin(1e-10) == shortestPath->getPathWeight(4));
        REQUIRE(Approx(5).margin(1e-10) == shortestPath->getPathWeight(5));
        REQUIRE(Approx(9).margin(1e-10) == shortestPath->getPathWeight(6));

        delete shortestPath;
    }
    SECTION("Start 1") {
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->initialize(&G, 1);
        shortestPath->run();

        testPath({}, shortestPath->getPath(0));
        testPath({1}, shortestPath->getPath(1));
        testPath({1, 2}, shortestPath->getPath(2));
        testPath({1, 2, 3}, shortestPath->getPath(3));
        testPath({1, 2, 3, 4}, shortestPath->getPath(4));
        testPath({1, 2, 5}, shortestPath->getPath(5));
        testPath({1, 2, 5, 6}, shortestPath->getPath(6));

        REQUIRE(Approx(Graph::Edge::WEIGHT_INF).margin(1e-10) == shortestPath->getPathWeight(0));
        REQUIRE(Approx(0).margin(1e-10) == shortestPath->getPathWeight(1));
        REQUIRE(Approx(2).margin(1e-10) == shortestPath->getPathWeight(2));
        REQUIRE(Approx(3).margin(1e-10) == shortestPath->getPathWeight(3));
        REQUIRE(Approx(5).margin(1e-10) == shortestPath->getPathWeight(4));
        REQUIRE(Approx(4).margin(1e-10) == shortestPath->getPathWeight(5));
        REQUIRE(Approx(8).margin(1e-10) == shortestPath->getPathWeight(6));

        delete shortestPath;
    }
}

TEST_CASE("Dijkstra - large network", "[dijkstra-large]") {
    SumoNetwork sumoNetwork = SumoNetwork::loadFromFile("data/network/net.net.xml");
    SumoTAZs sumoTAZs = SumoTAZs::loadFromFile("data/network/taz.xml");
    auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
    StaticNetwork *network = get<0>(t);
    const auto &str2id = get<1>(t);
    const auto &str2id_taz = get<2>(t);

    // Demand
    OFormatDemand oDemand = OFormatDemand::loadFromFile("data/od/matrix.8.0.9.0.1.fma");
    StaticDemand demand = StaticDemand::fromOFormat(oDemand, str2id_taz);

    StaticSolution xn;
    Graph G = network->toGraph(xn);
    unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
    sp.get()->initialize(&G, 4456);
    sp.get()->run();

    REQUIRE(sp.get()->getPrev(2953).u != -1);
    REQUIRE(sp.get()->getPrev(4253).u != -1);
}
