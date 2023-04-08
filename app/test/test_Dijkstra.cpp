#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>

#include "data/sumo/TAZs.hpp"
#include "shortest-path/Dijkstra.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/BPRNetwork.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

extern string baseDir;

const long EDGE_ID_IRRELEVANT = -1;

Graph graph1(){
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
    return G;
}

void testPath(std::vector<Graph::Node> expected, Graph::Path got) {
    if (expected.size() == 0) {
        REQUIRE(1 == got.size());
        REQUIRE(-1 == got.front().id);
        REQUIRE(Graph::NODE_INVALID == got.front().u);
        REQUIRE(Graph::NODE_INVALID == got.front().v);
        REQUIRE_THAT(got.front().w, WithinAbs(0, 1e-10));

        return;
    }
    REQUIRE(got.size() == expected.size() - 1);
    auto itExpected = expected.begin();
    auto itGot = got.begin();
    for (; itGot != got.end(); ++itExpected, ++itGot) {
        auto itExpectedNext = itExpected;
        ++itExpectedNext;
        REQUIRE(*itExpected == itGot->u);
        REQUIRE(*itExpectedNext == itGot->v);
    }
}

TEST_CASE("Dijkstra's algorithm", "[shortestpath][shortestpath-onemany][dijkstra]") {

    SECTION("Start 0") {
        Graph G = graph1();
    
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->solve(&G, 0);

        testPath({0}, shortestPath->getPath(0));
        testPath({0, 1}, shortestPath->getPath(1));
        testPath({0, 1, 2}, shortestPath->getPath(2));
        testPath({0, 1, 2, 3}, shortestPath->getPath(3));
        testPath({0, 1, 2, 3, 4}, shortestPath->getPath(4));
        testPath({0, 1, 2, 5}, shortestPath->getPath(5));
        testPath({0, 1, 2, 5, 6}, shortestPath->getPath(6));

        REQUIRE_THAT(shortestPath->getPathWeight(0), WithinAbs(0, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(1), WithinAbs(1, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(2), WithinAbs(3, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(3), WithinAbs(4, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(4), WithinAbs(6, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(5), WithinAbs(5, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(6), WithinAbs(9, 1e-10));

        delete shortestPath;
    }
    SECTION("Start 1") {
        Graph G = graph1();

        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->solve(&G, 1);

        testPath({}, shortestPath->getPath(0));
        testPath({1}, shortestPath->getPath(1));
        testPath({1, 2}, shortestPath->getPath(2));
        testPath({1, 2, 3}, shortestPath->getPath(3));
        testPath({1, 2, 3, 4}, shortestPath->getPath(4));
        testPath({1, 2, 5}, shortestPath->getPath(5));
        testPath({1, 2, 5, 6}, shortestPath->getPath(6));

        REQUIRE_THAT(shortestPath->getPathWeight(0), WithinAbs(Graph::Edge::WEIGHT_INF, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(1), WithinAbs(0, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(2), WithinAbs(2, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(3), WithinAbs(3, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(4), WithinAbs(5, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(5), WithinAbs(4, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(6), WithinAbs(8, 1e-10));

        delete shortestPath;
    }

    SECTION("crossroads1") {
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/crossroads1/crossroads1.net.xml");
        SumoTAZs sumoTAZs;
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        StaticDemand demand;

        StaticSolutionBase xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->solve(&G, adapter.toNodes("2").first);

        const double v1 = 13.89, l1 = 14.07;
        const double v2 =  8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 =  8.33, l4 = 39.34;
        const double t1 = l1/(v1*0.9);
        const double t2 = l2/(v2*0.9);
        const double t3 = l3/(v3*0.9);
        const double t4 = l4/(v4*0.9);

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("2").first), WithinAbs(0, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("2").second), WithinAbs(t2, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-1").first), WithinAbs(t2+10, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-1").second), WithinAbs(t2+10+t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-4").first), WithinAbs(t2, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-4").second), WithinAbs(t2+t4, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-3").first), WithinAbs(t2+20, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-3").second), WithinAbs(t2+20+t3, 1e-6));

        delete network;
    }

    SECTION("crossroads2") {
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/crossroads2/crossroads2.net.xml");
        SumoTAZs sumoTAZs;
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        StaticDemand demand;

        StaticSolutionBase xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->solve(&G, adapter.toNodes("2").first);

        const double v1 = 13.89, l1 = 14.07;
        const double v2 =  8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 =  8.33, l4 = 39.34;
        const double t1 = l1/(v1*0.9);
        const double t2 = l2/(v2*0.9);
        const double t3 = l3/(v3*0.9);
        const double t4 = l4/(v4*0.9);

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("2").first), WithinAbs(0, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("2").second), WithinAbs(t2, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-1").first), WithinAbs(t2+10, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-1").second), WithinAbs(t2+10+t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("1").first), WithinAbs(t2+10+t1+20, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("1").second), WithinAbs(t2+10+t1+20+t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-3").first), WithinAbs(t2+10+t1+20+t1, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-3").second), WithinAbs(t2+10+t1+20+t1+t3, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("3").first), WithinAbs(t2+10+t1+20+t1+t3+20, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("3").second), WithinAbs(t2+10+t1+20+t1+t3+20+t3, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-4").first), WithinAbs(t2, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("-4").second), WithinAbs(t2+t4, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("4").first), WithinAbs(t2+t4+20, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(adapter.toNodes("4").second), WithinAbs(t2+t4+20+t4, 1e-6));

        delete network;
    }

    SECTION("Large") {
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/net.net.xml");
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(baseDir + "data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        StaticNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        StaticSolutionBase xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->solve(&G, 4455);

        REQUIRE(sp.get()->getPrev(2952).u != -1);
        REQUIRE(sp.get()->getPrev(4252).u != -1);

        delete network;
    }
}
