#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>

#include "data/sumo/TAZs.hpp"
#include "shortest-path/Dijkstra.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "test/utils.hpp"

using namespace std;
using Catch::Approx;

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
        REQUIRE(Approx(0).margin(1e-10) == got.front().w);

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
        Graph G = graph1();

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

    SECTION("crossroads1"){
        filesystem::path exePath = getExePath();
        filesystem::path basePath = exePath.parent_path().parent_path();

        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(basePath.string() + "/data/network/crossroads1/crossroads1.net.xml");
        SumoTAZs sumoTAZs;
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        StaticDemand demand;

        StaticSolution xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->initialize(&G, adapter.toNodes("2").first);
        sp.get()->run();

        const double v1 = 13.89, l1 = 14.07;
        const double v2 =  8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 =  8.33, l4 = 39.34;
        const double t1 = l1/(v1*0.9);
        const double t2 = l2/(v2*0.9);
        const double t3 = l3/(v3*0.9);
        const double t4 = l4/(v4*0.9);

        REQUIRE(Approx(0).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("2").first));
        REQUIRE(Approx(t2).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("2").second));

        REQUIRE(Approx(t2+10).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-1").first));
        REQUIRE(Approx(t2+10+t1).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-1").second));

        REQUIRE(Approx(t2).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-4").first));
        REQUIRE(Approx(t2+t4).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-4").second));

        REQUIRE(Approx(t2+20).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-3").first));
        REQUIRE(Approx(t2+20+t3).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-3").second));

        delete network;
    }

    SECTION("crossroads2"){
        filesystem::path exePath = getExePath();
        filesystem::path basePath = exePath.parent_path().parent_path();

        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(basePath.string() + "/data/network/crossroads2/crossroads2.net.xml");
        SumoTAZs sumoTAZs;
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        StaticDemand demand;

        StaticSolution xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->initialize(&G, adapter.toNodes("2").first);
        sp.get()->run();

        const double v1 = 13.89, l1 = 14.07;
        const double v2 =  8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 =  8.33, l4 = 39.34;
        const double t1 = l1/(v1*0.9);
        const double t2 = l2/(v2*0.9);
        const double t3 = l3/(v3*0.9);
        const double t4 = l4/(v4*0.9);

        REQUIRE(Approx(0).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("2").first));
        REQUIRE(Approx(t2).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("2").second));

        REQUIRE(Approx(t2+10).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-1").first));
        REQUIRE(Approx(t2+10+t1).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-1").second));

        REQUIRE(Approx(t2+10+t1+20).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("1").first));
        REQUIRE(Approx(t2+10+t1+20+t1).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("1").second));

        REQUIRE(Approx(t2+10+t1+20+t1).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-3").first));
        REQUIRE(Approx(t2+10+t1+20+t1+t3).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-3").second));

        REQUIRE(Approx(t2+10+t1+20+t1+t3+20).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("3").first));
        REQUIRE(Approx(t2+10+t1+20+t1+t3+20+t3).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("3").second));

        REQUIRE(Approx(t2).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-4").first));
        REQUIRE(Approx(t2+t4).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("-4").second));

        REQUIRE(Approx(t2+t4+20).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("4").first));
        REQUIRE(Approx(t2+t4+20+t4).margin(1e-6) == sp.get()->getPathWeight(adapter.toNodes("4").second));

        delete network;
    }

    SECTION("Large"){
        filesystem::path exePath = getExePath();
        filesystem::path basePath = exePath.parent_path().parent_path();

        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(basePath.string() + "/data/network/net.net.xml");
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(basePath.string() + "/data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        StaticNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(basePath.string() + "/data/od/matrix.9.0.10.0.2.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        StaticSolution xn;
        Graph G = network->toGraph(xn);
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->initialize(&G, 4455);
        sp.get()->run();

        REQUIRE(sp.get()->getPrev(2952).u != -1);
        REQUIRE(sp.get()->getPrev(4252).u != -1);

        delete network;
    }
}
