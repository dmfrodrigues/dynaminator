#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <memory>

#include "Alg/ShortestPath/Dijkstra.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/TAZ.hpp"
#include "test/problem/graphs.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

extern string baseDir;

void testPath(std::vector<Alg::Graph::Node> expected, Alg::Graph::Path got) {
    if(expected.size() == 0) {
        REQUIRE(1 == got.size());
        REQUIRE(-1 == got.front().id);
        REQUIRE(Alg::Graph::NODE_INVALID == got.front().u);
        REQUIRE(Alg::Graph::NODE_INVALID == got.front().v);
        REQUIRE_THAT(got.front().w, WithinAbs(0, 1e-10));

        return;
    }
    REQUIRE(got.size() == expected.size() - 1);
    auto itExpected = expected.begin();
    auto itGot      = got.begin();
    for(; itGot != got.end(); ++itExpected, ++itGot) {
        auto itExpectedNext = itExpected;
        ++itExpectedNext;
        REQUIRE(*itExpected == itGot->u);
        REQUIRE(*itExpectedNext == itGot->v);
    }
}

TEST_CASE("Dijkstra's algorithm", "[shortestpath][shortestpath-onemany][dijkstra]") {
    SECTION("Start 0") {
        Alg::Graph G = graph1();

        Alg::ShortestPath::ShortestPathOneMany *shortestPath = new Alg::ShortestPath::Dijkstra();
        shortestPath->solve(G, 0);

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
        Alg::Graph G = graph1();

        Alg::ShortestPath::ShortestPathOneMany *shortestPath = new Alg::ShortestPath::Dijkstra();
        shortestPath->solve(G, 1);

        testPath({}, shortestPath->getPath(0));
        testPath({1}, shortestPath->getPath(1));
        testPath({1, 2}, shortestPath->getPath(2));
        testPath({1, 2, 3}, shortestPath->getPath(3));
        testPath({1, 2, 3, 4}, shortestPath->getPath(4));
        testPath({1, 2, 5}, shortestPath->getPath(5));
        testPath({1, 2, 5, 6}, shortestPath->getPath(6));

        REQUIRE_THAT(shortestPath->getPathWeight(0), WithinAbs(Alg::Graph::Edge::WEIGHT_INF, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(1), WithinAbs(0, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(2), WithinAbs(2, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(3), WithinAbs(3, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(4), WithinAbs(5, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(5), WithinAbs(4, 1e-10));
        REQUIRE_THAT(shortestPath->getPathWeight(6), WithinAbs(8, 1e-10));

        delete shortestPath;
    }

    SECTION("crossroads1") {
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/crossroads1/crossroads1.net.xml");
        SUMO::TAZs        sumoTAZs;
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        // Demand
        Static::Demand demand;

        Static::SolutionBase                               xn;
        Alg::Graph                                         G = network->toGraph(xn);
        unique_ptr<Alg::ShortestPath::ShortestPathOneMany> sp(new Alg::ShortestPath::Dijkstra());
        sp.get()->solve(G, loader.adapter.toNodes("2").first);

        const double RIGHT_TURN = 10;
        const double LEFT_TURN = 20;

        const double v1 = 13.89, l1 = 14.07;
        const double v2 = 8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 = 8.33, l4 = 39.34;
        const double t1 = l1 / (v1 * 0.9);
        const double t2 = l2 / (v2 * 0.9);
        const double t3 = l3 / (v3 * 0.9);
        const double t4 = l4 / (v4 * 0.9);

        const double r1 = 60.0;
        const double r2 = 30.0;
        const double C = 90.0;

        const double t21 = r1 * r1 / (2.0 * C);
        const double t23 = r1 * r1 / (2.0 * C);
        const double t24 = r1 * r1 / (2.0 * C);


        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("2").first), WithinAbs(0, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("2").second), WithinAbs(t2, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-1").first), WithinAbs(t2 + t21 + RIGHT_TURN, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-1").second), WithinAbs(t2 + t21 + RIGHT_TURN + t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-4").first), WithinAbs(t2 + t24, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-4").second), WithinAbs(t2 + t24 + t4, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-3").first), WithinAbs(t2 + t23 + LEFT_TURN, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-3").second), WithinAbs(t2 + t23 + LEFT_TURN + t3, 1e-6));

        delete network;
    }

    SECTION("crossroads2") {
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/crossroads2/crossroads2.net.xml");
        SUMO::TAZs        sumoTAZs;
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        // Demand
        Static::Demand demand;

        Static::SolutionBase                               xn;
        Alg::Graph                                         G = network->toGraph(xn);
        unique_ptr<Alg::ShortestPath::ShortestPathOneMany> sp(new Alg::ShortestPath::Dijkstra());
        sp.get()->solve(G, loader.adapter.toNodes("2").first);

        const double RIGHT_TURN = 10;
        const double LEFT_TURN = 20;
        const double TURN_AROUND = 20;

        const double v1 = 13.89, l1 = 14.07;
        const double v2 = 8.33, l2 = 18.80;
        const double v3 = 13.89, l3 = 33.24;
        const double v4 = 8.33, l4 = 39.34;
        const double t1 = l1 / (v1 * 0.9);
        const double t2 = l2 / (v2 * 0.9);
        const double t3 = l3 / (v3 * 0.9);
        const double t4 = l4 / (v4 * 0.9);

        const double r1 = 60.0;
        const double r2 = 30.0;
        const double C = 90.0;

        const double t21 = r1 * r1 / (2.0 * C);
        const double t23 = r1 * r1 / (2.0 * C);
        const double t24 = r1 * r1 / (2.0 * C);

        const double t13 = r2 * r2 / (2.0 * C);

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("2").first), WithinAbs(0, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("2").second), WithinAbs(t2, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-1").first), WithinAbs(t2 + t21 + RIGHT_TURN, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-1").second), WithinAbs(t2 + t21 + RIGHT_TURN + t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("1").first), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("1").second), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND + t1, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-3").first), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND + t1 + t13, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-3").second), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND + t1 + t13 + t3, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("3").first), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND + t1 + t13 + t3 + TURN_AROUND, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("3").second), WithinAbs(t2 + t21 + RIGHT_TURN + t1 + TURN_AROUND + t1 + t13 + t3 + TURN_AROUND + t3, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-4").first), WithinAbs(t2 + t24, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("-4").second), WithinAbs(t2 + t24 + t4, 1e-6));

        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("4").first), WithinAbs(t2 + t24 + t4 + TURN_AROUND, 1e-6));
        REQUIRE_THAT(sp.get()->getPathWeight(loader.adapter.toNodes("4").second), WithinAbs(t2 + t24 + t4 + TURN_AROUND + t4, 1e-6));

        delete network;
    }

    SECTION("Large") {
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(baseDir + "data/porto/porto-armis.taz.xml");
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

        Static::SolutionBase                               xn;
        Alg::Graph                                         G = network->toGraph(xn);
        unique_ptr<Alg::ShortestPath::ShortestPathOneMany> sp(new Alg::ShortestPath::Dijkstra());
        sp.get()->solve(G, 4455);

        REQUIRE(sp.get()->getPrev(2952).u != -1);
        REQUIRE(sp.get()->getPrev(4252).u != -1);

        delete network;
    }
}
