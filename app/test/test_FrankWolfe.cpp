#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#include "data/sumo/TAZ.hpp"
#include "opt/GoldenSectionSolver.hpp"
#include "opt/QuadraticGuessSolver.hpp"
#include "opt/QuadraticSolver.hpp"
#include "static/algos/ConjugateFrankWolfe.hpp"
#include "static/algos/DijkstraAoN.hpp"
#include "static/algos/FrankWolfe.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "test/problem/cases.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

extern string baseDir;

typedef chrono::steady_clock clk;

TEST_CASE("Frank-Wolfe", "[fw]") {
    SECTION("Case 1") {
        auto problem = getStaticProblemTestCase1();

        DijkstraAoN aon;
        StaticSolutionBase x0 = aon.solve(*problem.first, *problem.second);

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(4.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        FrankWolfe fw(aon, solver);
        fw.setStopCriteria(1e-3);
        StaticSolution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = (-3.0 + sqrt(53)) / 2.0;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(4.0 - x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        delete problem.first;
        delete problem.second;
    }

    SECTION("Case 2") {
        auto problem = getStaticProblemTestCase2();

        DijkstraAoN aon;
        StaticSolutionBase x0 = aon.solve(*problem.first, *problem.second);

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(7000.0, 1e-10));

        GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        FrankWolfe fw(aon, solver);
        fw.setStopCriteria(1e-3);
        StaticSolution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = 3376.36917;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        delete problem.first;
        delete problem.second;
    }

    SECTION("Case 3") {
        auto problem = getStaticProblemTestCase3();

        DijkstraAoN aon;
        StaticSolutionBase x0 = aon.solve(*problem.first, *problem.second);

        GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        FrankWolfe fw(aon, solver);
        fw.setStopCriteria(1e-3);
        StaticSolution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = 4131.89002;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        delete problem.first;
        delete problem.second;
    }
}

TEST_CASE("Frank-Wolfe - large tests", "[fw][fw-large][!benchmark]") {
    SECTION("Large") {
        // Supply
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/net.net.xml");
        SUMO::TAZs sumoTAZs = SUMO::TAZ::loadFromFile("data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(102731.0 / (60 * 60), 1e-4));

        // FW
        clk::time_point begin = clk::now();

        DijkstraAoN aon;
        StaticSolutionBase x0 = aon.solve(*network, demand);
        REQUIRE_THAT(network->evaluate(x0), WithinAbs(13662.6299061352, 1e-4));

        // Solver
        QuadraticSolver innerSolver;
        QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        /**
         * 1e-4 is the adequate scale because a 1s difference for one driver
         * translates to a difference in the cost function of
         * 1 veh/h * 1s = 1/3600 = 2.778e-4.
         * But this takes too much time.
         * So we are using a criteria of 0.2 for the optimal value,
         * and x for automated testing
         */
        // double epsilon = 0.2;
        FrankWolfe fw(aon, solver);
        double epsilon = 2.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(10000);

        StaticSolution x = fw.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(network->evaluate(x), WithinAbs(12110.1838409 , epsilon));

        network->saveResultsToFile(x, adapter, baseDir + "data/out/edgedata-static.xml", baseDir + "data/out/routes-static.xml");
    }
}

TEST_CASE("Conjugate Frank-Wolfe - large tests", "[cfw][cfw-large][!benchmark]") {
    SECTION("Large") {
        // Supply
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/network/net.net.xml");
        SUMO::TAZs sumoTAZs = SUMO::TAZ::loadFromFile("data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(102731.0 / (60 * 60), 1e-4));

        // FW
        clk::time_point begin = clk::now();

        DijkstraAoN aon;
        StaticSolutionBase x0 = aon.solve(*network, demand);
        REQUIRE_THAT(network->evaluate(x0), WithinAbs(13662.6299061352, 1e-4));

        // Solver
        QuadraticSolver innerSolver;
        QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        /**
         * 1e-4 is the adequate scale because a 1s difference for one driver
         * translates to a difference in the cost function of
         * 1 veh/h * 1s = 1/3600 = 2.778e-4.
         * But this takes too much time.
         * So we are using a criteria of 0.2 for the optimal value,
         * and x for automated testing
         */
        // double epsilon = 0.2;
        ConjugateFrankWolfe fw(aon, solver);
        double epsilon = 2.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(10000);

        StaticSolution x = fw.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(network->evaluate(x), WithinAbs(12110.1838409 , epsilon));

        network->saveResultsToFile(x, adapter,
            baseDir + "data/out/edgedata-static.xml",
            baseDir + "data/out/routes-static.xml"
        );
    }
}
