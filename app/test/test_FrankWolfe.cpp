#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#include "Log/ProgressLoggerTableOStream.hpp"
#include "Opt/GeneticIntervalSolver.hpp"
#include "Opt/GoldenSectionSolver.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/algos/ConjugateFrankWolfe.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/TAZ.hpp"
#include "test/problem/cases.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

extern string baseDir;

typedef chrono::steady_clock clk;

TEST_CASE("Frank-Wolfe", "[fw]") {
    Log::ProgressLoggerTableOStream logger;

    SECTION("Case 1") {
        auto problem = getStaticProblemTestCase1();

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*problem.first, *problem.second);

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(4.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        Opt::GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        Static::FrankWolfe fw(aon, solver, logger);
        fw.setStopCriteria(1e-3);
        Static::Solution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = (-3.0 + sqrt(53)) / 2.0;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(4.0 - x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(problem.second->getTotalDemand(), 1e-6));

        delete problem.first;
        delete problem.second;
    }

    SECTION("Case 2") {
        auto problem = getStaticProblemTestCase2();

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*problem.first, *problem.second);

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(7000.0, 1e-10));

        Opt::GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        Static::FrankWolfe fw(aon, solver, logger);
        fw.setStopCriteria(1e-3);
        Static::Solution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = 3376.36917;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(problem.second->getTotalDemand(), 1e-6));

        delete problem.first;
        delete problem.second;
    }

    SECTION("Case 3") {
        auto problem = getStaticProblemTestCase3();

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*problem.first, *problem.second);

        Opt::GoldenSectionSolver solver;
        solver.setInterval(0.0, 1.0);
        solver.setStopCriteria(1e-6);

        Static::FrankWolfe fw(aon, solver, logger);
        fw.setStopCriteria(1e-3);
        Static::Solution x = fw.solve(*problem.first, *problem.second, x0);

        double x1 = 4131.89002;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(problem.second->getTotalDemand(), 1e-6));

        delete problem.first;
        delete problem.second;
    }
}

TEST_CASE("Frank-Wolfe - large tests", "[fw][fw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    SECTION("Large") {
        // Supply
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile("data/porto/porto-armis.taz.xml");
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(102731.0 / (60 * 60), 1e-4));

        // FW
        clk::time_point begin = clk::now();

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*network, demand);
        // REQUIRE_THAT(network->evaluate(x0), WithinAbs(20795.4893081106, 1e-4));

        // Solver
        Opt::GeneticIntervalSolver solver;
        solver.setInterval(0, 1);
        solver.setStopCriteria(1e-6);

        Static::FrankWolfe fw(aon, solver, logger);

        /**
         * 1e-4 is the adequate scale because a 1s difference for one driver
         * translates to a difference in the cost function of
         * 1 veh/h * 1s = 1/3600 = 2.778e-4.
         * But this takes too much time.
         * So we are using a criteria of 0.2 for the optimal value,
         * and x for automated testing
         */
        // double epsilon = 0.2;
        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(100);

        Static::Solution x = fw.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        // REQUIRE_THAT(network->evaluate(x), WithinAbs(12332.6969610878, epsilon));

        network->saveResultsToFile(sumo, x, loader.adapter, baseDir + "data/out/edgedata-static.xml", baseDir + "data/out/routes-static.xml");
    }
}

TEST_CASE("Conjugate Frank-Wolfe - large tests", "[cfw][cfw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    SECTION("Large") {
        // Supply
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile("data/porto/porto-armis.taz.xml");
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/od/matrix.9.0.10.0.2.fma");
        Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(102731.0 / (60 * 60), 1e-4));

        // FW
        clk::time_point begin = clk::now();

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*network, demand);
        // REQUIRE_THAT(network->evaluate(x0), WithinAbs(20795.4893081106, 1e-4));

        // Solver
        Opt::GeneticIntervalSolver solver;
        solver.setInterval(0, 1);
        solver.setStopCriteria(1e-6);

        Static::ConjugateFrankWolfe fw(aon, solver, logger);

        /**
         * 1e-4 is the adequate scale because a 1s difference for one driver
         * translates to a difference in the cost function of
         * 1 veh/h * 1s = 1/3600 = 2.778e-4.
         * But this takes too much time.
         * So we are using a criteria of 0.2 for the optimal value,
         * and x for automated testing
         */
        // double epsilon = 0.2;
        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(100);

        Static::Solution x = fw.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        // REQUIRE_THAT(network->evaluate(x), WithinAbs(12332.6969610878, epsilon));

        network->saveResultsToFile(sumo, x, loader.adapter, baseDir + "data/out/edgedata-static.xml", baseDir + "data/out/routes-static.xml");
    }
}
