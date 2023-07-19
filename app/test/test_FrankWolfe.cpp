#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <memory>
#include <random>

#include "Log/ProgressLoggerIgnore.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Opt/GeneticIntervalSolver.hpp"
#include "Opt/GoldenSectionSolver.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/algos/ConjugateFrankWolfe.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/algos/IterativeEquilibration.hpp"
#include "Static/supply/BPRConvexNetwork.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/Routes.hpp"
#include "data/SUMO/TAZ.hpp"
#include "test/problem/cases.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

extern string baseDir;
extern string benchmarkDir;

typedef chrono::steady_clock clk;

const double MATRIX_9_10_TOTAL_DEMAND = 102731.0 / (60 * 60);

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

const double PORTO_ARMIS_AON_COST = 442094728.844042182;

TEST_CASE("Frank-Wolfe - Large", "[fw][fw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    // Supply
    shared_ptr<SUMO::Network> sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.net.xml");
    SUMO::TAZs                sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.taz.xml");
    SUMO::NetworkTAZs         sumo{*sumoNetwork, sumoTAZs};

    // Demand
    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(benchmarkDir + "data/dynaminator-data/matrix.9.0.10.0.2.fma");

    clk::time_point begin = clk::now();

    SECTION("Convex") {
        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
        Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

        SECTION("Large") {
            Static::DijkstraAoN  aon;
            Static::SolutionBase x0 = aon.solve(*network, demand);
            REQUIRE_THAT(network->evaluate(x0), WithinAbs(18094.5701505728, 1e-4));

            Opt::QuadraticSolver      innerSolver;
            Opt::QuadraticGuessSolver solver(
                innerSolver,
                0.5,
                0.2,
                0.845,
                0.365
            );
            solver.setStopCriteria(0.01);

            Static::FrankWolfe fw(aon, solver, logger);

            double epsilon = 5.0;
            fw.setStopCriteria(epsilon);
            fw.setIterations(10000);

            Static::Solution x = fw.solve(*network, demand, x0);

            REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
            REQUIRE_THAT(network->evaluate(x), WithinAbs(12330.7051681671, epsilon));
        }

        delete network;
    }

    SECTION("Not convex") {
        Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNotConvexNetwork                           *network = loader.load(sumo);

        Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
        Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

        // FW
        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*network, demand);
        REQUIRE_THAT(network->evaluate(x0), WithinRel(PORTO_ARMIS_AON_COST, 0.005));

        SECTION("Normal FW") {
            // Solver
            Opt::GeneticIntervalSolver solver(
                8,
                8,
                0.1,
                1000,
                8,
                std::make_shared<std::mt19937>(0)
            );
            solver.setInterval(0, 1);
            solver.setStopCriteria(1e-6);

            Static::FrankWolfe fw(aon, solver, logger);

            double epsilon = 1.0;
            fw.setStopCriteria(epsilon);
            fw.setIterations(15);

            Static::Solution x = fw.solve(*network, demand, x0);

            REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
            REQUIRE_THAT(network->evaluate(x), WithinRel(5397115.9215797437, 0.005));
        }

        SECTION("Normal FW - convex approximation") {
            // Solver
            Opt::QuadraticSolver      innerSolver;
            Opt::QuadraticGuessSolver solver(
                innerSolver,
                0.5,
                0.2,
                0.845,
                0.365
            );
            solver.setStopCriteria(0.01);

            Static::FrankWolfe fw(aon, solver, logger);

            double epsilon = 1.0;
            fw.setStopCriteria(epsilon);
            fw.setIterations(25);

            Static::Solution zero;

            Static::Solution x = fw.solve(network->makeConvex(zero), demand, x0);

            REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
            REQUIRE_THAT(network->evaluate(x), WithinRel(99544739.6486251801, 0.001));
        }

        delete network;
    }

    clk::time_point end = clk::now();
    cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;
}

TEST_CASE("Conjugate Frank-Wolfe - large tests", "[cfw][cfw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    // Supply
    shared_ptr<SUMO::Network> sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.net.xml");
    SUMO::TAZs                sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.taz.xml");
    SUMO::NetworkTAZs         sumo{*sumoNetwork, sumoTAZs};

    // Demand
    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(benchmarkDir + "data/dynaminator-data/matrix.9.0.10.0.2.fma");

    clk::time_point begin = clk::now();

    SECTION("Large") {
        Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNetwork                           *network = loader.load(sumo);

        Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
        Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*network, demand);
        REQUIRE_THAT(network->evaluate(x0), WithinAbs(18094.5701505728, 1e-4));

        Opt::QuadraticSolver      innerSolver;
        Opt::QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        Static::ConjugateFrankWolfe fw(aon, solver, logger);

        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(10000);

        Static::Solution x = fw.solve(*network, demand, x0);

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinAbs(12328.3692388374, epsilon));

        delete network;
    }

    SECTION("Large not convex") {
        Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;
        Static::BPRNotConvexNetwork                           *network = loader.load(sumo);

        Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
        Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

        Static::DijkstraAoN  aon;
        Static::SolutionBase x0 = aon.solve(*network, demand);
        REQUIRE_THAT(network->evaluate(x0), WithinRel(PORTO_ARMIS_AON_COST, 0.005));

        Opt::GeneticIntervalSolver solver(
            8,
            8,
            0.1,
            1000,
            8,
            std::make_shared<std::mt19937>(0)
        );
        solver.setInterval(0, 1);
        solver.setStopCriteria(1e-6);

        Static::ConjugateFrankWolfe fw(aon, solver, logger);

        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(15);

        Static::Solution x = fw.solve(*network, demand, x0);

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinRel(5398577.8846157538, 0.005));

        delete network;
    }

    clk::time_point end = clk::now();
    cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;
}

TEST_CASE("Iterative equilibration", "[ie][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;

    // Supply
    shared_ptr<SUMO::Network>    sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.net.xml");
    SUMO::TAZs                   sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto-armis.taz.xml");
    SUMO::NetworkTAZs            sumo{*sumoNetwork, sumoTAZs};
    Static::BPRNotConvexNetwork *network = loader.load(sumo);

    // Demand
    VISUM::OFormatDemand                                                              oDemand = VISUM::OFormatDemand::loadFromFile(benchmarkDir + "data/dynaminator-data/matrix.9.0.10.0.2.fma");
    Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
    Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

    double totalDemand = demand.getTotalDemand();
    REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

    clk::time_point begin = clk::now();

    Static::DijkstraAoN  aon;
    Static::SolutionBase x0 = aon.solve(*network, demand);
    REQUIRE_THAT(network->evaluate(x0), WithinRel(PORTO_ARMIS_AON_COST, 0.005));

    SECTION("FW") {
        Opt::QuadraticSolver      innerSolver;
        Opt::QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        Static::FrankWolfe fw(aon, solver, Log::ProgressLoggerIgnore::INSTANCE);

        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(3);

        Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::FrankWolfe> ie(fw, logger);
        ie.setIterations(15);

        Static::Solution x = ie.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinRel(5381438.867287376, 0.005));
    }

    SECTION("CFW") {
        Opt::QuadraticSolver      innerSolver;
        Opt::QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        Static::ConjugateFrankWolfe fw(aon, solver, Log::ProgressLoggerIgnore::INSTANCE);

        fw.setStopCriteria(0.0);
        fw.setIterations(5);

        Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::ConjugateFrankWolfe> ie(fw, logger);
        ie.setIterations(10);

        Static::Solution x = ie.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinRel(5380740.3194306595, 0.005));
    }

    delete network;
}

TEST_CASE("Iterative equilibration - Fixed map", "[ie-fixed][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;

    // Supply
    shared_ptr<SUMO::Network>    sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.net.xml");
    SUMO::TAZs                   sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.taz.xml");
    SUMO::NetworkTAZs            sumo{*sumoNetwork, sumoTAZs};
    Static::BPRNotConvexNetwork *network = loader.load(sumo);

    // Demand
    VISUM::OFormatDemand                                                              oDemand = VISUM::OFormatDemand::loadFromFile(benchmarkDir + "data/dynaminator-data/matrix.9.0.10.0.2.fma");
    Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
    Static::Demand                                                                    demand = demandLoader.load(oDemand, loader.adapter);

    double totalDemand = demand.getTotalDemand();
    REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

    clk::time_point begin = clk::now();

    Static::DijkstraAoN  aon;
    Static::SolutionBase x0 = aon.solve(*network, demand);
    REQUIRE_THAT(network->evaluate(x0), WithinRel(115488.4860793933, 0.1));

    // SECTION("FW") {
    //     Opt::QuadraticSolver      innerSolver;
    //     Opt::QuadraticGuessSolver solver(
    //         innerSolver,
    //         0.5,
    //         0.2,
    //         0.845,
    //         0.365
    //     );
    //     solver.setStopCriteria(0.01);

    //     Static::FrankWolfe fw(aon, solver, loggerNull);

    //     double epsilon = 1.0;
    //     fw.setStopCriteria(epsilon);
    //     fw.setIterations(3);

    //     Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::FrankWolfe> ie(fw, logger);
    //     ie.setIterations(15);

    //     Static::Solution x = ie.solve(*network, demand, x0);

    //     clk::time_point end = clk::now();
    //     cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;

    //     REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
    //     REQUIRE_THAT(network->evaluate(x), WithinRel(5381438.867287376, 0.005));
    // }

    SECTION("CFW") {
        Opt::QuadraticSolver      innerSolver;
        Opt::QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        Static::ConjugateFrankWolfe fw(aon, solver, Log::ProgressLoggerIgnore::INSTANCE);

        fw.setStopCriteria(0.0);
        fw.setIterations(5);

        Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::ConjugateFrankWolfe> ie(fw, logger);
        ie.setIterations(25);

        Static::Solution x = ie.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << " [s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinRel(13372.6692234558, 0.1));

        // clang-format off
        SUMO::EdgeData::Loader<
            const SUMO::NetworkTAZs &,
            const Static::BPRNetwork &,
            const Static::Solution &,
            const Static::SUMOAdapter &
        > edgeDataLoader;
        // clang-format on
        SUMO::EdgeData edgeData = edgeDataLoader.load(sumo, *network, x, loader.adapter);
        edgeData.saveToFile(baseDir + "data/out/edgedata-static.xml");

        // clang-format off
        SUMO::Routes::Loader<
            const Static::Network &,
            const Static::Solution &,
            const Static::SUMOAdapter &
        > routesLoader;
        // clang-format on
        SUMO::Routes routes = routesLoader.load(*network, x, loader.adapter);
        routes.saveToFile(baseDir + "data/out/routes-static.xml");
    }

    delete network;
}
