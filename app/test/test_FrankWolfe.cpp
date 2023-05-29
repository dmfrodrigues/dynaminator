#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <memory>

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
#include "data/SUMO/TAZ.hpp"
#include "test/problem/cases.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

extern string baseDir;

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

const double PORTO_ARMIS_AON_COST = 442094699.2112036943;

TEST_CASE("Frank-Wolfe - Large", "[fw][fw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    try {
        // Supply
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(baseDir + "data/porto/porto-armis.taz.xml");
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/porto/matrix.9.0.10.0.2.fma");

        clk::time_point begin = clk::now();

        SECTION("Convex") {
            Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;

            Static::BPRNetwork *network = loader.load(sumo);
            Static::Demand      demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

            double totalDemand = demand.getTotalDemand();
            REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

            SECTION("Large") {
                Static::DijkstraAoN  aon;
                Static::SolutionBase x0 = aon.solve(*network, demand);
                REQUIRE_THAT(network->evaluate(x0), WithinAbs(17546.6462131649, 1e-4));

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
                REQUIRE_THAT(network->evaluate(x), WithinAbs(12091.1614891103, epsilon));
            }
        }

        SECTION("Not convex") {
            Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;

            Static::BPRNotConvexNetwork *network = loader.load(sumo);
            Static::Demand               demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

            double totalDemand = demand.getTotalDemand();
            REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

            // FW
            Static::DijkstraAoN  aon;
            Static::SolutionBase x0 = aon.solve(*network, demand);
            REQUIRE_THAT(network->evaluate(x0), WithinAbs(PORTO_ARMIS_AON_COST, 1e-3));

            SECTION("Normal FW") {
                // Solver
                Opt::GeneticIntervalSolver solver;
                solver.setInterval(0, 1);
                solver.setStopCriteria(1e-6);

                Static::FrankWolfe fw(aon, solver, logger);

                double epsilon = 1.0;
                fw.setStopCriteria(epsilon);
                fw.setIterations(25);

                Static::Solution x = fw.solve(*network, demand, x0);

                REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
                REQUIRE_THAT(network->evaluate(x), WithinAbs(5397037.4691067543, epsilon));
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
                REQUIRE_THAT(network->evaluate(x), WithinAbs(99604837.7255771309, epsilon));
            }
        }

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

    } catch(ios_base::failure &e) {
        std::cerr << "baseDir: " << baseDir << std::endl;
        throw e;
    }
}

TEST_CASE("Conjugate Frank-Wolfe - large tests", "[cfw][cfw-large][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    try {
        // Supply
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(baseDir + "data/porto/porto-armis.taz.xml");
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/porto/matrix.9.0.10.0.2.fma");

        clk::time_point begin = clk::now();

        SECTION("Large") {
            Static::BPRNetwork::Loader<SUMO::NetworkTAZs> loader;
            Static::BPRNetwork                           *network = loader.load(sumo);

            Static::Demand demand = Static::Demand::fromOFormat(oDemand, loader.adapter);

            double totalDemand = demand.getTotalDemand();
            REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

            Static::DijkstraAoN  aon;
            Static::SolutionBase x0 = aon.solve(*network, demand);
            REQUIRE_THAT(network->evaluate(x0), WithinAbs(17546.6462131649, 1e-4));

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
            REQUIRE_THAT(network->evaluate(x), WithinAbs(12091.1614891103, epsilon));
        }

        SECTION("Large not convex") {
            Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;
            Static::BPRNotConvexNetwork                           *network = loader.load(sumo);
            Static::Demand                                         demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

            double totalDemand = demand.getTotalDemand();
            REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

            Static::DijkstraAoN  aon;
            Static::SolutionBase x0 = aon.solve(*network, demand);
            REQUIRE_THAT(network->evaluate(x0), WithinAbs(PORTO_ARMIS_AON_COST, 1e-4));

            Opt::GeneticIntervalSolver solver;
            solver.setInterval(0, 1);
            solver.setStopCriteria(1e-6);

            Static::ConjugateFrankWolfe fw(aon, solver, logger);

            double epsilon = 1.0;
            fw.setStopCriteria(epsilon);
            fw.setIterations(100);

            Static::Solution x = fw.solve(*network, demand, x0);

            REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
            REQUIRE_THAT(network->evaluate(x), WithinAbs(5397036.2276287684, epsilon));
        }

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

    } catch(ios_base::failure &e) {
        std::cerr << "baseDir: " << baseDir << std::endl;
        throw e;
    }
}

TEST_CASE("Iterative equilibration", "[ie][!benchmark]") {
    Log::ProgressLoggerTableOStream logger;

    Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;

    // Supply
    SUMO::Network                sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto-armis.net.xml");
    SUMO::TAZs                   sumoTAZs    = SUMO::TAZ::loadFromFile(baseDir + "data/porto/porto-armis.taz.xml");
    SUMO::NetworkTAZs            sumo{sumoNetwork, sumoTAZs};
    Static::BPRNotConvexNetwork *network = loader.load(sumo);

    // Demand
    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/porto/matrix.9.0.10.0.2.fma");
    Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, loader.adapter);

    double totalDemand = demand.getTotalDemand();
    REQUIRE_THAT(totalDemand, WithinAbs(MATRIX_9_10_TOTAL_DEMAND, 1e-4));

    clk::time_point begin = clk::now();

    Static::DijkstraAoN  aon;
    Static::SolutionBase x0 = aon.solve(*network, demand);
    REQUIRE_THAT(network->evaluate(x0), WithinAbs(PORTO_ARMIS_AON_COST, 1e-3));

    ofstream                        ofsNull("/dev/null");
    Log::ProgressLoggerTableOStream loggerNull(ofsNull);

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

        Static::FrankWolfe fw(aon, solver, loggerNull);

        double epsilon = 1.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(3);

        Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::FrankWolfe> ie(fw, logger);
        ie.setIterations(10);

        Static::Solution x = ie.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinAbs(5403604.5468498664, epsilon));

        network->saveResultsToFile(sumo, x, loader.adapter, baseDir + "data/out/edgedata-static.xml", baseDir + "data/out/routes-static.xml");
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

        Static::ConjugateFrankWolfe fw(aon, solver, loggerNull);

        fw.setStopCriteria(0.0);
        fw.setIterations(5);

        Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::ConjugateFrankWolfe> ie(fw, logger);
        ie.setIterations(10);

        Static::Solution x = ie.solve(*network, demand, x0);

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(x.getTotalFlow(), WithinAbs(totalDemand, 1e-4));
        REQUIRE_THAT(network->evaluate(x), WithinAbs(5380740.3194306595, 1e-6));

        network->saveResultsToFile(sumo, x, loader.adapter, baseDir + "data/out/edgedata-static.xml", baseDir + "data/out/routes-static.xml");
    }
}
