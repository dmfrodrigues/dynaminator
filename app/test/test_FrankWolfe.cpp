#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#include "data/sumo/TAZs.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/algos/FrankWolfe.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "test/problem/cases.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

typedef chrono::steady_clock clk;

TEST_CASE("Frank-Wolfe", "[fw]") {
    SECTION("Case 1") {
        StaticProblem *problem = getStaticProblemTestCase1();

        AllOrNothing aon(*problem);
        StaticSolutionBase x0 = aon.solve();

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(4.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = (-3.0 + sqrt(53)) / 2.0;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(4.0 - x1, 1e-6));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

        delete problem;
    }

    SECTION("Case 2") {
        StaticProblem *problem = getStaticProblemTestCase2();

        AllOrNothing aon(*problem);
        StaticSolutionBase x0 = aon.solve();

        REQUIRE_THAT(x0.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x0.getFlowInEdge(2), WithinAbs(7000.0, 1e-10));

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = 3376.36917;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        delete problem;
    }

    SECTION("Case 3") {
        StaticProblem *problem = getStaticProblemTestCase3();

        AllOrNothing aon(*problem);
        StaticSolutionBase x0 = aon.solve();

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = 4131.89002;
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(x1, 1e-2));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(7000.0 - x1, 1e-2));

        delete problem;
    }

    SECTION("Large") {
        // Supply
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile("data/network/net.net.xml");
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile("data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile("data/od/matrix.9.0.10.0.2.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE_THAT(totalDemand, WithinAbs(102731.0/(60*60), 1e-4));

        // FW
        StaticProblem problem{*network, demand};

        clk::time_point begin = clk::now();

        AllOrNothing aon(problem);
        StaticSolution x0 = aon.solve();
        REQUIRE_THAT(network->evaluate(x0), WithinAbs(12548.1603305499, 1e-4));

        FrankWolfe fw(problem);
        fw.setStartingSolution(x0);
        /**
         * 1e-4 is the adequate scale because a 1s difference for one driver
         * translates to a difference in the cost function of
         * 1 veh/h * 1s = 1/3600 = 2.778e-4.
         * But this takes too much time.
         * So we are using a criteria of 0.2 for the optimal value,
         * and x for automated testing
         */
        // double epsilon = 0.2;
        double epsilon = 2.0;
        fw.setStopCriteria(epsilon);
        fw.setIterations(10000);

        StaticSolution x = fw.solve();

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE_THAT(network->evaluate(x), WithinAbs(11999.9047499, epsilon));

        network->saveResultsToFile(x, adapter, "data/out/edgedata-static.xml");
    }
}
