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
#include "test/utils.hpp"

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
        filesystem::path exePath = getExePath();
        filesystem::path basePath = exePath.parent_path().parent_path();

        // Supply
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(basePath.string() + "/data/network/net.net.xml");
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(basePath.string() + "/data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(basePath.string() + "/data/od/matrix.9.0.10.0.2.fma");
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
         * 1 veh/h * 1s = 1/3600 = 2.778e-4
         */
        // fw.setStopCriteria(1e-3);
        fw.setStopCriteria(1e-1);

        StaticSolution x = fw.solve();

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        // REQUIRE_THAT(network->evaluate(x), WithinAbs(12000.3361258556, 1e-3));
        REQUIRE_THAT(network->evaluate(x), WithinAbs(12003.0892665995, 1e-2));

        network->saveResultsToFile(x, adapter, basePath.string() + "/data/out/edgedata-static.xml");
    }
}
