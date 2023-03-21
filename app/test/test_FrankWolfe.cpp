#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#include "data/SumoTAZs.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/algos/FrankWolfe.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "test/problem/cases.hpp"
#include "test/utils.hpp"

using namespace std;
using Catch::Approx;

typedef chrono::steady_clock clk;

TEST_CASE("Frank-Wolfe", "[fw]") {
    const double e = 1e-5;

    SECTION("Case 1") {
        StaticProblem *problem = getStaticProblemTestCase1();

        AllOrNothing aon(*problem);
        StaticSolution x0 = aon.solve();

        REQUIRE(Approx(0.0).margin(1e-10) == x0.getFlowInEdge(1));
        REQUIRE(Approx(4.0).margin(1e-10) == x0.getFlowInEdge(2));
        REQUIRE(Approx(4.0).margin(1e-10) == x0.getFlowInEdge(3));

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = (-6.0 + sqrt(312)) / 6.0;
        REQUIRE(Approx(x1).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(4.0 - x1).margin(e) == x.getFlowInEdge(2));
        REQUIRE(Approx(4.0).margin(1e-10) == x.getFlowInEdge(3));

        delete problem;
    }

    SECTION("Case 2") {
        StaticProblem *problem = getStaticProblemTestCase2();

        AllOrNothing aon(*problem);
        StaticSolution x0 = aon.solve();

        REQUIRE(Approx(0.0).margin(1e-10) == x0.getFlowInEdge(1));
        REQUIRE(Approx(7000.0).margin(1e-10) == x0.getFlowInEdge(2));

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = 4131.8886544297;
        REQUIRE(Approx(x1).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(7000.0 - x1).margin(e) == x.getFlowInEdge(2));

        delete problem;
    }

    SECTION("Case 3") {
        StaticProblem *problem = getStaticProblemTestCase3();

        AllOrNothing aon(*problem);
        StaticSolution x0 = aon.solve();

        FrankWolfe fw(*problem);
        fw.setStartingSolution(x0);
        StaticSolution x = fw.solve();

        double x1 = 3376.36917;
        REQUIRE(Approx(x1).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(7000.0 - x1).margin(e) == x.getFlowInEdge(2));

        delete problem;
    }

    SECTION("Large") {
        filesystem::path exePath = getExePath();
        filesystem::path basePath = exePath.parent_path().parent_path();

        // Supply
        SumoNetwork sumoNetwork = SumoNetwork::loadFromFile(basePath.string() + "/data/network/net.net.xml");
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(basePath.string() + "/data/network/taz.xml");
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(basePath.string() + "/data/od/matrix.8.0.9.0.1.fma");
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        double totalDemand = demand.getTotalDemand();
        REQUIRE(Approx(8.3302777778).margin(1e-3) == totalDemand);

        // FW
        StaticProblem problem{*network, demand};

        clk::time_point begin = clk::now();

        AllOrNothing aon(problem);
        StaticSolution x0 = aon.solve();
        REQUIRE(Approx(3152.5516079364).margin(1e-6) == network->evaluate(x0));

        FrankWolfe fw(problem);
        fw.setStartingSolution(x0);
        fw.setStopCriteria(1e-7);

        StaticSolution x = fw.solve();

        clk::time_point end = clk::now();
        cout << "Time difference = " << (double)chrono::duration_cast<chrono::nanoseconds>(end - begin).count() * 1e-9 << "[s]" << endl;

        REQUIRE(Approx(3147.015646313).margin(1e-6) == network->evaluate(x));

        network->saveResultsToFile(x, adapter, basePath.string() + "/data/out/edgedata-static.xml");
    }
}
