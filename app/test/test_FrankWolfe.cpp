#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "test/problem/cases.hpp"

#include "static/algos/AllOrNothing.hpp"
#include "static/algos/FrankWolfe.hpp"

#include <cmath>
#include <memory>

using namespace std;
using Catch::Approx;

TEST_CASE("Frank-Wolfe", "[fw]"){
    const double e = 1e-5;

    StaticProblem *problem = getStaticProblemTestCase1();

    AllOrNothing aon(*problem);
    StaticSolution x0 = aon.solve();

    REQUIRE(0.0 == x0.getFlowInEdge(1));
    REQUIRE(4.0 == x0.getFlowInEdge(2));
    REQUIRE(4.0 == x0.getFlowInEdge(3));

    FrankWolfe fw(*problem);
    fw.setStartingSolution(x0);
    StaticSolution x = fw.solve();

    double x1 = (-6.0 + sqrt(312))/6.0;
    REQUIRE(Approx(x1).margin(e) == x.getFlowInEdge(1));
    REQUIRE(Approx(4.0 - x1).margin(e) == x.getFlowInEdge(2));
    REQUIRE(4.0 == x.getFlowInEdge(3));

    delete problem;
}
