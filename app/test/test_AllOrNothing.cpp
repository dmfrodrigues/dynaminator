#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "static/algos/AllOrNothing.hpp"
#include "test/problem/cases.hpp"

using namespace std;

using Catch::Approx;

TEST_CASE("All or Nothing", "[allornothing]") {
    StaticProblem *problem = getStaticProblemTestCase1();

    AllOrNothing solver(*problem);
    StaticSolutionBase x = solver.solve();

    REQUIRE(Approx(0.0).margin(1e-10) == x.getFlowInEdge(1));
    REQUIRE(Approx(4.0).margin(1e-10) == x.getFlowInEdge(2));
    REQUIRE(Approx(4.0).margin(1e-10) == x.getFlowInEdge(3));

    delete problem;
}
