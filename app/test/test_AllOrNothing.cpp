#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>

#include "static/algos/AllOrNothing.hpp"
#include "test/problem/cases.hpp"

using namespace std;

using Catch::Matchers::WithinAbs;

TEST_CASE("All or Nothing", "[allornothing]") {
    StaticProblem *problem = getStaticProblemTestCase1();

    AllOrNothing solver(*problem);
    StaticSolutionBase x = solver.solve();

    REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(4.0, 1e-10));
    REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(4.0, 1e-10));

    delete problem;
}
