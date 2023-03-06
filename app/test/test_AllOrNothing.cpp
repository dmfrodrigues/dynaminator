#include <catch2/catch_test_macros.hpp>

#include "test/problem/cases.hpp"

#include "static/algos/AllOrNothing.hpp"

using namespace std;

TEST_CASE("All or Nothing", "[allornothing]"){
    unique_ptr<StaticProblem> problem = getStaticProblemTestCase1();

    AllOrNothing solver(*problem.get());
    StaticSolution x = solver.solve();

    REQUIRE(0.0 == x.getFlowInEdge(1));
    REQUIRE(4.0 == x.getFlowInEdge(2));
    REQUIRE(4.0 == x.getFlowInEdge(3));
}
