#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>

#include "static/StaticSolution.hpp"

using Catch::Matchers::WithinAbs;
using namespace std;

TEST_CASE("Static solution", "[static-solution]") {
    StaticSolutionBase x;
    SECTION("Add path"){
        x.addPath(StaticNetwork::Path{1l, 3l}, 1.0);
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(1.0, 1e-10));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(1.0, 1e-10));
    }
}

TEST_CASE("Static solution interpolation", "[static-solution][interpolation]") {
    StaticSolutionBase x1; x1.addPath(StaticNetwork::Path{1, 4, 3, 7}, 2.0);
    StaticSolutionBase x2; x2.addPath(StaticNetwork::Path{4, 2, 6, 5}, 3.0);

    double e = 1e-10;
    
    SECTION("alpha=0.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.0);
        REQUIRE_THAT(x.getFlowInEdge(0), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(2.0, e));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(2.0, e));
        REQUIRE_THAT(x.getFlowInEdge(4), WithinAbs(2.0, e));
        REQUIRE_THAT(x.getFlowInEdge(5), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(6), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(7), WithinAbs(2.0, e));
        REQUIRE_THAT(x.getFlowInEdge(8), WithinAbs(0.0, e));
    }

    SECTION("alpha=1.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 1.0);
        REQUIRE_THAT(x.getFlowInEdge(0), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(3.0, e));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(4), WithinAbs(3.0, e));
        REQUIRE_THAT(x.getFlowInEdge(5), WithinAbs(3.0, e));
        REQUIRE_THAT(x.getFlowInEdge(6), WithinAbs(3.0, e));
        REQUIRE_THAT(x.getFlowInEdge(7), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(8), WithinAbs(0.0, e));
    }

    SECTION("alpha=0.5"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.5);
        REQUIRE_THAT(x.getFlowInEdge(0), WithinAbs(0.0, e));
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(1.0, e));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(1.5, e));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(1.0, e));
        REQUIRE_THAT(x.getFlowInEdge(4), WithinAbs(2.5, e));
        REQUIRE_THAT(x.getFlowInEdge(5), WithinAbs(1.5, e));
        REQUIRE_THAT(x.getFlowInEdge(6), WithinAbs(1.5, e));
        REQUIRE_THAT(x.getFlowInEdge(7), WithinAbs(1.0, e));
        REQUIRE_THAT(x.getFlowInEdge(8), WithinAbs(0.0, e));
    }

    SECTION("alpha=0.3"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.3);
        REQUIRE_THAT(x.getFlowInEdge(0), WithinAbs(0.0*(1.0-0.3) + 0.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(1), WithinAbs(2.0*(1.0-0.3) + 0.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(2), WithinAbs(0.0*(1.0-0.3) + 3.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(3), WithinAbs(2.0*(1.0-0.3) + 0.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(4), WithinAbs(2.0*(1.0-0.3) + 3.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(5), WithinAbs(0.0*(1.0-0.3) + 3.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(6), WithinAbs(0.0*(1.0-0.3) + 3.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(7), WithinAbs(2.0*(1.0-0.3) + 0.0*0.3, e));
        REQUIRE_THAT(x.getFlowInEdge(8), WithinAbs(0.0*(1.0-0.3) + 0.0*0.3, e));
    }
}
