#include <catch2/catch_test_macros.hpp>

#include "static/StaticSolution.hpp"

TEST_CASE("Static solution", "[static-solution]"){
    StaticSolution x;
    SECTION("Add path"){
        x.addPath(StaticNetwork::Path{1l, 3l}, 1.0);
        REQUIRE(1.0 == x.getFlowInEdge(1));
        REQUIRE(0.0 == x.getFlowInEdge(2));
        REQUIRE(1.0 == x.getFlowInEdge(3));
    }
}

TEST_CASE("Static solution interpolation", "[static-solution][interpolation]"){
    StaticSolution x1; x1.addPath(StaticNetwork::Path{1, 4, 3, 7}, 2.0);
    StaticSolution x2; x2.addPath(StaticNetwork::Path{4, 2, 6, 5}, 3.0);
    
    SECTION("alpha=0.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.0);
        REQUIRE(0.0 == x.getFlowInEdge(0));
        REQUIRE(2.0 == x.getFlowInEdge(1));
        REQUIRE(0.0 == x.getFlowInEdge(2));
        REQUIRE(2.0 == x.getFlowInEdge(3));
        REQUIRE(2.0 == x.getFlowInEdge(4));
        REQUIRE(0.0 == x.getFlowInEdge(5));
        REQUIRE(0.0 == x.getFlowInEdge(6));
        REQUIRE(2.0 == x.getFlowInEdge(7));
        REQUIRE(0.0 == x.getFlowInEdge(8));
    }

    SECTION("alpha=1.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 1.0);
        REQUIRE(0.0 == x.getFlowInEdge(0));
        REQUIRE(0.0 == x.getFlowInEdge(1));
        REQUIRE(3.0 == x.getFlowInEdge(2));
        REQUIRE(0.0 == x.getFlowInEdge(3));
        REQUIRE(3.0 == x.getFlowInEdge(4));
        REQUIRE(3.0 == x.getFlowInEdge(5));
        REQUIRE(3.0 == x.getFlowInEdge(6));
        REQUIRE(0.0 == x.getFlowInEdge(7));
        REQUIRE(0.0 == x.getFlowInEdge(8));
    }

    SECTION("alpha=0.5"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.5);
        REQUIRE(0.0 == x.getFlowInEdge(0));
        REQUIRE(1.0 == x.getFlowInEdge(1));
        REQUIRE(1.5 == x.getFlowInEdge(2));
        REQUIRE(1.0 == x.getFlowInEdge(3));
        REQUIRE(2.5 == x.getFlowInEdge(4));
        REQUIRE(1.5 == x.getFlowInEdge(5));
        REQUIRE(1.5 == x.getFlowInEdge(6));
        REQUIRE(1.0 == x.getFlowInEdge(7));
        REQUIRE(0.0 == x.getFlowInEdge(8));
    }

    SECTION("alpha=0.3"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.3);
        REQUIRE(0.0*(1.0-0.3) + 0.0*0.3 == x.getFlowInEdge(0));
        REQUIRE(2.0*(1.0-0.3) + 0.0*0.3 == x.getFlowInEdge(1));
        REQUIRE(0.0*(1.0-0.3) + 3.0*0.3 == x.getFlowInEdge(2));
        REQUIRE(2.0*(1.0-0.3) + 0.0*0.3 == x.getFlowInEdge(3));
        REQUIRE(2.0*(1.0-0.3) + 3.0*0.3 == x.getFlowInEdge(4));
        REQUIRE(0.0*(1.0-0.3) + 3.0*0.3 == x.getFlowInEdge(5));
        REQUIRE(0.0*(1.0-0.3) + 3.0*0.3 == x.getFlowInEdge(6));
        REQUIRE(2.0*(1.0-0.3) + 0.0*0.3 == x.getFlowInEdge(7));
        REQUIRE(0.0*(1.0-0.3) + 0.0*0.3 == x.getFlowInEdge(8));
    }
}
