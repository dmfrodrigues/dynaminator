#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "static/StaticSolution.hpp"

using Catch::Approx;

TEST_CASE("Static solution", "[static-solution]"){
    StaticSolution x;
    SECTION("Add path"){
        x.addPath(StaticNetwork::Path{1l, 3l}, 1.0);
        REQUIRE(Approx(1.0).margin(1e-10) == x.getFlowInEdge(1));
        REQUIRE(Approx(0.0).margin(1e-10) == x.getFlowInEdge(2));
        REQUIRE(Approx(1.0).margin(1e-10) == x.getFlowInEdge(3));
    }
}

TEST_CASE("Static solution interpolation", "[static-solution][interpolation]"){
    StaticSolution x1; x1.addPath(StaticNetwork::Path{1, 4, 3, 7}, 2.0);
    StaticSolution x2; x2.addPath(StaticNetwork::Path{4, 2, 6, 5}, 3.0);

    double e = 1e-10;
    
    SECTION("alpha=0.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.0);
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(0));
        REQUIRE(Approx(2.0).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(2));
        REQUIRE(Approx(2.0).margin(e) == x.getFlowInEdge(3));
        REQUIRE(Approx(2.0).margin(e) == x.getFlowInEdge(4));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(5));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(6));
        REQUIRE(Approx(2.0).margin(e) == x.getFlowInEdge(7));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(8));
    }

    SECTION("alpha=1.0"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 1.0);
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(0));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(3.0).margin(e) == x.getFlowInEdge(2));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(3));
        REQUIRE(Approx(3.0).margin(e) == x.getFlowInEdge(4));
        REQUIRE(Approx(3.0).margin(e) == x.getFlowInEdge(5));
        REQUIRE(Approx(3.0).margin(e) == x.getFlowInEdge(6));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(7));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(8));
    }

    SECTION("alpha=0.5"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.5);
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(0));
        REQUIRE(Approx(1.0).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(1.5).margin(e) == x.getFlowInEdge(2));
        REQUIRE(Approx(1.0).margin(e) == x.getFlowInEdge(3));
        REQUIRE(Approx(2.5).margin(e) == x.getFlowInEdge(4));
        REQUIRE(Approx(1.5).margin(e) == x.getFlowInEdge(5));
        REQUIRE(Approx(1.5).margin(e) == x.getFlowInEdge(6));
        REQUIRE(Approx(1.0).margin(e) == x.getFlowInEdge(7));
        REQUIRE(Approx(0.0).margin(e) == x.getFlowInEdge(8));
    }

    SECTION("alpha=0.3"){
        StaticSolution x = StaticSolution::interpolate(x1, x2, 0.3);
        REQUIRE(Approx(0.0*(1.0-0.3) + 0.0*0.3).margin(e) == x.getFlowInEdge(0));
        REQUIRE(Approx(2.0*(1.0-0.3) + 0.0*0.3).margin(e) == x.getFlowInEdge(1));
        REQUIRE(Approx(0.0*(1.0-0.3) + 3.0*0.3).margin(e) == x.getFlowInEdge(2));
        REQUIRE(Approx(2.0*(1.0-0.3) + 0.0*0.3).margin(e) == x.getFlowInEdge(3));
        REQUIRE(Approx(2.0*(1.0-0.3) + 3.0*0.3).margin(e) == x.getFlowInEdge(4));
        REQUIRE(Approx(0.0*(1.0-0.3) + 3.0*0.3).margin(e) == x.getFlowInEdge(5));
        REQUIRE(Approx(0.0*(1.0-0.3) + 3.0*0.3).margin(e) == x.getFlowInEdge(6));
        REQUIRE(Approx(2.0*(1.0-0.3) + 0.0*0.3).margin(e) == x.getFlowInEdge(7));
        REQUIRE(Approx(0.0*(1.0-0.3) + 0.0*0.3).margin(e) == x.getFlowInEdge(8));
    }
}
