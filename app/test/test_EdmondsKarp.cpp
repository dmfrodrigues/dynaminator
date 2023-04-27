#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Alg/Flow/EdmondsKarp.hpp"
#include "Alg/Flow/MaxFlow.hpp"
#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/BFS.hpp"
#include "test/problem/graphs.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;

TEST_CASE("Edmonds-Karp algorithm", "[flow][maxflow][edmonds-karp]") {
    SECTION("0,5") {
        Alg::Graph G = graph2();

        Alg::ShortestPath::ShortestPathOneOne *sp = new Alg::ShortestPath::BFS();
        Alg::Flow::EdmondsKarp                 maxFlow(*sp);

        REQUIRE_THAT(maxFlow.solve(G, 0, 5), WithinAbs(23, 1e-9));
        // TODO: check if flows are somewhat correct.

        delete sp;
    }
}
