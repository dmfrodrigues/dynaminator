#pragma once

#include "Graph.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

const long EDGE_ID_IRRELEVANT = -1;

Graph graph1(){
    Graph G;
    for (int i = 0; i < 7; ++i) G.addNode(i);
    G.addEdge(EDGE_ID_IRRELEVANT, 0, 1, 1);
    G.addEdge(EDGE_ID_IRRELEVANT, 1, 2, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 0, 3, 5);
    G.addEdge(EDGE_ID_IRRELEVANT, 3, 4, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 2, 3, 1);
    G.addEdge(EDGE_ID_IRRELEVANT, 2, 5, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 4, 5, 3);
    G.addEdge(EDGE_ID_IRRELEVANT, 5, 6, 4);
    return G;
}

void testPath(std::vector<Graph::Node> expected, Graph::Path got) {
    if (expected.size() == 0) {
        REQUIRE(1 == got.size());
        REQUIRE(-1 == got.front().id);
        REQUIRE(Graph::NODE_INVALID == got.front().u);
        REQUIRE(Graph::NODE_INVALID == got.front().v);
        REQUIRE_THAT(got.front().w, Catch::Matchers::WithinAbs(0, 1e-10));

        return;
    }
    REQUIRE(got.size() == expected.size() - 1);
    auto itExpected = expected.begin();
    auto itGot = got.begin();
    for (; itGot != got.end(); ++itExpected, ++itGot) {
        auto itExpectedNext = itExpected;
        ++itExpectedNext;
        REQUIRE(*itExpected == itGot->u);
        REQUIRE(*itExpectedNext == itGot->v);
    }
}
