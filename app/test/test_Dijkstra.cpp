#include <catch2/catch_test_macros.hpp>

#include "shortest-path/Dijkstra.hpp"

const long EDGE_ID_IRRELEVANT = -1;

void testPath(std::vector<Graph::Node> expected, std::vector<Graph::Edge> got){
    if(expected.size() == 0){
        REQUIRE(1 == got.size());
        REQUIRE(Graph::EDGE_INVALID == got[0]);
        return;
    }
    REQUIRE(got.size() == expected.size() - 1);
    for(int i = 0; i < got.size(); ++i){
        REQUIRE(expected.at(i  ) == got.at(i).u);
        REQUIRE(expected.at(i+1) == got.at(i).v);
    }
}

TEST_CASE("Dijkstra's algorithm", "[shortestpath][shortestpath-onemany][dijkstra]"){
    Graph G;
    for(int i = 0; i < 7; ++i) G.addNode(i);
    G.addEdge(EDGE_ID_IRRELEVANT, 0, 1, 1); G.addEdge(EDGE_ID_IRRELEVANT, 1, 2, 2); G.addEdge(EDGE_ID_IRRELEVANT, 0, 3, 5); G.addEdge(EDGE_ID_IRRELEVANT, 3, 4, 2);
    G.addEdge(EDGE_ID_IRRELEVANT, 2, 3, 1); G.addEdge(EDGE_ID_IRRELEVANT, 2, 5, 2); G.addEdge(EDGE_ID_IRRELEVANT, 4, 5, 3); G.addEdge(EDGE_ID_IRRELEVANT, 5, 6, 4);

    SECTION("Start 0"){
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->initialize(&G, 0);
        shortestPath->run();

        testPath({0               }, shortestPath->getPath(0));
        testPath({0, 1            }, shortestPath->getPath(1));
        testPath({0, 1, 2         }, shortestPath->getPath(2));
        testPath({0, 1, 2, 3      }, shortestPath->getPath(3));
        testPath({0, 1, 2, 3, 4   }, shortestPath->getPath(4));
        testPath({0, 1, 2, 5      }, shortestPath->getPath(5));
        testPath({0, 1, 2, 5, 6   }, shortestPath->getPath(6));

        REQUIRE(0 == shortestPath->getPathWeight(0));
        REQUIRE(1 == shortestPath->getPathWeight(1));
        REQUIRE(3 == shortestPath->getPathWeight(2));
        REQUIRE(4 == shortestPath->getPathWeight(3));
        REQUIRE(6 == shortestPath->getPathWeight(4));
        REQUIRE(5 == shortestPath->getPathWeight(5));
        REQUIRE(9 == shortestPath->getPathWeight(6));

        delete shortestPath;
    }
    SECTION("Start 1"){
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->initialize(&G, 1);
        shortestPath->run();

        testPath({                }, shortestPath->getPath(0));
        testPath({1               }, shortestPath->getPath(1));
        testPath({1, 2            }, shortestPath->getPath(2));
        testPath({1, 2, 3         }, shortestPath->getPath(3));
        testPath({1, 2, 3, 4      }, shortestPath->getPath(4));
        testPath({1, 2, 5         }, shortestPath->getPath(5));
        testPath({1, 2, 5, 6      }, shortestPath->getPath(6));

        REQUIRE(Graph::Edge::WEIGHT_INF == shortestPath->getPathWeight(0));
        REQUIRE(0 == shortestPath->getPathWeight(1));
        REQUIRE(2 == shortestPath->getPathWeight(2));
        REQUIRE(3 == shortestPath->getPathWeight(3));
        REQUIRE(5 == shortestPath->getPathWeight(4));
        REQUIRE(4 == shortestPath->getPathWeight(5));
        REQUIRE(8 == shortestPath->getPathWeight(6));

        delete shortestPath;
    }
}
