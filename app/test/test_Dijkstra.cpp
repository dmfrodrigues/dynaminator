#include <catch2/catch_test_macros.hpp>

#include "Dijkstra.hpp"

TEST_CASE("Dijkstra's algorithm", "[shortestpath][shortestpath-onemany][dijkstra]"){
    Graph G;
    for(int i = 0; i < 7; ++i) G.addNode(i);
    G.addEdge(0, 1, 1); G.addEdge(1, 2, 2); G.addEdge(0, 3, 5); G.addEdge(3, 4, 2);
    G.addEdge(2, 3, 1); G.addEdge(2, 5, 2); G.addEdge(4, 5, 3); G.addEdge(5, 6, 4);

    SECTION("Start 0"){
        ShortestPathOneMany *shortestPath = new Dijkstra();
        shortestPath->initialize(&G, 0);
        shortestPath->run();

        REQUIRE(std::list<Graph::Node>({0               }) == shortestPath->getPath(0));
        REQUIRE(std::list<Graph::Node>({0, 1            }) == shortestPath->getPath(1));
        REQUIRE(std::list<Graph::Node>({0, 1, 2         }) == shortestPath->getPath(2));
        REQUIRE(std::list<Graph::Node>({0, 1, 2, 3      }) == shortestPath->getPath(3));
        REQUIRE(std::list<Graph::Node>({0, 1, 2, 3, 4   }) == shortestPath->getPath(4));
        REQUIRE(std::list<Graph::Node>({0, 1, 2, 5      }) == shortestPath->getPath(5));
        REQUIRE(std::list<Graph::Node>({0, 1, 2, 5, 6   }) == shortestPath->getPath(6));

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

        REQUIRE(std::list<Graph::Node>({                }) == shortestPath->getPath(0));
        REQUIRE(std::list<Graph::Node>({1               }) == shortestPath->getPath(1));
        REQUIRE(std::list<Graph::Node>({1, 2            }) == shortestPath->getPath(2));
        REQUIRE(std::list<Graph::Node>({1, 2, 3         }) == shortestPath->getPath(3));
        REQUIRE(std::list<Graph::Node>({1, 2, 3, 4      }) == shortestPath->getPath(4));
        REQUIRE(std::list<Graph::Node>({1, 2, 5         }) == shortestPath->getPath(5));
        REQUIRE(std::list<Graph::Node>({1, 2, 5, 6      }) == shortestPath->getPath(6));

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
