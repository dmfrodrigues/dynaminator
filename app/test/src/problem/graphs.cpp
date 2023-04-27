#include "test/problem/graphs.hpp"

Alg::Graph graph1() {
    Alg::Graph G;
    for(int i = 0; i <= 6; ++i) G.addNode(i);
    G.addEdge(0, 0, 1, 1);
    G.addEdge(1, 1, 2, 2);
    G.addEdge(2, 0, 3, 5);
    G.addEdge(3, 3, 4, 2);
    G.addEdge(4, 2, 3, 1);
    G.addEdge(5, 2, 5, 2);
    G.addEdge(6, 4, 5, 3);
    G.addEdge(7, 5, 6, 4);
    return G;
}

Alg::Graph graph2() {
    Alg::Graph G;
    for(int i = 0; i <= 5; ++i) G.addNode(i);

    G.addEdge(0, 0, 1, 16);
    G.addEdge(1, 0, 2, 13);
    G.addEdge(2, 1, 2, 10);
    G.addEdge(3, 1, 3, 12);
    G.addEdge(4, 2, 1, 4);
    G.addEdge(5, 2, 4, 14);
    G.addEdge(6, 3, 2, 9);
    G.addEdge(7, 3, 5, 20);
    G.addEdge(8, 4, 3, 7);
    G.addEdge(9, 4, 5, 4);

    return G;
}
