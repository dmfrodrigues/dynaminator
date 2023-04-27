#include "Alg/ShortestPath/BFS.hpp"

#include <chrono>
#include <iostream>
#include <queue>
#include <utility>

#include "Alg/BinaryHeap.hpp"

using namespace std;
using namespace Alg;
using namespace Alg::ShortestPath;

typedef Graph::Node Node;
typedef Graph::Edge Edge;
typedef queue<Node> Queue;

Node BFS::getStart() const{
    return s;
}

void BFS::solve(const Graph &G, Node s_) {
    // Initialize
    s = s_;

    Node maxNode = 0;
    for (const Node &u : G.getNodes()) {
        maxNode = max(maxNode, u);
    }
    dist = vector<Weight>(maxNode+1, BFS::WEIGHT_INF);
    prev = vector<Edge>(maxNode+1, Graph::EDGE_INVALID);

    // Run
    Queue Q;

    dist[s] = 0;
    Q.push(s);
    while (!Q.empty()) {
        Node u = Q.front();
        Q.pop();
        BFS::Weight du = dist[u];
        for (const Edge &e: G.getAdj(u)){
            BFS::Weight c_ = du + 1;
            BFS::Weight &distV = dist[e.v];
            if (e.w > 0 && c_ < distV) {
                Q.push(e.v);
                distV = c_;
                prev[e.v] = e;
            }
        }
    }
}

Graph::Edge::Weight BFS::solve(const Graph &G, Node s_, Node t_){
    solve(G, s_);
    return getPathWeight(t_);
}

Edge BFS::getPrev(Node d) const {
    return prev.at(d);
}

Graph::Edge::Weight BFS::getPathWeight(Node d) const {
    return (Graph::Edge::Weight)dist.at(d);
}

bool BFS::hasVisited(Node u) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (dist.at(u) != BFS::WEIGHT_INF);
#pragma GCC diagnostic pop
}
