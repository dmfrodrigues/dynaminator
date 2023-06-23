#include "Alg/ShortestPath/Dijkstra.hpp"

#include <chrono>
#include <iostream>
#include <queue>
#include <utility>

#include "Alg/BinaryHeap.hpp"

using namespace std;
using namespace Alg;
using namespace Alg::ShortestPath;

typedef Graph::Node         Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge         Edge;

typedef BinaryHeap<pair<Weight, Node>> MinPriorityQueue;

bool Dijkstra::isStart(Node u) const {
    return sSet.count(u);
}

void Dijkstra::solveList(const Graph &G, const list<Node> &sList) {
    // Initialize
    Node maxNode = 0;
    for(const Node &u: G.getNodes()) {
        maxNode = max(maxNode, u);
    }
    dist = vector<Weight>(maxNode + 1, Edge::WEIGHT_INF);
    prev = vector<Edge>(maxNode + 1, Graph::EDGE_INVALID);

    sSet.clear();
    sSet.insert(sList.begin(), sList.end());

    // Run
    vector<MinPriorityQueue::Element *> elements(dist.size());

    MinPriorityQueue Q;
    Q.reserve(dist.size());

    for(const Node &s: sList) {
        dist[s]     = 0;
        elements[s] = &Q.push({0, s});
    }

    while(!Q.empty()) {
        const auto &[du, u] = Q.top();
        Q.pop();
        for(const Edge &e: G.getAdj(u)) {
            Weight  c_    = du + e.w;
            Weight &distV = dist[e.v];
            if(c_ < distV) {
                MinPriorityQueue::Element *&elV = elements[e.v];
                if(elV)
                    elV->decreaseKey({c_, e.v});
                else
                    elV = &Q.push({c_, e.v});
                distV     = c_;
                prev[e.v] = e;
            }
        }
    }
}

Edge Dijkstra::getPrev(Node d) const {
    return prev.at(d);
}

Weight Dijkstra::getPathWeight(Node d) const {
    return dist.at(d);
}

// clang-format off
bool Dijkstra::hasVisited(Node u) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (dist.at(u) != Edge::WEIGHT_INF);
#pragma GCC diagnostic pop
}
// clang-format on
