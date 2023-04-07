#include "shortest-path/Dijkstra.hpp"

#include <chrono>
#include <iostream>
#include <queue>
#include <utility>

#include "structs/BinaryHeap.hpp"

using namespace std;

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;
template<class K, class V> using umap = std::unordered_map<K, V>;
typedef umap<Node, Weight> dist_t;
typedef umap<Node, Node> prev_t;
typedef BinaryHeap<std::pair<Weight, Node>> MinPriorityQueue;
typedef std::chrono::high_resolution_clock hrc;
#define mk(a, b) (std::make_pair((a), (b)))

Node Dijkstra::getStart() const{
    return s;
}

void Dijkstra::initialize(const Graph *G_, Node s_) {
    this->s = s_;
    this->G = G_;
    Node maxNode = 0;
    for (const Node &u : G->getNodes()) {
        maxNode = max(maxNode, u);
    }
    dist = vector<Weight>(maxNode+1, Edge::WEIGHT_INF);
    prev = vector<Edge>(maxNode+1, Graph::EDGE_INVALID);
}

void Dijkstra::run() {
    vector<MinPriorityQueue::Element *> elements(dist.size());

    MinPriorityQueue Q;
    Q.reserve(dist.size());

    dist[s] = 0;
    elements[s] = &Q.push({0, s});
    while (!Q.empty()) {
        pair<Weight, Node> p = Q.top();
        Q.pop();
        const Node &u = p.second;
        const Weight &du = p.first;
        for (const Edge &e: G->getAdj(u)){
            Weight c_ = du + e.w;
            Weight &distV = dist[e.v];
            if (c_ < distV) {
            MinPriorityQueue::Element *&elV = elements[e.v];
                if(elV) elV->decreaseKey({c_, e.v});
                else elV = &Q.push({c_, e.v});
                distV = c_;
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

bool Dijkstra::hasVisited(Node u) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (dist.at(u) != Edge::WEIGHT_INF);
#pragma GCC diagnostic pop
}
