#include "shortest-path/Dijkstra.hpp"

#include <queue>
#include <utility>
#include <chrono>

using namespace std;

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;
template<class K, class V> using umap = std::unordered_map<K, V>;
typedef umap<Node, Weight> dist_t;
typedef umap<Node, Node  > prev_t;
typedef std::priority_queue<std::pair<Weight, Node>,
                std::vector<std::pair<Weight, Node>>,
               std::greater<std::pair<Weight, Node>>> MinPriorityQueue;
typedef std::chrono::high_resolution_clock hrc;
#define mk(a, b) (std::make_pair((a), (b)))

Node Dijkstra::getStart() const{
    return s;
}

void Dijkstra::initialize(const Graph *G_, Node s_){
    this->s = s_;
    this->G = G_;
    for(const Node &u: G->getNodes()){
        dist[u] = Edge::WEIGHT_INF;
        prev[u] = Graph::EDGE_INVALID;
    }
}

void Dijkstra::run(){
    MinPriorityQueue Q;
    dist[s] = 0; Q.push(mk(dist[s], s));
    while(!Q.empty()){
        Node u = Q.top().second;
        Q.pop();
        for(const Edge &e: G->getAdj(u)){
            Weight c_ = dist.at(u) + e.w;
            Weight &distV = dist[e.v];
            if (c_ < distV) {
                // if(distV == Edge::WEIGHT_INF) elements[e.v] = &Q.push({c_, e.v});
                // else elements[e.v]->decreaseKey({c_, e.v});
                Q.push(mk(c_, e.v));
                distV = c_;
                prev[e.v] = e;
            }
        }
    }
}

Edge Dijkstra::getPrev(Node d) const{
    auto it = prev.find(d);
    if(it == prev.end()) return Graph::EDGE_INVALID;
    return it->second;
}

Weight Dijkstra::getPathWeight(Node d) const{
    return dist.at(d);
}

bool Dijkstra::hasVisited(Node u) const{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"
    return (dist.at(u) != Edge::WEIGHT_INF);
    #pragma GCC diagnostic pop
}
