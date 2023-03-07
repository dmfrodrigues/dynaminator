#include "shortest-path/Dijkstra.hpp"

#include <queue>
#include <utility>
#include <chrono>

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;
template<class K, class V> using umap = std::unordered_map<K, V>;
typedef umap<Node, Weight> dist_t;
typedef umap<Node, Node  > prev_t;
typedef std::priority_queue<std::pair<Weight, Node>,
                std::vector<std::pair<Weight, Node>>,
               std::greater<std::pair<Weight, Node>>> min_priority_queue;
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
    min_priority_queue Q;
    dist[s] = 0; Q.push(mk(dist[s], s));
    while(!Q.empty()){
        Node u = Q.top().second;
        Q.pop();
        for(const Edge &e: G->getAdj(u)){
            Weight c_ = dist[u] + e.w;
            if(c_ < dist[e.v]){
                dist[e.v] = c_;
                prev[e.v] = e;
                Q.push(mk(dist[e.v], e.v));
            }
        }
    }
}

Edge Dijkstra::getPrev(Node d) const{
    return prev.at(d);
}

Weight Dijkstra::getPathWeight(Node d) const{
    return dist.at(d);
}

bool Dijkstra::hasVisited(Node u) const{
    return (dist.at(u) != Edge::WEIGHT_INF);
}
