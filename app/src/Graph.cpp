#include "Graph.hpp"

void Graph::addNode(long u){
    adj[u];
}

void Graph::addEdge(long u, long v, double c){
    adj[u].push_back({v, c});
}

const std::vector<Graph::Edge> &Graph::getAdj(long u) const {
    return adj.at(u);
}
