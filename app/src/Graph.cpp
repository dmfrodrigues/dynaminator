#include "Graph.hpp"

void Graph::addNode(Node u){
    nodes.push_back(u);
    adj[u];
}

void Graph::addEdge(Node u, Node v, Edge::Weight c){
    adj[u].push_back({v, c});
}

const std::vector<Graph::Node> &Graph::getNodes() const {
    return nodes;
}

const std::vector<Graph::Edge> &Graph::getAdj(Node u) const {
    return adj.at(u);
}
