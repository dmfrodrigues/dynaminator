#include "Graph.hpp"

const Graph::Edge Graph::EDGE_INVALID = {-1, NODE_INVALID, NODE_INVALID, 0};

bool Graph::Edge::operator==(const Edge &e) const {
    return
        id == e.id &&
        u  == e.u  &&
        v  == e.v  &&
        w  == e.w
    ;
}

void Graph::addNode(Node u){
    nodes.push_back(u);
    adj[u];
}

void Graph::addEdge(Edge::Id id, Node u, Node v, Edge::Weight c){
    adj[u].push_back({id, u, v, c});
}

const std::vector<Graph::Node> &Graph::getNodes() const {
    return nodes;
}

const std::vector<Graph::Edge> &Graph::getAdj(Node u) const {
    return adj.at(u);
}
