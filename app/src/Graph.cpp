#include "Graph.hpp"

using namespace std;

const Graph::Edge::Weight Graph::Edge::WEIGHT_INF = 1.0e15;

const Graph::Edge Graph::EDGE_INVALID = {-1, NODE_INVALID, NODE_INVALID, 0};

void Graph::addNode(Node u){
    adj[u];
}

void Graph::addEdge(Edge::ID id, Node u, Node v, Edge::Weight c){
    adj[u].push_back({id, u, v, c});
    adj[v];
}

vector<Graph::Node> Graph::getNodes() const {
    vector<Graph::Node> ret;
    for(const auto &p: adj)
        ret.push_back(p.first);

    return ret;
}

const vector<Graph::Edge> &Graph::getAdj(Node u) const {
    return adj.at(u);
}
