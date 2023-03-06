#include "static/supply/CustomStaticNetwork.hpp"

using namespace std;

typedef CustomStaticNetwork::Node Node;
typedef CustomStaticNetwork::Edge Edge;
typedef CustomStaticNetwork::Edge::Id EdgeId;
typedef CustomStaticNetwork::Cost Cost;

void CustomStaticNetwork::addNode(Node u){
    adj[u];
}

void CustomStaticNetwork::addEdge(EdgeId id, Node u, Node v, CostFunction f){
    CustomEdge *e = new CustomEdge{id, v, f};
    adj[u].push_back(e);
    edges[id] = e;
}

vector<Node> CustomStaticNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &p: adj)
        ret.push_back(p.first);
    return ret;
}

vector<Edge *> CustomStaticNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Edge *>(v.begin(), v.end());
}

Cost CustomStaticNetwork::calculateCost(EdgeId id, Flow f) const {
    return edges.at(id)->cost(f);
}
