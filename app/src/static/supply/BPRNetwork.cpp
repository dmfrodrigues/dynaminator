#include "static/supply/BPRNetwork.hpp"

using namespace std;

typedef BPRNetwork::Node Node;
typedef BPRNetwork::Edge Edge;
typedef BPRNetwork::Flow Flow;
typedef BPRNetwork::Cost Cost;

std::vector<Node> BPRNetwork::getNodes() const {
    vector<Node> ret;
    for(const auto &p: adj)
        ret.push_back(p.first);

    return ret;
}

std::vector<Edge *> BPRNetwork::getAdj(Node u) const {
    return adj.at(u);
}
