#include "static/StaticDemand.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Flow Flow;

void StaticDemand::addDemand(Node u, Node v, Flow f) {
    flows[u][v] += f;
}

vector<Node> StaticDemand::getStartNodes() const {
    vector<Node> ret;
    ret.reserve(flows.size());

    for (const auto &p : flows)
        ret.push_back(p.first);

    return ret;
}

vector<Node> StaticDemand::getDestinations(Node u) const {
    const auto &dest = flows.at(u);

    vector<Node> ret;
    ret.reserve(dest.size());

    for (const auto &p : dest)
        ret.push_back(p.first);

    return ret;
}

Flow StaticDemand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}
