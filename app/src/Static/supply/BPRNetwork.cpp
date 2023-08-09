#include "Static/supply/BPRNetwork.hpp"

#include <cmath>
#include <color/color.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <stdexcept>

#include "Static/SUMOAdapter.hpp"
#include "Static/Solution.hpp"
#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/Routes.hpp"
#include "data/SUMO/SUMO.hpp"
#include "data/SUMO/TAZ.hpp"
#include "utils/stringify.hpp"
#include "utils/timer.hpp"
#include "utils/xml.hpp"

using namespace std;
using namespace rapidxml;
using namespace Static;
using namespace utils::stringify;

typedef BPRNetwork::Node           Node;
typedef BPRNetwork::Edge           Edge;
typedef BPRNetwork::NormalEdge     NormalEdge;
typedef BPRNetwork::ConnectionEdge ConnectionEdge;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;

BPRNetwork::Edge::Edge(Edge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    NetworkDifferentiable::Edge(id_, u_, v_),
    network(network_),
    t0(t0_),
    c(c_) {}

BPRNetwork::Edge::~Edge() {}

Time BPRNetwork::Edge::calculateCongestion(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return f / c;
}

BPRNetwork::NormalEdge::NormalEdge(NormalEdge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    Edge(id_, u_, v_, network_, t0_, c_) {}

Time BPRNetwork::NormalEdge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * (1.0 + network.alpha * pow(f / c, network.beta));
}

Time BPRNetwork::NormalEdge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);
}

Time BPRNetwork::NormalEdge::calculateCostDerivative(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);
}

BPRNetwork::ConnectionEdge::ConnectionEdge(ConnectionEdge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    Edge(id_, u_, v_, network_, t0_, c_) {}

Time BPRNetwork::ConnectionEdge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * (1.0 + network.alpha * pow(f / c, network.beta));
}

Time BPRNetwork::ConnectionEdge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);
}

Time BPRNetwork::ConnectionEdge::calculateCostDerivative(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);
}

BPRNetwork::BPRNetwork(Time alpha_, Time beta_):
    alpha(alpha_), beta(beta_) {}

void BPRNetwork::addNode(Node u) {
    adj[u];
}

NormalEdge *BPRNetwork::addNormalEdge(
    Edge::ID          id,
    Node              u,
    Node              v,
    const BPRNetwork &network,
    Time              t0,
    Flow              c
) {
    NormalEdge *e = new NormalEdge(id, u, v, network, t0, c);
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
    return e;
}

ConnectionEdge *BPRNetwork::addConnectionEdge(
    Edge::ID          id,
    Node              u,
    Node              v,
    const BPRNetwork &network,
    Time              t0,
    Flow              c
) {
    ConnectionEdge *e = new ConnectionEdge(id, u, v, network, t0, c);
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
    return e;
}

vector<Node> BPRNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &[u, _]: adj)
        ret.push_back(u);
    return ret;
}

Edge &BPRNetwork::getEdge(Edge::ID e) const {
    return *edges.at(e);
}

vector<Network::Edge *> BPRNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Network::Edge *>(v.begin(), v.end());
}

BPRNetwork::~BPRNetwork() {
    for(const auto &[_, e]: edges)
        delete e;
}
