#include "Static/supply/BPRNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/Routes.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wconversion"
#include <color/color.hpp>
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#pragma GCC diagnostic pop

#include "Static/Solution.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "data/SUMO/TAZ.hpp"
#include "data/SumoAdapterStatic.hpp"
#include "utils/stringify.hpp"
#include "utils/timer.hpp"
#include "utils/xml.hpp"

using namespace std;
using namespace rapidxml;
using namespace Static;
using namespace utils::stringify;

namespace fs  = std::filesystem;
namespace xml = utils::xml;

typedef BPRNetwork::Node           Node;
typedef BPRNetwork::Edge           Edge;
typedef BPRNetwork::NormalEdge     NormalEdge;
typedef BPRNetwork::ConnectionEdge ConnectionEdge;
typedef BPRNetwork::Flow           Flow;
typedef BPRNetwork::Time           Time;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;

BPRNetwork::Edge::Edge(Edge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    NetworkDifferentiable::Edge(id_, u_, v_),
    network(network_),
    t0(t0_),
    c(c_) {}

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

BPRNetwork::BPRNetwork(Flow alpha_, Flow beta_):
    alpha(alpha_), beta(beta_) {}

void BPRNetwork::addNode(Node u) {
    adj[u];
}

void BPRNetwork::addEdge(Edge *e) {
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
}

std::vector<Node> BPRNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &[u, _]: adj)
        ret.push_back(u);
    return ret;
}

Edge &BPRNetwork::getEdge(Edge::ID e) const {
    return *edges.at(e);
}

std::vector<Network::Edge *> BPRNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Network::Edge *>(v.begin(), v.end());
}

void BPRNetwork::saveResultsToFile(
    const SUMO::NetworkTAZs &sumo,
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &edgeDataPath,
    const string            &routesPath
) const {
    // clang-format off
    SUMO::EdgeData::Loader<
        const SUMO::NetworkTAZs &,
        const Static::BPRNetwork &,
        const Static::Solution &,
        const SumoAdapterStatic &
    > edgeDataLoader;
    // clang-format on
    unique_ptr<SUMO::EdgeData> edgeData(
        edgeDataLoader.load(
            sumo,
            *this,
            x,
            adapter
        )
    );
    edgeData->saveToFile(edgeDataPath);

    // clang-format off
    SUMO::Routes::Loader<
        const Static::Network &,
        const Static::Solution &,
        const SumoAdapterStatic &
    > routesLoader;
    // clang-format on
    unique_ptr<SUMO::Routes> routes(
        routesLoader.load(
            *this,
            x,
            adapter
        )
    );
    routes->saveToFile(routesPath);
}
