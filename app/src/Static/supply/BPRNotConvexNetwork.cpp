#include "Static/supply/BPRNotConvexNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

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
#include "Static/supply/BPRConvexNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/SUMO.hpp"
#include "data/SUMO/TAZ.hpp"
#include "Static/SUMOAdapter.hpp"
#include "utils/stringify.hpp"
#include "utils/timer.hpp"

using namespace std;
using namespace rapidxml;
using namespace Static;
using namespace utils::stringify;

typedef BPRNotConvexNetwork::Node           Node;
typedef BPRNotConvexNetwork::Edge           Edge;
typedef BPRNotConvexNetwork::NormalEdge     NormalEdge;
typedef BPRNotConvexNetwork::ConnectionEdge ConnectionEdge;
typedef BPRNotConvexNetwork::Flow           Flow;
typedef BPRNotConvexNetwork::Time           Time;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;

const double T_CR = 5.0;

BPRNotConvexNetwork::NormalEdge::NormalEdge(NormalEdge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    BPRNetwork::NormalEdge(id_, u_, v_, network_, t0_, c_) {}

BPRNotConvexNetwork::ConnectionEdge::ConnectionEdge(ConnectionEdge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Flow c_):
    BPRNetwork::ConnectionEdge(id_, u_, v_, network_, t0_, c_) {}

const Flow EPSILON_FLOW = 1.0e-5;
const Flow EPSILON_TIME = 1.0e-3;
const Time CAPACITY_INF = 1.0e+6;

Time BPRNotConvexNetwork::ConnectionEdge::getLessPriorityCapacity(const Solution &x) const {
    if(conflicts.empty()) return CAPACITY_INF;

    Flow totalCapacity = 0.0;

    for(const vector<pair<const Edge *, double>> &v: conflicts) {
        Flow lambda = 0.0;
        for(const auto &[e, p]: v) {
            lambda += x.getFlowInEdge(e->id) * p;
        }
        if(lambda < EPSILON_FLOW) return CAPACITY_INF;
        double EW = (exp(lambda * T_CR) - 1.0) / lambda - T_CR;
        if(EW < EPSILON_TIME) return CAPACITY_INF;
        totalCapacity += 1.0 / EW;
    }

    return max(totalCapacity, 1.0 / 60.0);
}

Time BPRNotConvexNetwork::ConnectionEdge::calculateCost(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Time t1 = t0 * (1.0 + network.alpha * pow(f / c, network.beta));

    Flow c2 = getLessPriorityCapacity(x);
    Time t2 = 0.0;
    if(c2 < CAPACITY_INF) {
        Time fft = 1.0 / c2;
        t2       = fft * (1.0 + network.alpha * pow(f / c2, network.beta));
    }

    return t1 + t2;
}

Time BPRNotConvexNetwork::ConnectionEdge::calculateCostGlobal(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Time t1 = t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);

    Flow c2 = getLessPriorityCapacity(x);
    Time t2 = 0.0;
    if(c2 < CAPACITY_INF) {
        Time fft = 1.0 / c2;
        t2       = fft * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c2, network.beta) + 1.0);
    }

    return t1 + t2;
}

Time BPRNotConvexNetwork::ConnectionEdge::calculateCostDerivative(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Time t1 = t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);

    Flow c2 = getLessPriorityCapacity(x);
    Time t2 = 0.0;
    if(c2 < CAPACITY_INF) {
        Time fft = 1.0 / c2;
        t2       = fft * network.alpha * network.beta * pow(f / c2, network.beta - 1);
    }

    return t1 + t2;
}

BPRNotConvexNetwork::BPRNotConvexNetwork(Flow alpha_, Flow beta_):
    BPRNetwork(alpha_, beta_) {}

NormalEdge *BPRNotConvexNetwork::addNormalEdge(
    Edge::ID id,
    Node u,
    Node v,
    const BPRNetwork &network,
    Time t0,
    Flow c
) {
    NormalEdge *e = new NormalEdge(id, u, v, network, t0, c);
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
    return e;
}

ConnectionEdge *BPRNotConvexNetwork::addConnectionEdge(
    Edge::ID id,
    Node u,
    Node v,
    const BPRNetwork &network,
    Time t0,
    Flow c
) {
    ConnectionEdge *e = new ConnectionEdge(id, u, v, network, t0, c);
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
    return e;
}

BPRConvexNetwork BPRNotConvexNetwork::makeConvex(const Solution &x) const {
    return BPRConvexNetwork(*this, x);
}
