#include "Static/supply/CustomNetwork.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#pragma GCC diagnostic pop

#include "Static/SUMOAdapter.hpp"
#include "Static/Solution.hpp"

using namespace std;
using namespace rapidxml;
using namespace Static;

typedef CustomNetwork::Node Node;
typedef CustomNetwork::Edge Edge;

CustomNetwork::Edge::Edge(ID id_, Node u_, Node v_, CostFunction f_, CostFunction fGlobal_):
    Network::Edge(id_, u_, v_),
    cost(f_),
    costGlobal(fGlobal_) {}

Time CustomNetwork::Edge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return cost(f);
}

Time CustomNetwork::Edge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return costGlobal(f);
}

void CustomNetwork::addNode(Node u) {
    adj[u];
}

void CustomNetwork::addEdge(Edge::ID id, Node u, Node v, CostFunction f, CostFunction fGlobal) {
    Edge *e = new Edge(id, u, v, f, fGlobal);
    adj[u].push_back(e);
    edges[id] = e;
}

vector<Node> CustomNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &[u, _]: adj)
        ret.push_back(u);
    return ret;
}

Edge &CustomNetwork::getEdge(Edge::ID e) const {
    return *edges.at(e);
}

vector<Network::Edge *> CustomNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Network::Edge *>(v.begin(), v.end());
}

CustomNetwork::~CustomNetwork() {
    for(const auto &p: edges)
        delete p.second;
}
