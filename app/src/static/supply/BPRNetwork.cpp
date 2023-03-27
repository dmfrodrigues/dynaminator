#include "static/supply/BPRNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include "data/SumoAdapterStatic.hpp"
#include "static/StaticSolution.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#pragma GCC diagnostic pop

using namespace std;
using namespace rapidxml;

typedef BPRNetwork::Node Node;
typedef BPRNetwork::Edge Edge;
typedef BPRNetwork::Flow Flow;
typedef BPRNetwork::Cost Cost;

typedef SumoNetwork::Edge::Lane Lane;

typedef pair<
    BPRNetwork *,
    SumoAdapterStatic>
    Tuple;

BPRNetwork::BPRNetwork(Flow alpha_, Flow beta_) : alpha(alpha_), beta(beta_) {}

void BPRNetwork::addNode(Node u) {
    adj[u];
}

void BPRNetwork::addEdge(Edge::Id id, Node u, Node v, Time t0, Capacity c) {
    CustomEdge *e = new CustomEdge{id, u, v, t0, c};
    adj[u].push_back(e);
    adj[v];
    edges[id] = e;
}

std::vector<Node> BPRNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for (const auto &p : adj)
        ret.push_back(p.first);
    return ret;
}

std::vector<Edge *> BPRNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Edge *>(v.begin(), v.end());
}

Cost BPRNetwork::calculateCost(Edge::Id id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return e->t0 * (1.0 + alpha * pow(f / e->c, beta));
}

Cost BPRNetwork::calculateCostGlobal(Edge::Id id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return e->t0 * f * ((alpha/(beta+1.0)) * pow(f/e->c, beta) + 1.0);
}

Cost BPRNetwork::calculateCongestion(Edge::Id id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return f / e->c;
}

Tuple BPRNetwork::fromSumo(const SumoNetwork &sumoNetwork, const SumoTAZs &sumoTAZs) {
    BPRNetwork *network = new BPRNetwork();
    SumoAdapterStatic adapter;

    const vector<SumoNetwork::Junction> &junctions = sumoNetwork.getJunctions();
    for (const SumoNetwork::Junction &j : junctions) {
        Node u = adapter.addSumoJunction(j.id);
        network->addNode(u);
    }

    const std::vector<SumoNetwork::Edge> &edges = sumoNetwork.getEdges();
    for (const SumoNetwork::Edge &e : edges) {
        if (e.function == SumoNetwork::Edge::Function::INTERNAL) continue;
        if (
            e.from == SumoNetwork::Junction::INVALID ||
            e.to == SumoNetwork::Junction::INVALID) {
            cerr << "Edge " << e.id << " is invalid, failed to build BPRNetwork" << endl;
            delete network;
            return Tuple(nullptr, adapter);
        }

        Lane::Length length = 0;
        Lane::Speed averageSpeed = 0;
        for (const auto &p : e.lanes) {
            length += p.second.length;
            averageSpeed += p.second.speed;
        }
        length /= Lane::Length(e.lanes.size());
        averageSpeed /= Lane::Speed(e.lanes.size());

        Lane::Speed freeFlowSpeed = averageSpeed;
        Cost freeFlowTime = length / freeFlowSpeed;

        // Estimated using Greenshields' model. Sheffi (1985), p. 350.
        // This roughly agrees with the Highway Capacity Manual (p. 10-24, 15-9)
        const Cost jamDensity = 1.0 / 8.0;
        Cost capacity = 0.25 * freeFlowSpeed * jamDensity * (Cost)e.lanes.size();

        Edge::Id eid = adapter.addSumoEdge(e.id);
        network->addEdge(eid, adapter.toNode(e.from), adapter.toNode(e.to), freeFlowTime, capacity);
    }

    const vector<SumoTAZs::TAZ> tazs = sumoTAZs.getTAZs();
    for (const SumoTAZs::TAZ &taz : tazs) {
        auto p = adapter.addSumoTAZ(taz.id);
        Node source = p.first;
        for (const SumoTAZs::TAZ::Source &s : taz.sources) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            network->addEdge(
                adapter.addSumoEdge(),
                source, e->u, 0, 1e9);
            // network->addEdge(
            //     adapter.addSumoEdge(),
            //     source, e->v, 0, 1e9);
        }
        Node sink = p.second;
        for (const SumoTAZs::TAZ::Sink &s : taz.sinks) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            // network->addEdge(
            //     adapter.addSumoEdge(),
            //     e->u, sink, 0, 1e9);
            network->addEdge(
                adapter.addSumoEdge(),
                e->v, sink, 0, 1e9);
        }
    }

    return Tuple(network, adapter);
}

void BPRNetwork::saveResultsToFile(
    const StaticSolution &x,
    const SumoAdapterStatic &adapter,
    const string &path) const {
    xml_document<> doc;
    auto meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    for (const auto &p : edges) {
        Edge::Id e = p.first;

        Flow f = x.getFlowInEdge(e);
        Cost c = calculateCongestion(e, f);

        try {
            const SumoNetwork::Edge::Id &eid = adapter.toSumoEdge(e);

            char *fs = new char[256];
            sprintf(fs, "%lf", f);
            char *cs = new char[256];
            sprintf(cs, "%lf", c);

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id", eid.c_str()));
            edge->append_attribute(doc.allocate_attribute("flow", fs));
            edge->append_attribute(doc.allocate_attribute("congestion", cs));
            interval->append_node(edge);
        } catch (const out_of_range &ex) {
            cerr << "Could not find SUMO edge corresponding to edge " << e << ", ignoring" << endl;
        }
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    os.open(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
