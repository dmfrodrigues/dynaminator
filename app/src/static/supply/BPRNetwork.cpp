#include "static/supply/BPRNetwork.hpp"

#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdio>

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

typedef tuple<
    BPRNetwork *,
    unordered_map<SumoNetwork::Junction::Id, Node>,
    unordered_map<SumoNetwork::Junction::Id, pair<Node, Node>>,
    unordered_map<SumoNetwork::Edge::Id, Edge::Id>
>
    Tuple;

BPRNetwork::BPRNetwork(double alpha_, double beta_) : alpha(alpha_), beta(beta_) {}

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

Cost BPRNetwork::calculateCongestion(Edge::Id id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return f / e->c;
}

Tuple BPRNetwork::fromSumo(const SumoNetwork &sumoNetwork, const SumoTAZs &sumoTAZs) {
    BPRNetwork *network = new BPRNetwork();
    unordered_map<SumoNetwork::Junction::Id, Node> nodeStr2id;
    unordered_map<SumoNetwork::Junction::Id, pair<Node, Node>> str2id_taz;
    unordered_map<SumoNetwork::Junction::Id, Edge::Id> edgeStr2id;

    Node nodeCounter = 1;

    const vector<SumoNetwork::Junction> &junctions = sumoNetwork.getJunctions();
    for (const SumoNetwork::Junction &j : junctions) {
        nodeStr2id[j.id] = nodeCounter;
        network->addNode(nodeCounter);
        ++nodeCounter;
    }

    Edge::Id edgeCounter = 1;

    const std::vector<SumoNetwork::Edge> &edges = sumoNetwork.getEdges();
    for (const SumoNetwork::Edge &e : edges) {
        if (e.function == SumoNetwork::Edge::Function::INTERNAL) continue;
        if (
            e.from == SumoNetwork::Junction::INVALID ||
            e.to == SumoNetwork::Junction::INVALID) {
            cerr << "Edge " << e.id << " is invalid, failed to build BPRNetwork" << endl;
            delete network;
            return Tuple(nullptr, nodeStr2id, str2id_taz, edgeStr2id);
        }

        double length = 0, averageSpeed = 0;
        for (const auto &p : e.lanes) {
            length += p.second.length;
            averageSpeed += p.second.speed;
        }
        length /= double(e.lanes.size());
        averageSpeed /= double(e.lanes.size());

        double freeFlowSpeed = averageSpeed;
        double freeFlowTime = length / freeFlowSpeed;

        // Estimated using Greenshields' model. Sheffi (1985), p. 350.
        // This roughly agrees with the Highway Capacity Manual (p. 10-24, 15-9)
        const double jamDensity = 1.0 / 8.0;
        double capacity = 0.25 * freeFlowSpeed * jamDensity;

        network->addEdge(edgeCounter, nodeStr2id.at(e.from), nodeStr2id.at(e.to), freeFlowTime, capacity);
        edgeStr2id[e.id] = edgeCounter;
        ++edgeCounter;
    }

    const vector<SumoTAZs::TAZ> tazs = sumoTAZs.getTAZs();
    for (const SumoTAZs::TAZ &taz : tazs) {
        Node source = nodeCounter++;
        for (const SumoTAZs::TAZ::Source &s : taz.sources) {
            const Edge *e = network->edges.at(edgeStr2id.at(s.id));
            network->addEdge(edgeCounter++, source, e->u, 0, 1e9);
            network->addEdge(edgeCounter++, source, e->v, 0, 1e9);
        }
        Node sink = nodeCounter++;
        for (const SumoTAZs::TAZ::Sink &s : taz.sinks) {
            const Edge *e = network->edges.at(edgeStr2id.at(s.id));
            network->addEdge(edgeCounter++, e->u, sink, 0, 1e9);
            network->addEdge(edgeCounter++, e->v, sink, 0, 1e9);
        }
        str2id_taz[taz.id] = make_pair(source, sink);
    }

    return Tuple(network, nodeStr2id, str2id_taz, edgeStr2id);
}

void BPRNetwork::saveResultsToFile(
    const StaticSolution &x,
    const unordered_map<
        SumoNetwork::Edge::Id,
        StaticNetwork::Edge::Id
    > &edgeStr2id,
    const string &path
) const {
    unordered_map<StaticNetwork::Edge::Id, SumoNetwork::Edge::Id> edgeId2str;
    for(const auto &p: edgeStr2id)
        edgeId2str[p.second] = p.first;

    xml_document<> doc;
    auto meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    vector<Edge::Id> edgesVtr = x.getEdges();
    for (const Edge::Id &e : edgesVtr) {
        double f = x.getFlowInEdge(e);
        double c = calculateCongestion(e, f);

        auto it = edgeId2str.find(e);
        if(it == edgeId2str.end()) continue;

        char *fs = new char[256]; sprintf(fs, "%lf", f);
        char *cs = new char[256]; sprintf(cs, "%lf", c);

        auto edge = doc.allocate_node(node_element, "edge");
        edge->append_attribute(doc.allocate_attribute("id", it->second.c_str()));
        edge->append_attribute(doc.allocate_attribute("flow", fs));
        edge->append_attribute(doc.allocate_attribute("congestion", cs));
        interval->append_node(edge);
    }

    ofstream os(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}