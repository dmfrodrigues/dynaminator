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

typedef SUMO::Network::Edge::Lane Lane;

typedef pair<
    BPRNetwork *,
    SumoAdapterStatic>
    Tuple;

BPRNetwork::BPRNetwork(Flow alpha_, Flow beta_) : alpha(alpha_), beta(beta_) {}

void BPRNetwork::addNode(Node u) {
    adj[u];
}

void BPRNetwork::addEdge(Edge::ID id, Node u, Node v, Time t0, Capacity c) {
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

Cost BPRNetwork::calculateCost(Edge::ID id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return e->t0 * (1.0 + alpha * pow(f / e->c, beta));
}

Cost BPRNetwork::calculateCostGlobal(Edge::ID id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return e->t0 * f * ((alpha/(beta+1.0)) * pow(f/e->c, beta) + 1.0);
}

Cost BPRNetwork::calculateCongestion(Edge::ID id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return f / e->c;
}

Cost BPRNetwork::calculateDelay(Edge::ID id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return 1.0 + alpha * pow(f / e->c, beta);
}

Cost calculateLength(const SUMO::Network::Edge &e){
    Lane::Length length = 0;
    for (const auto &p : e.lanes) {
        length += p.second.length;
    }
    length /= Lane::Length(e.lanes.size());
    return length;
}

Cost calculateSpeed(const SUMO::Network::Edge &e){
    Lane::Speed averageSpeed = 0;
    for (const auto &p : e.lanes) {
        averageSpeed += p.second.speed;
    }
    averageSpeed /= Lane::Speed(e.lanes.size());
    return averageSpeed;
}

Cost calculateFreeFlowSpeed(const SUMO::Network::Edge &e){
    return calculateSpeed(e) * 0.9;
}

Cost calculateFreeFlowTime(const SUMO::Network::Edge &e){
    Lane::Length length = calculateLength(e);
    Lane::Speed freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost freeFlowTime = length / freeFlowSpeed;
    return freeFlowTime;
}

const Cost SATURATION_FLOW = 1110.0; // vehicles per hour per lane

Cost calculateCapacity(const SUMO::Network::Edge &e){
    Lane::Speed freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost capacity = (SATURATION_FLOW/60.0/60.0) * (freeFlowSpeed/(50.0/3.6)) * (Cost)e.lanes.size();
    return capacity;
}

Tuple BPRNetwork::fromSumo(const SUMO::Network &sumoNetwork, const SumoTAZs &sumoTAZs) {
    BPRNetwork *network = new BPRNetwork();
    SumoAdapterStatic adapter;

    map<SUMO::Network::Junction::ID, list<SUMO::Network::Edge>> adj;

    const vector<SUMO::Network::Edge> &edges = sumoNetwork.getEdges();
    for (const SUMO::Network::Edge &edge: edges) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const auto &p = adapter.addSumoEdge(edge.id);
        const Edge::ID &eid = p.first;
        Node u = p.second.first, v = p.second.second;

        network->addNode(u);
        network->addNode(v);
        network->addEdge(
            eid,
            u,
            v,
            calculateFreeFlowTime(edge),
            calculateCapacity(edge)
        );

        adj[edge.from].push_back(edge);
    }

    for (const SUMO::Network::Edge &edge : edges) {
        if (edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;
        if (
            edge.from == SUMO::Network::Junction::INVALID ||
            edge.to == SUMO::Network::Junction::INVALID) {
            cerr << "Edge " << edge.id << " is invalid, failed to build BPRNetwork" << endl;
            delete network;
            return Tuple(nullptr, adapter);
        }

        for(const SUMO::Network::Edge &nextEdge: adj[edge.to]){
            if(nextEdge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

            network->addEdge(
                adapter.addEdge(),
                network->edges.at(adapter.toEdge(edge.id))->v,
                network->edges.at(adapter.toEdge(nextEdge.id))->u,
                0,
                1e9
            );
        }
    }

    const vector<SumoTAZs::TAZ> tazs = sumoTAZs.getTAZs();
    for (const SumoTAZs::TAZ &taz : tazs) {
        auto p = adapter.addSumoTAZ(taz.id);
        Node source = p.first;
        for (const SumoTAZs::TAZ::Source &s : taz.sources) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            network->addEdge(
                adapter.addEdge(),
                source, e->u, 0, 1e9);
        }
        Node sink = p.second;
        for (const SumoTAZs::TAZ::Sink &s : taz.sinks) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            network->addEdge(
                adapter.addEdge(),
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
        Edge::ID e = p.first;

        Flow f = x.getFlowInEdge(e);
        Cost c = calculateCongestion(e, f);
        Cost d = calculateDelay(e, f);

        try {
            const SUMO::Network::Edge::ID &eid = adapter.toSumoEdge(e);

            char *fs = new char[256];
            sprintf(fs, "%lf", f);
            char *cs = new char[256];
            sprintf(cs, "%lf", c);
            char *ds = new char[256];
            sprintf(ds, "%lf", d);

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id", eid.c_str()));
            edge->append_attribute(doc.allocate_attribute("flow", fs));
            edge->append_attribute(doc.allocate_attribute("congestion", cs));
            edge->append_attribute(doc.allocate_attribute("delay", ds));
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
