#include "Static/supply/CustomNetwork.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

#include "data/SumoAdapterStatic.hpp"
#include "Static/Solution.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#pragma GCC diagnostic pop

using namespace std;
using namespace rapidxml;
using namespace Static;

typedef CustomNetwork::Node Node;
typedef CustomNetwork::Edge Edge;
typedef CustomNetwork::Cost Cost;

void CustomNetwork::addNode(Node u) {
    adj[u];
}

void CustomNetwork::addEdge(Edge::ID id, Node u, Node v, CostFunction f, CostFunction fGlobal) {
    CustomEdge *e = new CustomEdge{id, u, v, f, fGlobal};
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

vector<Edge *> CustomNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Edge *>(v.begin(), v.end());
}

Cost CustomNetwork::calculateCost(Edge::ID id, Flow f) const {
    return edges.at(id)->cost(f);
}

Cost CustomNetwork::calculateCostGlobal(Edge::ID id, Flow f) const {
    return edges.at(id)->costGlobal(f);
}

CustomNetwork::~CustomNetwork() {
    for(const auto &p: edges)
        delete p.second;
}

void CustomNetwork::saveResultsToFile(
    const Solution &x,
    const SumoAdapterStatic &adapter,
    const string &edgeDataPath,
    const string &
) const {
    xml_document<> doc;
    auto meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    for(const auto &[e, _]: edges) {
        Flow f = x.getFlowInEdge(e);

        try {
            const SUMO::Network::Edge::ID &eid = adapter.toSumoEdge(e);

            char *fs = new char[256];
            sprintf(fs, "%lf", f);

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id", eid.c_str()));
            edge->append_attribute(doc.allocate_attribute("flow", fs));
            interval->append_node(edge);
        } catch(const out_of_range &ex) {
            cerr << "Could not find SUMO edge corresponding to edge " << e << ", ignoring" << endl;
        }
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    os.open(edgeDataPath);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
