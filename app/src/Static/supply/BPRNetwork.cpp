#include "Static/supply/BPRNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/NetworkTAZ.hpp"

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

using namespace std;
using namespace rapidxml;
using namespace Static;
using namespace utils::stringify;

namespace fs = std::filesystem;

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

template<typename T>
void add_attribute(xml_node<> &xmlNode, const char *key, const T &val) {
    xml_document<> &doc  = *xmlNode.document();
    auto            attr = doc.allocate_attribute(
        key,
        doc.allocate_string(
            stringify<T>::toString(val).c_str()
        )
    );
    xmlNode.append_attribute(attr);
}

void BPRNetwork::saveRoutes(
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &filePath
) const {
    xml_document<> doc;
    xml_node<>    &routesEl = *doc.allocate_node(node_element, "routes");
    doc.append_node(&routesEl);

    const Static::Solution::Routes &routes = x.getRoutes();

    map<pair<SUMO::TAZ::ID, SUMO::TAZ::ID>, vector<pair<Flow, SUMO::Route>>> allRoutes;

    size_t numberFlows = 0;
    Flow   maxFlow     = 0.0;

    for(const auto &[path, flow]: routes) {
        SUMO::Route route;
        for(const NormalEdge::ID &eid: path) {
            if(adapter.isSumoEdge(eid))
                route.push_back(adapter.toSumoEdge(eid));
        }
        const SUMO::TAZ::ID &fromTaz = adapter.toSumoTAZ(edges.at(*path.begin())->u);
        const SUMO::TAZ::ID &toTaz   = adapter.toSumoTAZ(edges.at(*path.rbegin())->v);

        allRoutes[{fromTaz, toTaz}].push_back({flow, route});
        ++numberFlows;
        maxFlow = max(maxFlow, flow);
    }

    size_t flowID = 0;
    for(const auto &[fromTo, fromToRoutes]: allRoutes) {
        const auto &[fromTaz, toTaz] = fromTo;

        const float MIN_INTENSITY     = 0.5;
        const float MAX_INTENSITY     = 1.5;
        const float DEFAULT_INTENSITY = 1.0;

        for(size_t i = 0; i < fromToRoutes.size(); ++i) {
            const auto &[flow, route] = fromToRoutes[i];

            float intensity = (fromToRoutes.size() <= 1 ? DEFAULT_INTENSITY : MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * float(i) / float(fromToRoutes.size() - 1));

            float h = 360.0f * float(flowID) / float(numberFlows);
            float v = 100.0f * min(intensity, 1.0f);
            float s = 100.0f * (1.0f - max(intensity - 1.0f, 0.0f));

            color::hsv<float> colorHSV({h, s, v});
            color::rgb<float> colorRGB;
            colorRGB = colorHSV;

            xml_node<> &flowEl = *doc.allocate_node(node_element, "flow");
            routesEl.append_node(&flowEl);

            add_attribute(flowEl, "id", flowID++);
            add_attribute(flowEl, "color", colorRGB);
            add_attribute(flowEl, "begin", "0"s);
            add_attribute(flowEl, "end", "3600"s);
            add_attribute(flowEl, "fromTaz", fromTaz);
            add_attribute(flowEl, "toTaz", toTaz);
            add_attribute(flowEl, "vehsPerHour", flow * 60 * 60);
            add_attribute(flowEl, "departPos", "random_free"s);
            add_attribute(flowEl, "departSpeed", "random"s);

            xml_node<> &routeEl = *doc.allocate_node(node_element, "route");
            flowEl.append_node(&routeEl);

            add_attribute(routeEl, "edges", route);
        }
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    fs::path p = fs::path(filePath).parent_path();
    if(!fs::is_directory(p)) {
        cerr << "Creating directory " << p << endl;
        if(!fs::create_directory(p)) {
            throw ios_base::failure("Could not create directory " + p.string());
        }
    }
    try {
        os.open(filePath);
    } catch(const ios_base::failure &ex) {
        throw ios_base::failure("Could not open file " + filePath);
    }

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
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

    // saveEdges(sumo, x, adapter, edgeDataPath);
    saveRoutes(x, adapter, routesPath);
}
