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
#include "data/SumoAdapterStatic.hpp"
#include "utils/stringify.hpp"
#include "utils/timer.hpp"

using namespace std;
using namespace rapidxml;
using namespace Static;
using namespace utils::stringify;

namespace fs = std::filesystem;

typedef BPRNotConvexNetwork::Node           Node;
typedef BPRNotConvexNetwork::Edge           Edge;
typedef BPRNotConvexNetwork::Edges          Edges;
typedef BPRNotConvexNetwork::NormalEdge     NormalEdge;
typedef BPRNotConvexNetwork::ConnectionEdge ConnectionEdge;
typedef BPRNotConvexNetwork::Flow           Flow;
typedef BPRNotConvexNetwork::Cost           Cost;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;

const double T_CR = 5.0;

BPRNotConvexNetwork::Edge::Edge(Edge::ID id_, Node u_, Node v_, Capacity c_):
    NetworkDifferentiable::Edge(id_, u_, v_),
    c(c_) {}

BPRNotConvexNetwork::NormalEdge::NormalEdge(NormalEdge::ID id_, Node u_, Node v_, const BPRNotConvexNetwork &network_, Time t0_, Capacity c_):
    Edge(id_, u_, v_, c_),
    network(network_),
    t0(t0_) {}

Cost BPRNotConvexNetwork::NormalEdge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * (1.0 + network.alpha * pow(f / c, network.beta));
}

Cost BPRNotConvexNetwork::NormalEdge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);
}

Cost BPRNotConvexNetwork::NormalEdge::calculateCostDerivative(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);
}

Cost BPRNotConvexNetwork::NormalEdge::calculateCongestion(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return f / c;
}

BPRNotConvexNetwork::ConnectionEdge::ConnectionEdge(ConnectionEdge::ID id_, Node u_, Node v_, const BPRNotConvexNetwork &network_, Time t0_, Capacity c_):
    Edge(id_, u_, v_, c_),
    network(network_),
    t0(t0_) {}

const Flow EPSILON_FLOW = 1.0e-5;
const Flow EPSILON_TIME = 1.0e-3;
const Cost CAPACITY_INF = 1.0e+3;

Cost BPRNotConvexNetwork::ConnectionEdge::getLessPriorityCapacity(const Solution &x) const {
    if(conflicts.empty()) return CAPACITY_INF;

    Capacity totalCapacity = 0.0;

    // if(id == 18600 || id == 18601){
    //     for(const auto &v: conflicts){
    //         for(const auto &[e, p]: v){
    //             cerr << id << " conflicts with " << e->id << endl;
    //         }
    //     }
    // }

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

Cost BPRNotConvexNetwork::ConnectionEdge::calculateCost(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Cost t1 = t0 * (1.0 + network.alpha * pow(f / c, network.beta));

    Cost c2  = getLessPriorityCapacity(x);
    Time fft = 1.0 / c2;
    Cost t2  = fft * (1.0 + network.alpha * pow(f / c2, network.beta));

    return t1 + t2;
}

Cost BPRNotConvexNetwork::ConnectionEdge::calculateCostGlobal(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Cost t1 = t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);

    Cost c2  = getLessPriorityCapacity(x);
    Time fft = 1.0 / c2;
    Cost t2  = fft * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c2, network.beta) + 1.0);

    return t1 + t2;
}

Cost BPRNotConvexNetwork::ConnectionEdge::calculateCostDerivative(const Solution &x) const {
    Flow f  = x.getFlowInEdge(id);
    Cost t1 = t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);

    Cost c2  = getLessPriorityCapacity(x);
    Time fft = 1.0 / c2;
    Cost t2  = fft * network.alpha * network.beta * pow(f / c2, network.beta - 1);

    return t1 + t2;
}

BPRNotConvexNetwork::BPRNotConvexNetwork(Flow alpha_, Flow beta_):
    alpha(alpha_), beta(beta_) {}

void BPRNotConvexNetwork::addNode(Node u) {
    adj[u];
}

void BPRNotConvexNetwork::addEdge(Edge *e) {
    adj[e->u].push_back(e);
    adj[e->v];
    edges[e->id] = e;
}

std::vector<Node> BPRNotConvexNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &[u, _]: adj)
        ret.push_back(u);
    return ret;
}

Edge *BPRNotConvexNetwork::getEdge(Edge::ID e) const {
    return edges.at(e);
}

std::vector<Network::Edge *> BPRNotConvexNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Network::Edge *>(v.begin(), v.end());
}

const Edges &BPRNotConvexNetwork::getEdges() const {
    return edges;
}

void BPRNotConvexNetwork::saveEdges(
    const SUMO::NetworkTAZs &sumo,
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &filePath
) const {
    xml_document<> doc;
    auto           meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "3600.0"));
    meandata->append_node(interval);

    const vector<SUMO::Network::Edge::ID> &sumoEdges = adapter.getSumoEdges();

    list<string> strs;

    for(const SUMO::Network::Edge::ID &sumoEdgeID: sumoEdges) {
        try {
            Edge::ID    eID = adapter.toEdge(sumoEdgeID);
            NormalEdge *e   = dynamic_cast<NormalEdge *>(getEdge(eID));
            assert(e != nullptr);

            Node v = adapter.toNodes(sumoEdgeID).second;

            const size_t &N = sumo.network.getEdge(sumoEdgeID).lanes.size();

            Capacity cap = e->c;
            Flow f   = x.getFlowInEdge(eID);
            Cost c = e->calculateCongestion(x);

            double t0  = e->calculateCost(SolutionBase());
            double fft = f * t0, t = f * e->calculateCost(x);
            for(const Edge *edge: adj.at(v)) {
                Flow f_ = x.getFlowInEdge(edge->id);
                fft += f_ * edge->calculateCost(SolutionBase());
                t += f_ * edge->calculateCost(x);
            }

            Cost d;
            if(f == 0.0) {
                fft = t = t0;
                d       = 1.0;
            } else {
                fft /= f;
                t /= f;
                d = t / fft;
            }

            string &caps  = (strs.emplace_back() = stringify<Flow>::toString(cap));
            string &cappls= (strs.emplace_back() = stringify<Flow>::toString(cap / N));
            string &fs    = (strs.emplace_back() = stringify<Flow>::toString(f));
            string &fpls  = (strs.emplace_back() = stringify<Flow>::toString(f / N));
            string &cs    = (strs.emplace_back() = stringify<Flow>::toString(c));
            string &t0s   = (strs.emplace_back() = stringify<Flow>::toString(t0));
            string &ffts  = (strs.emplace_back() = stringify<Flow>::toString(fft));
            string &ts    = (strs.emplace_back() = stringify<Flow>::toString(t));
            string &ds    = (strs.emplace_back() = stringify<Flow>::toString(d));
            string &dlogs = (strs.emplace_back() = stringify<Flow>::toString(log(d) / log(2)));

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id", sumoEdgeID.c_str()));
            edge->append_attribute(doc.allocate_attribute("capacity", caps.c_str()));
            edge->append_attribute(doc.allocate_attribute("capacityPerLane", cappls.c_str()));
            edge->append_attribute(doc.allocate_attribute("flow", fs.c_str()));
            edge->append_attribute(doc.allocate_attribute("flowPerLane", fpls.c_str()));
            edge->append_attribute(doc.allocate_attribute("congestion", cs.c_str()));
            edge->append_attribute(doc.allocate_attribute("t0", t0s.c_str()));
            edge->append_attribute(doc.allocate_attribute("fft", ffts.c_str()));
            edge->append_attribute(doc.allocate_attribute("t", ts.c_str()));
            edge->append_attribute(doc.allocate_attribute("delay", ds.c_str()));
            edge->append_attribute(doc.allocate_attribute("log_delay", dlogs.c_str()));
            interval->append_node(edge);
        } catch(const out_of_range &ex) {
            // cerr << "Could not find SUMO edge corresponding to edge " << e << ", ignoring" << endl;
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

void BPRNotConvexNetwork::saveRoutes(
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &filePath
) const {
    xml_document<> doc;
    auto           routesEl = doc.allocate_node(node_element, "routes");
    doc.append_node(routesEl);

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

    list<string> strs;
    size_t       flowID = 0;
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
            colorRGB      = colorHSV;
            string &color = (strs.emplace_back() = stringify<color::rgb<float>>::toString(colorRGB));

            string &rs      = (strs.emplace_back() = stringify<SUMO::Route>::toString(route));
            string &ids     = (strs.emplace_back() = stringify<size_t>::toString(flowID++));
            string &periods = (strs.emplace_back() = stringify<Flow>::toString(flow*60*60));

            auto flowEl = doc.allocate_node(node_element, "flow");
            flowEl->append_attribute(doc.allocate_attribute("id", ids.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("color", color.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("begin", "0"));
            flowEl->append_attribute(doc.allocate_attribute("end", "3600"));
            flowEl->append_attribute(doc.allocate_attribute("fromTaz", fromTaz.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("toTaz", toTaz.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("vehsPerHour", periods.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("departPos", "random_free"));
            flowEl->append_attribute(doc.allocate_attribute("departSpeed", "random"));

            auto routeEl = doc.allocate_node(node_element, "route");
            routeEl->append_attribute(doc.allocate_attribute("edges", rs.c_str()));

            flowEl->append_node(routeEl);
            routesEl->append_node(flowEl);
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

void BPRNotConvexNetwork::saveResultsToFile(
    const SUMO::NetworkTAZs &sumo,
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &edgeDataPath,
    const string            &routesPath
) const {
    saveEdges(sumo, x, adapter, edgeDataPath);
    saveRoutes(x, adapter, routesPath);
}

BPRConvexNetwork BPRNotConvexNetwork::makeConvex(const Solution &x) const {
    return BPRConvexNetwork(*this, x);
}
