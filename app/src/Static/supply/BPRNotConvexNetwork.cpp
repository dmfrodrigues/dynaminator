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
typedef BPRNotConvexNetwork::NormalEdge     NormalEdge;
typedef BPRNotConvexNetwork::ConnectionEdge ConnectionEdge;
typedef BPRNotConvexNetwork::Flow           Flow;
typedef BPRNotConvexNetwork::Time           Time;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;

const double T_CR = 5.0;

BPRNotConvexNetwork::NormalEdge::NormalEdge(NormalEdge::ID id_, Node u_, Node v_, const BPRNotConvexNetwork &network_, Time t0_, Flow c_):
    BPRNetwork::NormalEdge(id_, u_, v_, network_, t0_, c_) {}

BPRNotConvexNetwork::ConnectionEdge::ConnectionEdge(ConnectionEdge::ID id_, Node u_, Node v_, const BPRNotConvexNetwork &network_, Time t0_, Flow c_):
    Edge(id_, u_, v_, network_, t0_, c_) {}

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
            NormalEdge &e   = dynamic_cast<NormalEdge &>(getEdge(eID));

            Node v = adapter.toNodes(sumoEdgeID).second;

            const size_t &N = sumo.network.getEdge(sumoEdgeID).lanes.size();

            Flow cap = e.c;
            Flow f   = x.getFlowInEdge(eID);
            Time c   = e.calculateCongestion(x);

            double t0  = e.calculateCost(SolutionBase());
            double fft = f * t0, t = f * e.calculateCost(x);
            for(const Edge *edge: adj.at(v)) {
                Flow f_ = x.getFlowInEdge(edge->id);
                fft += f_ * edge->calculateCost(SolutionBase());
                t += f_ * edge->calculateCost(x);
            }

            Time d;
            if(f == 0.0) {
                fft = t = t0;
                d       = 1.0;
            } else {
                fft /= f;
                t /= f;
                d = t / fft;
            }

            string &caps   = (strs.emplace_back() = stringify<Flow>::toString(cap));
            string &cappls = (strs.emplace_back() = stringify<Flow>::toString(cap / N));
            string &fs     = (strs.emplace_back() = stringify<Flow>::toString(f));
            string &fpls   = (strs.emplace_back() = stringify<Flow>::toString(f / N));
            string &cs     = (strs.emplace_back() = stringify<Flow>::toString(c));
            string &t0s    = (strs.emplace_back() = stringify<Flow>::toString(t0));
            string &ffts   = (strs.emplace_back() = stringify<Flow>::toString(fft));
            string &ts     = (strs.emplace_back() = stringify<Flow>::toString(t));
            string &ds     = (strs.emplace_back() = stringify<Flow>::toString(d));
            string &dlogs  = (strs.emplace_back() = stringify<Flow>::toString(log(d) / log(2)));

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

BPRConvexNetwork BPRNotConvexNetwork::makeConvex(const Solution &x) const {
    return BPRConvexNetwork(*this, x);
}
