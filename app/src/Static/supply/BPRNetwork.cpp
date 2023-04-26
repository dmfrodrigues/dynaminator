#include "Static/supply/BPRNetwork.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
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

typedef BPRNetwork::Node       Node;
typedef BPRNetwork::NormalEdge Edge;
typedef BPRNetwork::Flow       Flow;
typedef BPRNetwork::Cost       Cost;

typedef SUMO::Network::Edge::Lane Lane;

typedef pair<
    BPRNetwork *,
    SumoAdapterStatic>
    Tuple;

BPRNetwork::NormalEdge::NormalEdge(NormalEdge::ID id_, Node u_, Node v_, const BPRNetwork &network_, Time t0_, Capacity c_):
    NetworkDifferentiable::Edge(id_, u_, v_),
    network(network_),
    t0(t0_),
    c(c_) {
}

Cost BPRNetwork::NormalEdge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * (1.0 + network.alpha * pow(f / c, network.beta));
}

Cost BPRNetwork::NormalEdge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);
}

Cost BPRNetwork::NormalEdge::calculateCostDerivative(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);
}

Cost BPRNetwork::NormalEdge::calculateCongestion(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return f / c;
}

Cost BPRNetwork::SignalizedEdge::calculateCost(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * (1.0 + network.alpha * pow(f / c, network.beta));
}

Cost BPRNetwork::SignalizedEdge::calculateCostGlobal(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * f * ((network.alpha / (network.beta + 1.0)) * pow(f / c, network.beta) + 1.0);
}

Cost BPRNetwork::SignalizedEdge::calculateCostDerivative(const Solution &x) const {
    Flow f = x.getFlowInEdge(id);
    return t0 * network.alpha * network.beta * pow(f / c, network.beta - 1);
}

BPRNetwork::BPRNetwork(Flow alpha_, Flow beta_):
    alpha(alpha_), beta(beta_) {}

void BPRNetwork::addNode(Node u) {
    adj[u];
}

void BPRNetwork::addEdge(NormalEdge *e) {
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

Edge *BPRNetwork::getEdge(NormalEdge::ID e) const {
    return edges.at(e);
}

std::vector<Network::Edge *> BPRNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Network::Edge *>(v.begin(), v.end());
}

Cost calculateLength(const SUMO::Network::Edge &e) {
    Lane::Length length = 0;
    for(const auto &p: e.lanes) {
        length += p.second.length;
    }
    length /= Lane::Length(e.lanes.size());
    return length;
}

Cost calculateSpeed(const SUMO::Network::Edge &e) {
    Lane::Speed averageSpeed = 0;
    for(const auto &p: e.lanes) {
        averageSpeed += p.second.speed;
    }
    averageSpeed /= Lane::Speed(e.lanes.size());
    return averageSpeed;
}

Cost calculateFreeFlowSpeed(const SUMO::Network::Edge &e) {
    return calculateSpeed(e) * 0.9;
    // return calculateSpeed(e);
}

Cost calculateFreeFlowTime(const SUMO::Network::Edge &e) {
    Lane::Length length        = calculateLength(e);
    Lane::Speed  freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost         freeFlowTime  = length / freeFlowSpeed;
    return freeFlowTime;
}

const Cost SATURATION_FLOW = 1110.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 1800.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 2000.0;  // vehicles per hour per lane

Cost calculateCapacity(const SUMO::Network::Edge &e, const SUMO::Network &sumoNetwork) {
    const auto &connections   = sumoNetwork.getConnections();

    Lane::Speed freeFlowSpeed     = calculateFreeFlowSpeed(e);
    Cost        adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / (50.0 / 3.6));
    Cost        c                 = adjSaturationFlow * (Cost)e.lanes.size();

    vector<Cost> capacityPerLane(e.lanes.size(), 0.0);
    if(connections.count(e.id)) {
        for(const auto &[eNextID, eConnections]: connections.at(e.id)) {
            for(const SUMO::Network::Connection &conn: eConnections) {
                Cost cAdd = adjSaturationFlow;
                if(conn.tl) {
                    SUMO::Time
                        g = conn.tl->getGreenTime((size_t)conn.linkIndex),
                        C = conn.tl->getCycleTime();
                    // int n = tl.getNumberStops(conn.linkIndex);
                    cAdd *= g / C;
                }
                capacityPerLane.at(conn.fromLane().index) += cAdd;
            }
        }
    }
    for(Cost &capPerLane: capacityPerLane)
        capPerLane = min(capPerLane, adjSaturationFlow);
    Cost cNew = accumulate(capacityPerLane.begin(), capacityPerLane.end(), 0.0);
    if(cNew != 0.0) {
        c = min(c, cNew);
    }

    return c;
}

Tuple BPRNetwork::fromSumo(const SUMO::Network &sumoNetwork, const SUMO::TAZs &sumoTAZs) {
    BPRNetwork       *network = new BPRNetwork();
    SumoAdapterStatic adapter;

    map<SUMO::Network::Junction::ID, list<SUMO::Network::Edge>> in, out;

    // clang-format off
    unordered_map<
        SUMO::Network::Edge::ID, unordered_map<
            SUMO::Network::Edge::ID,
            list<SUMO::Network::Connection>
        >
    > connections = sumoNetwork.getConnections();
    // clang-format on

    const vector<SUMO::Network::Edge> &edges = sumoNetwork.getEdges();
    for(const SUMO::Network::Edge &edge: edges) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const auto           &p   = adapter.addSumoEdge(edge.id);
        const NormalEdge::ID &eid = p.first;
        Node                  u = p.second.first, v = p.second.second;

        Cost t0 = calculateFreeFlowTime(edge);
        Cost c  = calculateCapacity(edge, sumoNetwork);

        network->addNode(u);
        network->addNode(v);
        network->addEdge(new NormalEdge(eid, u, v, *network, t0, c));

        in[edge.to].push_back(edge);
        out[edge.from].push_back(edge);
    }

    for(const auto &[from, fromConnections]: connections) {
        if(!adapter.isEdge(from)) continue;

        for(const auto &[to, fromToConnections]: fromConnections) {
            if(!adapter.isEdge(from)) continue;

            // const size_t &numberLanes = p2.second.size();

            double t0 = 0;
            for(const SUMO::Network::Connection &conn: fromToConnections) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
                switch(conn.dir) {
                    case SUMO::Network::Connection::Direction::PARTIALLY_RIGHT:
                        t0 += 5.0;
                        break;
                    case SUMO::Network::Connection::Direction::RIGHT:
                        t0 += 10.0;
                        break;
                    case SUMO::Network::Connection::Direction::PARTIALLY_LEFT:
                        t0 += 15.0;
                        break;
                    case SUMO::Network::Connection::Direction::LEFT:
                        t0 += 20.0;
                        break;
                    default:
                        break;
                }
#pragma GCC diagnostic pop
            }
            t0 /= (double)fromToConnections.size();

            network->addEdge(new NormalEdge(
                adapter.addEdge(),
                adapter.toNodes(from).second,
                adapter.toNodes(to).first,
                *network,
                t0,
                1e9
            ));
        }
    }

    const vector<SUMO::Network::Junction> &junctions = sumoNetwork.getJunctions();
    for(const SUMO::Network::Junction &junction: junctions) {
        // Allow vehicles to go in any direction in dead ends
        if(junction.type == SUMO::Network::Junction::DEAD_END) {
            for(const SUMO::Network::Edge &e1: in[junction.id]) {
                for(const SUMO::Network::Edge &e2: out[junction.id]) {
                    network->addEdge(new NormalEdge(
                        adapter.addEdge(),
                        adapter.toNodes(e1.id).second,
                        adapter.toNodes(e2.id).first,
                        *network,
                        20,
                        1e9
                    ));
                }
            }
        }
    }

    for(const auto &[id, taz]: sumoTAZs) {
        const auto &[source, sink] = adapter.addSumoTAZ(taz.id);
        for(const SUMO::TAZ::Source &s: taz.sources) {
            const NormalEdge *e = network->edges.at(adapter.toEdge(s.id));
            network->addEdge(new NormalEdge(
                adapter.addEdge(),
                source,
                e->u,
                *network,
                0,
                1e9
            ));
        }
        for(const SUMO::TAZ::Sink &s: taz.sinks) {
            const NormalEdge *e = network->edges.at(adapter.toEdge(s.id));
            network->addEdge(new NormalEdge(
                adapter.addEdge(),
                e->v,
                sink,
                *network,
                0,
                1e9
            ));
        }
    }

    return Tuple(network, adapter);
}

void BPRNetwork::saveEdges(
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &filePath
) const {
    xml_document<> doc;
    auto           meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    const vector<SUMO::Network::Edge::ID> &sumoEdges = adapter.getSumoEdges();

    list<string> strs;
    for(const SUMO::Network::Edge::ID &sumoEdgeID: sumoEdges) {
        try {
            NormalEdge::ID eID = adapter.toEdge(sumoEdgeID);
            NormalEdge    *e   = getEdge(eID);

            Node v = adapter.toNodes(sumoEdgeID).second;

            Flow f = x.getFlowInEdge(eID);
            Cost c = e->calculateCongestion(x);

            double t0  = e->calculateCost(SolutionBase());
            double fft = f * t0, t = f * e->calculateCost(x);
            for(const NormalEdge *edge: adj.at(v)) {
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

            string &fs    = (strs.emplace_back() = stringify<Flow>::toString(f));
            string &cs    = (strs.emplace_back() = stringify<Flow>::toString(c));
            string &t0s   = (strs.emplace_back() = stringify<Flow>::toString(t0));
            string &ffts  = (strs.emplace_back() = stringify<Flow>::toString(fft));
            string &ts    = (strs.emplace_back() = stringify<Flow>::toString(t));
            string &ds    = (strs.emplace_back() = stringify<Flow>::toString(d));
            string &dlogs = (strs.emplace_back() = stringify<Flow>::toString(log(d) / log(2)));

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id", sumoEdgeID.c_str()));
            edge->append_attribute(doc.allocate_attribute("flow", fs.c_str()));
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

void BPRNetwork::saveRoutes(
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
            string &periods = (strs.emplace_back() = stringify<Flow>::toString(1.0 / flow));

            auto flowEl = doc.allocate_node(node_element, "flow");
            flowEl->append_attribute(doc.allocate_attribute("id", ids.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("color", color.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("begin", "0"));
            flowEl->append_attribute(doc.allocate_attribute("end", "3600"));
            flowEl->append_attribute(doc.allocate_attribute("fromTaz", fromTaz.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("toTaz", toTaz.c_str()));
            flowEl->append_attribute(doc.allocate_attribute("period", periods.c_str()));
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

void BPRNetwork::saveResultsToFile(
    const Solution          &x,
    const SumoAdapterStatic &adapter,
    const string            &edgeDataPath,
    const string            &routesPath
) const {
    saveEdges(x, adapter, edgeDataPath);
    saveRoutes(x, adapter, routesPath);
}
