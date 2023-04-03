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

using utils::stringifier;

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
    return e->t0 * f * ((alpha / (beta + 1.0)) * pow(f / e->c, beta) + 1.0);
}

Cost BPRNetwork::calculateCongestion(Edge::ID id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return f / e->c;
}

Cost calculateLength(const SUMO::Network::Edge &e) {
    Lane::Length length = 0;
    for (const auto &p : e.lanes) {
        length += p.second.length;
    }
    length /= Lane::Length(e.lanes.size());
    return length;
}

Cost calculateSpeed(const SUMO::Network::Edge &e) {
    Lane::Speed averageSpeed = 0;
    for (const auto &p : e.lanes) {
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
    Lane::Length length = calculateLength(e);
    Lane::Speed freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost freeFlowTime = length / freeFlowSpeed;
    return freeFlowTime;
}

const Cost SATURATION_FLOW = 1110.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 1800.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 2000.0;  // vehicles per hour per lane

Cost calculateCapacity(const SUMO::Network::Edge &e) {
    Lane::Speed freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost capacity = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / (50.0 / 3.6)) * (Cost)e.lanes.size();
    return capacity;
}

Tuple BPRNetwork::fromSumo(const SUMO::Network &sumoNetwork, const SumoTAZs &sumoTAZs) {
    BPRNetwork *network = new BPRNetwork();
    SumoAdapterStatic adapter;

    map<SUMO::Network::Junction::ID, list<SUMO::Network::Edge>> in, out;

    const vector<SUMO::Network::Edge> &edges = sumoNetwork.getEdges();
    for (const SUMO::Network::Edge &edge : edges) {
        if (edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

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
            calculateCapacity(edge));

        in[edge.to].push_back(edge);
        out[edge.from].push_back(edge);
    }

    const unordered_map<
        SUMO::Network::Edge::ID,
        unordered_map<
            SUMO::Network::Edge::ID,
            list<SUMO::Network::Connection>>> &connections = sumoNetwork.getConnections();
    for (const auto &p1 : connections) {
        const SUMO::Network::Edge::ID &from = p1.first;
        if (!adapter.isEdge(from)) continue;

        for (const auto &p2 : p1.second) {
            const SUMO::Network::Edge::ID &to = p2.first;
            if (!adapter.isEdge(from)) continue;

            // const size_t &numberLanes = p2.second.size();

            double t0 = 0;
            for (const SUMO::Network::Connection &conn : p2.second) {
                switch (conn.dir) {
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
                // if(conn.from == "1016658"){
                //     cerr << "connection from=" << conn.from << ", dir=" << stringifier<SUMO::Network::Connection::Direction>::toString(conn.dir) << ", t0=" << t0 << endl;
                // }
            }
            t0 /= (double)p2.second.size();

            network->addEdge(
                adapter.addEdge(),
                adapter.toNodes(from).second,
                adapter.toNodes(to).first,
                t0, 1e9);
        }
    }

    const vector<SUMO::Network::Junction> &junctions = sumoNetwork.getJunctions();
    for (const SUMO::Network::Junction &junction : junctions) {
        // Allow vehicles to go in any direction in dead ends
        if (junction.type == SUMO::Network::Junction::DEAD_END) {
            for (const SUMO::Network::Edge &e1 : in[junction.id]) {
                for (const SUMO::Network::Edge &e2 : out[junction.id]) {
                    network->addEdge(
                        adapter.addEdge(),
                        adapter.toNodes(e1.id).second,
                        adapter.toNodes(e2.id).first,
                        20, 1e9);
                }
            }
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

    const vector<SUMO::Network::Edge::ID> &sumoEdges = adapter.getSumoEdges();

    list<string> strs;
    for (const SUMO::Network::Edge::ID &eid : sumoEdges) {
        try {
            Edge::ID e = adapter.toEdge(eid);
            Node v = adapter.toNodes(eid).second;

            Flow f = x.getFlowInEdge(e);
            Cost c = calculateCongestion(e, f);

            double t0 = calculateCost(e, 0);
            double fft = f * t0, t = f * calculateCost(e, f);
            for(const CustomEdge *edge: adj.at(v)){
                Flow f_ = x.getFlowInEdge(edge->id);
                fft += f_ * calculateCost(edge->id, 0);
                t += f_ * calculateCost(edge->id, f_);
            }
            
            Cost d;
            if(f == 0.0){
                fft = t = t0;
                d = 1.0;
            } else {
                fft /= f;
                t /= f;
                d = t/fft;
            }


            // if(eid == "1016658"){
            //     cerr << "connection from=" << eid << ", dir=" << stringifier<SUMO::Network::Connection::Direction>::toString(conn.dir) << ", t0=" << t0 << endl;
            // }

            string &fs = (strs.emplace_back() = stringifier<Flow>::toString(f));
            string &cs = (strs.emplace_back() = stringifier<Flow>::toString(c));
            string &t0s = (strs.emplace_back() = stringifier<Flow>::toString(t0));
            string &ffts = (strs.emplace_back() = stringifier<Flow>::toString(fft));
            string &ts = (strs.emplace_back() = stringifier<Flow>::toString(t));
            string &ds = (strs.emplace_back() = stringifier<Flow>::toString(d));
            string &dlogs = (strs.emplace_back() = stringifier<Flow>::toString(log(d)/log(2)));

            auto edge = doc.allocate_node(node_element, "edge");
            edge->append_attribute(doc.allocate_attribute("id"          , eid  .c_str()));
            edge->append_attribute(doc.allocate_attribute("flow"        , fs   .c_str()));
            edge->append_attribute(doc.allocate_attribute("congestion"  , cs   .c_str()));
            edge->append_attribute(doc.allocate_attribute("t0"          , t0s  .c_str()));
            edge->append_attribute(doc.allocate_attribute("fft"         , ffts .c_str()));
            edge->append_attribute(doc.allocate_attribute("t"           , ts   .c_str()));
            edge->append_attribute(doc.allocate_attribute("delay"       , ds   .c_str()));
            edge->append_attribute(doc.allocate_attribute("log(delay)"  , dlogs.c_str()));
            interval->append_node(edge);
        } catch (const out_of_range &ex) {
            // cerr << "Could not find SUMO edge corresponding to edge " << e << ", ignoring" << endl;
        }
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    os.open(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
