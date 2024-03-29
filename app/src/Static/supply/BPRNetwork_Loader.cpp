#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <tuple>

#include "Alg/Flow/EdmondsKarp.hpp"
#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/BFS.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace Static;
using namespace utils;

using Alg::Graph;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;
typedef SUMO::Length              Length;

Time BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateFreeFlowSpeed(const Time &maxSpeed) const {
    return maxSpeed * 0.9;
}

Time BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateFreeFlowSpeed(const SUMO::Network::Edge &e) const {
    return e.speed() * 0.9;
}

Time BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateFreeFlowSpeed(const SUMO::Network::Edge::Lane &l) const {
    return l.speed * 0.9;
}

Time BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateFreeFlowTime(const SUMO::Network::Edge &e) const {
    Length length        = e.length();
    Speed  freeFlowSpeed = calculateFreeFlowSpeed(e);
    Time   freeFlowTime  = length / freeFlowSpeed;
    return freeFlowTime;
}

Time BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateFreeFlowTime(const SUMO::Network::Edge::Lane &l) const {
    Length length        = l.length;
    Speed  freeFlowSpeed = calculateFreeFlowSpeed(l);
    Time   freeFlowTime  = length / freeFlowSpeed;
    return freeFlowTime;
}

const Flow SATURATION_FLOW             = 1110.0;  // vehicles per hour per lane
const Flow SATURATION_FLOW_EXTRA_LANES = 800.0;

/**
 * @brief Cost of having traffic stop once. This should be a time penalty that
 * averages all the negative effects of cars having to start after the light
 * turns green.
 */
const Time STOP_PENALTY = 0.0;

Flow BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateCapacity(const SUMO::Network::Edge &e) const {
    Speed freeFlowSpeed               = calculateFreeFlowSpeed(e);
    Time  adjSaturationFlow           = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / calculateFreeFlowSpeed(50.0 / 3.6));
    Time  adjSaturationFlowExtraLanes = (SATURATION_FLOW_EXTRA_LANES / 60.0 / 60.0) * (freeFlowSpeed / calculateFreeFlowSpeed(50.0 / 3.6));
    Flow  c                           = adjSaturationFlow + adjSaturationFlowExtraLanes * (Time)(e.lanes.size() - 1);

    const vector<reference_wrapper<const SUMO::Network::Connection>> &connections = e.getOutgoingConnections();
    if(!connections.empty()) {
        vector<Flow> capacityPerLane(e.lanes.size(), 0.0);
        for(const SUMO::Network::Connection &conn: connections) {
            Flow cAdd = adjSaturationFlow;
            if(conn.tl) {
                SUMO::Time
                    g    = conn.getGreenTime(),
                    C    = conn.getCycleTime();
                size_t n = conn.getNumberStops();
                cAdd *= (g - STOP_PENALTY * (Time)n) / C;
            }
            capacityPerLane.at(conn.fromLane().index) += cAdd;
        }
        for(Flow &capPerLane: capacityPerLane)
            capPerLane = min(capPerLane, adjSaturationFlow);
        Flow cNew = accumulate(capacityPerLane.begin(), capacityPerLane.end(), 0.0);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if(cNew != 0.0) {
            c = min(c, cNew);
        }
#pragma GCC diagnostic pop
    }

    return c;
}

Flow BPRNetwork::Loader<SUMO::NetworkTAZs>::calculateCapacity(const SUMO::Network::Edge::Lane &lane) const {
    Speed freeFlowSpeed     = calculateFreeFlowSpeed(lane);
    Time  adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / calculateFreeFlowSpeed(50.0 / 3.6));
    Flow  c                 = adjSaturationFlow;
    return c;
}

BPRNetwork *BPRNetwork::Loader<SUMO::NetworkTAZs>::load(const SUMO::NetworkTAZs &sumo) {
    clear();

    network = new BPRNetwork();

    addNormalEdges(sumo);

    addConnections(sumo);

    iterateCapacities(sumo);

    addTAZs(sumo);

    return network;
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::addNormalEdges(const SUMO::NetworkTAZs &sumo) {
    const vector<SUMO::Network::Edge> &sumoEdges = sumo.network.getEdges();
    for(const SUMO::Network::Edge &edge: sumoEdges) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const auto           &p   = adapter.addSumoEdge(edge.id);
        const NormalEdge::ID &eid = p.first;
        Node                  u = p.second.first, v = p.second.second;

        // clang-format off
        normalEdges[edge.id] = network->addNormalEdge(
            eid,
            u, v,
            *network,
            calculateFreeFlowTime(edge),
            calculateCapacity(edge)
        );
        // clang-format on

        in[edge.to.value().get().id].push_back(edge);
        out[edge.from.value().get().id].push_back(edge);
    }
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::addConnections(const SUMO::NetworkTAZs &sumo) {
    auto connections = sumo.network.getConnections();

    for(const SUMO::Network::Edge &from: sumo.network.getEdges()) {
        if(from.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        for(const SUMO::Network::Edge &to: from.getOutgoing()) {
            if(to.function == SUMO::Network::Edge::Function::INTERNAL) continue;

            addConnection(sumo, from, to);
        }
    }
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to) {
    auto fromToConnections = sumo.network.getConnections(from, to);

    if(fromToConnections.empty()) return;

    Speed v = min(
        calculateFreeFlowSpeed(from),
        calculateFreeFlowSpeed(to)
    );
    Time adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (v / calculateFreeFlowSpeed(50.0 / 3.6));

    vector<Time> capacityFromLanes(from.lanes.size(), 0.0);
    vector<Time> capacityToLanes(to.lanes.size(), 0.0);

    double t0 = 0;

    for(const SUMO::Network::Connection &conn: fromToConnections) {
        Time cAdd = adjSaturationFlow;
        /// Traffic lights
        if(conn.tl) {
            SUMO::Time g = conn.getGreenTime();
            SUMO::Time C = conn.getCycleTime();
            SUMO::Time r = C - g;
            size_t     n = conn.getNumberStops();
            cAdd *= (g - STOP_PENALTY * (double)n) / C;

            t0 += r * r / (2.0 * C);
        }
        capacityFromLanes[conn.fromLane().index] += cAdd;
        capacityToLanes[conn.toLane().index] += cAdd;

        /// Direction changes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
        // clang-format off
        switch(conn.dir) {
            case SUMO::Network::Connection::Direction::PARTIALLY_RIGHT: t0 += 1.0; break;
            case SUMO::Network::Connection::Direction::RIGHT          : t0 += 2.0; break;
            case SUMO::Network::Connection::Direction::PARTIALLY_LEFT : t0 += 2.0; break;
            case SUMO::Network::Connection::Direction::LEFT           : t0 += 5.0; break;
            case SUMO::Network::Connection::Direction::TURN           : t0 += 20.0; break;
            default:
                break;
        }
            // clang-format on
#pragma GCC diagnostic pop
    }
    t0 /= (double)fromToConnections.size();

    for(Time &c: capacityFromLanes) c = min(c, adjSaturationFlow);
    for(Time &c: capacityToLanes) c = min(c, adjSaturationFlow);
    double cFrom = accumulate(capacityFromLanes.begin(), capacityFromLanes.end(), 0.0);
    double cTo   = accumulate(capacityToLanes.begin(), capacityToLanes.end(), 0.0);
    double c     = min(cFrom, cTo);

    ConnectionEdge *e = network->addConnectionEdge(
        adapter.addEdge(),
        adapter.toNodes(from.id).second,
        adapter.toNodes(to.id).first,
        *network,
        t0,
        c
    );

    connectionEdges[e->id] = make_tuple(e, from.id, to.id);

    // connectionMap[from.id][to.id] = e->id;
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::iterateCapacities(const SUMO::NetworkTAZs &sumo) {
    map<Edge::ID, Edge *> &netEdges = network->edges;

    const Flow   EPSILON    = 1.0 / 60.0 / 60.0 / 24.0;
    const size_t ITERATIONS = 100;

    bool changed = true;

    for(size_t i = 0; i < ITERATIONS && changed; ++i) {
        changed = false;

        for(auto &[edgeID, edge]: netEdges) {
            // 1.1
            {
                const auto &nextEdges = network->adj.at(edge->v);
                if(!nextEdges.empty()) {
                    Flow c = 0.0;
                    for(const Edge *nextEdge: nextEdges)
                        c += nextEdge->c;

                    if(edge->c > c + EPSILON) {
                        // cerr << "    1.1. | "
                        //      << "Capacity of edge " << edge->id
                        //      << " (SUMO edge " << (adapter.isSumoEdge(edge->id) ? adapter.toSumoEdge(edge->id) : "-") << ")"
                        //      << " was reduced from " << edge->c
                        //      << " to " << c
                        //      << " (delta=" << edge->c - c << ")"
                        //      << endl;
                        // cerr << "        Note: Adjacent edges' capacities are:" << endl;
                        // for(const Edge *nextEdge: nextEdges)
                        //     cerr << "        " << nextEdge->id << " (SUMO edge " << (adapter.isSumoEdge(nextEdge->id) ? adapter.toSumoEdge(nextEdge->id) : "-") << "): " << nextEdge->c << endl;
                        // if(c / edge->c < 0.5)
                        //     cerr << "        WARNING: Capacity reduced by more than 50%! ================================" << endl;
                        edge->c = c;
                        changed = true;
                    }
                }
            }
        }

        for(auto &[edgeID, edge]: netEdges) {
            // 1.2
            if(adapter.isSumoEdge(edge->id)) {
                const SUMO::Network::Edge::ID fromID = adapter.toSumoEdge(edge->id);
                const SUMO::Network::Edge    &from   = sumo.network.getEdge(fromID);

                const vector<reference_wrapper<const SUMO::Network::Connection>> connections = from.getOutgoingConnections();

                if(!connections.empty()) {
                    map<SUMO::Network::Edge::ID, Graph::Node>       sumoEdges2nodes;
                    map<SUMO::Network::Edge::Lane::ID, Graph::Node> sumoLanes2nodes;

                    Graph           G;
                    Graph::Node     incNode = 0;
                    Graph::Edge::ID incEdge = 0;

                    Graph::Node vSource = incNode++;
                    Graph::Node vSink   = incNode++;

                    for(const SUMO::Network::Connection &conn: connections) {
                        const Edge &nextEdge = network->getEdge(adapter.toEdge(conn.to.id));

                        // This from edge was never seen before
                        Graph::Node edgeSource;
                        if(!sumoEdges2nodes.count(conn.from.id)) {
                            edgeSource = incNode++;

                            sumoEdges2nodes[conn.from.id] = edgeSource;

                            G.addEdge(incEdge++, vSource, edgeSource, edge->c);
                        } else {
                            edgeSource = sumoEdges2nodes.at(conn.from.id);
                        }

                        // This to edge was never seen before
                        Graph::Node edgeSink;
                        if(!sumoEdges2nodes.count(conn.to.id)) {
                            edgeSink = incNode++;

                            sumoEdges2nodes[conn.to.id] = edgeSink;

                            G.addEdge(incEdge++, edgeSink, vSink, nextEdge.c);
                        } else {
                            edgeSink = sumoEdges2nodes.at(conn.to.id);
                        }

                        // This fromLane was never seen before
                        Graph::Node u;
                        if(!sumoLanes2nodes.count(conn.fromLane().id)) {
                            u = incNode++;

                            sumoLanes2nodes[conn.fromLane().id] = u;

                            G.addEdge(incEdge++, edgeSource, u, calculateCapacity(conn.fromLane()));
                        } else {
                            u = sumoLanes2nodes.at(conn.fromLane().id);
                        }

                        // This toLane was never seen before
                        Graph::Node v;
                        if(!sumoLanes2nodes.count(conn.toLane().id)) {
                            v = incNode++;

                            sumoLanes2nodes[conn.toLane().id] = v;

                            G.addEdge(incEdge++, v, edgeSink, calculateCapacity(conn.toLane()));
                        } else {
                            v = sumoLanes2nodes.at(conn.toLane().id);
                        }

                        G.addEdge(incEdge++, u, v, INFINITY);
                    }

                    Alg::ShortestPath::BFS   sp;
                    Alg::Flow::EdmondsKarp   maxFlow(sp);
                    Alg::Graph::Edge::Weight c = maxFlow.solve(G, vSource, vSink);

                    if(edge->c > c + EPSILON) {
                        // cerr << "    1.2. | "
                        //      << "Capacity of edge " << edge->id
                        //      << " (SUMO edge " << adapter.toSumoEdge(edge->id) << ")"
                        //      << " was reduced from " << edge->c
                        //      << " to " << c
                        //      << " (delta=" << edge->c - c << ")"
                        //      << endl;
                        edge->c = c;
                        changed = true;
                    }
                }
            }
        }

        // 2.
        for(auto &[edgeID, edge]: netEdges) {
            // 2.1
            if(!adapter.isSumoEdge(edge->id)) {
                const auto &[_, fromID, toID] = connectionEdges.at(edge->id);

                const SUMO::Network::Edge &from = sumo.network.getEdge(fromID);
                const SUMO::Network::Edge &to   = sumo.network.getEdge(toID);

                const Edge *prevEdge = netEdges.at(adapter.toEdge(from.id));
                const Edge *nextEdge = netEdges.at(adapter.toEdge(to.id));

                const auto &conns = sumo.network.getConnections(from, to);

                if(!conns.empty()) {
                    map<SUMO::Network::Edge::ID, Graph::Node>       sumoEdges2nodes;
                    map<SUMO::Network::Edge::Lane::ID, Graph::Node> sumoLanes2nodes;

                    Graph           G;
                    Graph::Node     incNode = 0;
                    Graph::Edge::ID incEdge = 0;

                    Graph::Node vSource = incNode++;
                    Graph::Node vSink   = incNode++;

                    Graph::Node source = incNode++;
                    G.addEdge(incEdge++, vSource, source, min(min(prevEdge->c, nextEdge->c), edge->c));

                    for(const SUMO::Network::Connection &conn: conns) {
                        // This fromLane was never seen before
                        Graph::Node u;
                        if(!sumoLanes2nodes.count(conn.fromLane().id)) {
                            u = incNode++;

                            sumoLanes2nodes[conn.fromLane().id] = u;

                            G.addEdge(incEdge++, source, u, calculateCapacity(conn.fromLane()));
                        } else {
                            u = sumoLanes2nodes.at(conn.fromLane().id);
                        }

                        // This toLane was never seen before
                        Graph::Node v;
                        if(!sumoLanes2nodes.count(conn.toLane().id)) {
                            v = incNode++;

                            sumoLanes2nodes[conn.toLane().id] = v;

                            G.addEdge(incEdge++, v, vSink, calculateCapacity(conn.toLane()));
                        } else {
                            v = sumoLanes2nodes.at(conn.toLane().id);
                        }

                        G.addEdge(incEdge++, u, v, INFINITY);
                    }

                    Alg::ShortestPath::BFS   sp;
                    Alg::Flow::EdmondsKarp   maxFlow(sp);
                    Alg::Graph::Edge::Weight c = maxFlow.solve(G, vSource, vSink);

                    if(edge->c > c + EPSILON) {
                        // cerr << "    2.1. | "
                        //      << "Capacity of edge " << edge->id
                        //      << " was reduced from " << edge->c
                        //      << " to " << c
                        //      << " (delta=" << edge->c - c << ")"
                        //      << endl;
                        edge->c = c;
                        changed = true;
                    }
                }
            }
        }
    }
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::addTAZs(const SUMO::NetworkTAZs &sumo) {
    for(const auto &[id, taz]: sumo.tazs) {
        const auto &[source, sink] = adapter.addSumoTAZ(taz.id);
        for(const SUMO::TAZ::Source &s: taz.sources) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            network->addNormalEdge(
                adapter.addEdge(),
                source,
                e->u,
                *network,
                0,
                1e9
            );
        }
        for(const SUMO::TAZ::Sink &s: taz.sinks) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
            network->addNormalEdge(
                adapter.addEdge(),
                e->v,
                sink,
                *network,
                0,
                1e9
            );
        }
    }
}

void BPRNetwork::Loader<SUMO::NetworkTAZs>::clear() {
    adapter.clear();
    in.clear();
    out.clear();
    normalEdges.clear();
}

BPRNetwork::Loader<SUMO::NetworkTAZs>::Loader::~Loader() {}
