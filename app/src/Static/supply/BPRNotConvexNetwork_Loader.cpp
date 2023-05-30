#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <tuple>

#include "Alg/Flow/EdmondsKarp.hpp"
#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/BFS.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace Static;
using namespace utils;

using Alg::Graph;

typedef BPRNotConvexNetwork::Time Time;
typedef BPRNotConvexNetwork::Flow Flow;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;
typedef SUMO::Length              Length;

const Flow SATURATION_FLOW             = 1110.0;  // vehicles per hour per lane

/**
 * @brief Cost of having traffic stop once. This should be a time penalty that
 * averages all the negative effects of cars having to start after the light
 * turns green.
 */
const Time STOP_PENALTY = 0.0;

BPRNotConvexNetwork *BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::load(const SUMO::NetworkTAZs &sumo) {
    clear();

    networkNotConvex = new BPRNotConvexNetwork();
    network = networkNotConvex;

    addNormalEdges(sumo);

    addConnections(sumo);

    iterateCapacities(sumo);

    addDeadEnds(sumo);

    addTAZs(sumo);

    for(auto &[eID, _]: connectionEdges) {
        addConnectionConflicts(sumo, eID);
    }

    return networkNotConvex;
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addConnection(const SUMO::NetworkTAZs &sumo, const SUMO::Network::Edge &from, const SUMO::Network::Edge &to) {
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

    for(const SUMO::Network::Connection *connPtr: fromToConnections) {
        const SUMO::Network::Connection &conn = *connPtr;

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

    BPRNetwork::ConnectionEdge *e = network->addConnectionEdge(
        adapter.addEdge(),
        adapter.toNodes(from.id).second,
        adapter.toNodes(to.id).first,
        *network,
        t0,
        c
    );

    connectionEdges[e->id] = make_tuple(e, from.id, to.id);

    connectionMap[from.id][to.id] = e->id;
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addConnectionConflicts(const SUMO::NetworkTAZs &sumo, const Edge::ID &eID) {
    auto &[e_, fromID, toID] = connectionEdges.at(eID);
    ConnectionEdge *e        = dynamic_cast<ConnectionEdge*>(e_);
    assert(e != nullptr);

    const SUMO::Network::Edge &from = sumo.network.getEdge(fromID);
    const SUMO::Network::Edge &to   = sumo.network.getEdge(toID);

    const vector<const SUMO::Network::Connection *> &fromToConnections = sumo.network.getConnections(from, to);

    for(const SUMO::Network::Connection *connPtr: fromToConnections) {
        const SUMO::Network::Connection &conn = *connPtr;

        if(conn.tl) {
            e->conflicts.clear();
            return;
        }

        vector<const SUMO::Network::Connection *> response = conn.getRequest().getResponse();

        vector<pair<const Edge *, double>> conf;

        for(const SUMO::Network::Connection *rPtr: response) {
            const SUMO::Network::Connection &r = *rPtr;

            ConnectionEdge::ID cID = connectionMap.at(r.from.id).at(r.to.id);
            ConnectionEdge *c = dynamic_cast<ConnectionEdge*>(get<0>(connectionEdges.at(cID)));
            assert(c != nullptr);

            size_t n = getNumberLanes(sumo, *c);
            assert(n > 0);

            conf.push_back({c, 1.0 / (double)n});
        }

        e->conflicts.push_back(conf);
    }
}

size_t BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::getNumberLanes(const SUMO::NetworkTAZs &sumo, const ConnectionEdge &e) const {
    const SUMO::Network::Edge::ID &fromID = adapter.fromNodeToSumoEdge(e.u);
    const SUMO::Network::Edge::ID &toID   = adapter.fromNodeToSumoEdge(e.v);

    vector<const SUMO::Network::Connection *> connections = sumo.network.getConnections(
        sumo.network.getEdge(fromID),
        sumo.network.getEdge(toID)
    );
    set<SUMO::Index> fromLanes;
    set<SUMO::Index> toLanes;
    for(const SUMO::Network::Connection *connection: connections) {
        fromLanes.insert(connection->fromLane().index);
        toLanes.insert(connection->toLane().index);
    }
    return min(fromLanes.size(), toLanes.size());
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::iterateCapacities(const SUMO::NetworkTAZs &sumo) {
    map<Edge::ID, Edge *> &edges = networkNotConvex->edges;

    const Flow   EPSILON    = 1.0 / 60.0 / 60.0 / 24.0;
    const size_t ITERATIONS = 100;

    bool changed = true;

    for(size_t i = 0; i < ITERATIONS && changed; ++i) {
        // cerr << "it=" << i << endl;

        changed = false;

        for(auto &[edgeID, edge]: edges) {
            // 1.1
            {
                const auto &nextEdges = networkNotConvex->adj.at(edge->v);
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

        for(auto &[edgeID, edge]: edges) {
            // 1.2
            if(adapter.isSumoEdge(edge->id)) {
                const SUMO::Network::Edge::ID fromID = adapter.toSumoEdge(edge->id);
                const SUMO::Network::Edge    &from   = sumo.network.getEdge(fromID);

                const vector<const SUMO::Network::Connection *> &connections = from.getOutgoingConnections();

                if(!connections.empty()) {
                    map<SUMO::Network::Edge::ID, Graph::Node>       sumoEdges2nodes;
                    map<SUMO::Network::Edge::Lane::ID, Graph::Node> sumoLanes2nodes;

                    Graph           G;
                    Graph::Node     incNode = 0;
                    Graph::Edge::ID incEdge = 0;

                    Graph::Node vSource = incNode++;
                    Graph::Node vSink   = incNode++;

                    for(const SUMO::Network::Connection *connPtr: connections) {
                        const SUMO::Network::Connection &conn = *connPtr;

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
        for(auto &[edgeID, edge]: edges) {
            // 2.1
            if(!adapter.isSumoEdge(edge->id)) {
                const auto &[_, fromID, toID] = connectionEdges.at(edge->id);

                const SUMO::Network::Edge &from = sumo.network.getEdge(fromID);
                const SUMO::Network::Edge &to   = sumo.network.getEdge(toID);

                const Edge *prevEdge = edges.at(adapter.toEdge(from.id));
                const Edge *nextEdge = edges.at(adapter.toEdge(to.id));

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

                    for(const SUMO::Network::Connection *connPtr: conns) {
                        const SUMO::Network::Connection &conn = *connPtr;

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

        // TODO
    }
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addDeadEnds(const SUMO::NetworkTAZs &sumo) {
    const vector<SUMO::Network::Junction> &junctions = sumo.network.getJunctions();
    for(const SUMO::Network::Junction &junction: junctions) {
        // Allow vehicles to go in any direction in dead ends
        if(junction.type == SUMO::Network::Junction::DEAD_END) {
            for(const SUMO::Network::Edge &e1: in[junction.id]) {
                for(const SUMO::Network::Edge &e2: out[junction.id]) {
                    network->addNormalEdge(
                        adapter.addEdge(),
                        adapter.toNodes(e1.id).second,
                        adapter.toNodes(e2.id).first,
                        *network,
                        20,
                        1.0 / 20.0
                    );
                }
            }
        }
    }
}

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::addTAZs(const SUMO::NetworkTAZs &sumo) {
    for(const auto &[id, taz]: sumo.tazs) {
        const auto &[source, sink] = adapter.addSumoTAZ(taz.id);
        for(const SUMO::TAZ::Source &s: taz.sources) {
            const Edge *e = networkNotConvex->edges.at(adapter.toEdge(s.id));
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
            const Edge *e = networkNotConvex->edges.at(adapter.toEdge(s.id));
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

void BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>::clear() {
    adapter.clear();
    in.clear();
    out.clear();
    normalEdges.clear();
}
