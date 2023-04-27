#include <numeric>

#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace Static;

typedef BPRNetwork::Cost     Cost;
typedef BPRNetwork::Capacity Capacity;

typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Speed               Speed;
typedef SUMO::Length              Length;

Cost calculateFreeFlowSpeed(const SUMO::Network::Edge &e) {
    return e.speed() * 0.9;
}

Cost calculateFreeFlowSpeed(const SUMO::Network::Edge::Lane &l) {
    return l.speed * 0.9;
}

Cost calculateFreeFlowTime(const SUMO::Network::Edge &e) {
    Length length        = e.length();
    Speed  freeFlowSpeed = calculateFreeFlowSpeed(e);
    Cost   freeFlowTime  = length / freeFlowSpeed;
    return freeFlowTime;
}

Cost calculateFreeFlowTime(const SUMO::Network::Edge::Lane &l) {
    Length length        = l.length;
    Speed  freeFlowSpeed = calculateFreeFlowSpeed(l);
    Cost   freeFlowTime  = length / freeFlowSpeed;
    return freeFlowTime;
}

const Capacity SATURATION_FLOW = 1110.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 1800.0;  // vehicles per hour per lane
// const Cost SATURATION_FLOW = 2000.0;  // vehicles per hour per lane

/**
 * @brief Cost of having traffic stop once. This should be a time penalty that
 * averages all the negative effects of cars having to start after the light
 * turns green.
 */
const Cost STOP_PENALTY = 0.0;

Capacity calculateCapacity(const SUMO::Network::Edge &e, const SUMO::Network &sumoNetwork) {
    const auto &connections = sumoNetwork.getConnections();

    Speed freeFlowSpeed     = calculateFreeFlowSpeed(e);
    Cost  adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / (50.0 / 3.6));
    Capacity  c                 = adjSaturationFlow * (Cost)e.lanes.size();

    vector<Capacity> capacityPerLane(e.lanes.size(), 0.0);
    if(connections.count(e.id)) {
        for(const auto &[eNextID, eConnections]: connections.at(e.id)) {
            for(const SUMO::Network::Connection &conn: eConnections) {
                Capacity cAdd = adjSaturationFlow;
                if(conn.tl) {
                    SUMO::Time
                        g    = conn.tl->getGreenTime((size_t)conn.linkIndex),
                        C    = conn.tl->getCycleTime();
                    size_t n = conn.tl->getNumberStops(conn.linkIndex);
                    cAdd *= (g - STOP_PENALTY * (Cost)n) / C;
                }
                capacityPerLane.at(conn.fromLane().index) += cAdd;
            }
        }
    }
    for(Capacity &capPerLane: capacityPerLane)
        capPerLane = min(capPerLane, adjSaturationFlow);
    Capacity cNew = accumulate(capacityPerLane.begin(), capacityPerLane.end(), 0.0);
    if(cNew != 0.0) {
        c = min(c, cNew);
    }

    return c;
}

Capacity calculateCapacity(const SUMO::Network::Edge::Lane &lane) {
    Speed    freeFlowSpeed     = calculateFreeFlowSpeed(lane);
    Cost     adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (freeFlowSpeed / (50.0 / 3.6));
    Capacity c                 = adjSaturationFlow;
    return c;
}

BPRNetwork *BPRNetwork::Loader<SUMO::NetworkTAZs>::load(const SUMO::NetworkTAZs &sumo) {
    adapter.clear();

    BPRNetwork *network = new BPRNetwork();

    map<SUMO::Network::Junction::ID, list<SUMO::Network::Edge>> in, out;

    // clang-format off
    unordered_map<
        SUMO::Network::Edge::ID, unordered_map<
            SUMO::Network::Edge::ID,
            list<SUMO::Network::Connection>
        >
    > connections = sumo.network.getConnections();
    // clang-format on

    map<SUMO::Network::Edge::ID, NormalEdge *> normalEdges;

    const vector<SUMO::Network::Edge> &sumoEdges = sumo.network.getEdges();
    for(const SUMO::Network::Edge &edge: sumoEdges) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const auto           &p   = adapter.addSumoEdge(edge.id);
        const NormalEdge::ID &eid = p.first;
        Node                  u = p.second.first, v = p.second.second;

        // clang-format off
        network->addEdge(normalEdges[edge.id] = new NormalEdge(
            eid,
            u, v,
            *network,
            calculateFreeFlowTime(edge),
            calculateCapacity(edge, sumo.network)
        ));
        // clang-format on

        in[edge.to->id].push_back(edge);
        out[edge.from->id].push_back(edge);
    }

    for(const auto &[fromID, fromConnections]: connections) {
        if(!adapter.isEdge(fromID)) continue;

        const SUMO::Network::Edge &from = sumo.network.getEdge(fromID);

        for(const auto &[toID, fromToConnections]: fromConnections) {
            if(!adapter.isEdge(fromID)) continue;

            const SUMO::Network::Edge &to = sumo.network.getEdge(toID);

            Speed v = min(
                calculateFreeFlowSpeed(from),
                calculateFreeFlowSpeed(to)
            );
            Cost adjSaturationFlow = (SATURATION_FLOW / 60.0 / 60.0) * (v / (50.0 / 3.6));

            vector<Cost> capacityFromLanes(from.lanes.size(), 0.0);
            vector<Cost> capacityToLanes(to.lanes.size(), 0.0);

            double t0 = 0;
            double c  = 0;

            for(const SUMO::Network::Connection &conn: fromToConnections) {
                Cost cAdd = adjSaturationFlow;
                if(conn.tl) {
                    SUMO::Time
                        g    = conn.tl->getGreenTime((size_t)conn.linkIndex),
                        C    = conn.tl->getCycleTime();
                    size_t n = conn.tl->getNumberStops(conn.linkIndex);
                    cAdd *= (g - STOP_PENALTY * (double)n) / C;
                }
                c += cAdd;

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

            network->addEdge(new ConnectionEdge(
                adapter.addEdge(),
                adapter.toNodes(fromID).second,
                adapter.toNodes(toID).first,
                *network,
                t0,
                c
            ));
        }
    }

    const vector<SUMO::Network::Junction> &junctions = sumo.network.getJunctions();
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
                        1.0 / 20.0
                    ));
                }
            }
        }
    }

    for(const auto &[id, taz]: sumo.tazs) {
        const auto &[source, sink] = adapter.addSumoTAZ(taz.id);
        for(const SUMO::TAZ::Source &s: taz.sources) {
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
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
            const Edge *e = network->edges.at(adapter.toEdge(s.id));
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

    return network;
}
