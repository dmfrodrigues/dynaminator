#include "Dynamic/Policy/PathPolicy.hpp"

#include <cassert>
#include <memory>
#include <random>
#include <stdexcept>

#include "Alg/Graph.hpp"
#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic;

typedef Alg::Graph::Path Path;

const Env::Edge::ID PathPolicy::START = -1;
const Env::Edge::ID PathPolicy::END   = -2;

PathPolicy::Action::Action(
    Env::Connection &connection_,
    Env::Lane       &lane_
):
    Env::Action{connection_, lane_} {}

void PathPolicy::Action::reward(Reward) {
    // Do nothing
}

PathPolicy::PathPolicy(
    Vehicle::ID             id_,
    const Alg::Graph::Path &path,
    shared_ptr<mt19937>     gen_
):
    id(id_),
    gen(gen_) {
    assert(path.size() % 2 == 1);

    Alg::Graph::Path newPath;
    for(size_t i = 0; i < path.size(); ++i) {
        // Because edges with odd indices are connections, so not normal edges
        if(i % 2 == 0) newPath.push_back(path[i]);
    }

    nextEdgeMap[START] = newPath.front().id;

    for(
        Path::const_iterator it     = newPath.begin(),
                             nextIt = ++newPath.begin();
        it != newPath.end();
        ++it, ++nextIt
    ) {
        Env::Edge::ID u = (Env::Edge::ID)it->id;
        Env::Edge::ID v = (nextIt == newPath.end() ? END : (Env::Edge::ID)nextIt->id);

        nextEdgeMap[u] = v;
    }
}

Env::Lane &PathPolicy::pickInitialLane(
    Vehicle  &vehicle,
    Env::Env &env
) {
    Env::Edge::ID edgeID = nextEdgeMap.at(START);
    Env::Edge    &edge   = env.getEdge(edgeID);

    Env::Edge::ID nextEdgeID = nextEdgeMap.at(edge.id);
    if(nextEdgeID == END) {
        vector<Env::Lane> &lanes = edge.lanes;

        uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

        auto it = lanes.begin();
        advance(it, lanesDistribution(*gen));

        Env::Lane &lane = *it;

        return lane;
    } else {
        const Env::Edge &nextEdge = env.getEdge(nextEdgeID);

        list<reference_wrapper<Env::Connection>> connections =
            edge.getOutgoingConnections(nextEdge);

        if(connections.empty()) {
            // clang-format off
            throw out_of_range(
                "PathPolicy::pickInitialLane: Edge " + to_string(edge.id) +
                " does not have any outgoing connections to edge " + to_string(nextEdge.id) +
                "; vehicle ID is " + to_string(id)
            );
            // clang-format on
        }

        uniform_int_distribution<size_t> connectionsDistribution(0, connections.size() - 1);

        auto it = connections.begin();
        advance(it, connectionsDistribution(*gen));

        const Env::Connection &connection = *it;

        Env::Lane &lane = connection.fromLane;

        return lane;
    }
}

shared_ptr<Env::Action> PathPolicy::pickConnection(Env::Env &env) {
    const Env::Vehicle &vehicle = env.getVehicle(id);
    const Env::Lane    &lane    = vehicle.position.lane;
    const Env::Edge    &edge    = lane.edge;

    Env::Edge::ID nextEdgeID;
    try {
        nextEdgeID = nextEdgeMap.at(edge.id);
    } catch(out_of_range &e) {
        throw out_of_range("PathPolicy::pickConnection: Edge " + to_string(edge.id) + " does not belong to the path of vehicle " + to_string(id));
    }

    if(nextEdgeID == END) {
        return make_shared<PathPolicy::Action>(Env::Connection::LEAVE, Env::Lane::INVALID);
    }

    Env::Edge &nextEdge = env.getEdge(nextEdgeID);

    list<reference_wrapper<Env::Connection>> connections =
        lane.getOutgoingConnections(nextEdge);

    if(connections.empty()) {
        // clang-format off
        throw out_of_range(
            "PathPolicy::pickConnection: Edge " + to_string(edge.id) +
            " does not have any outgoing connections to edge " + to_string(nextEdge.id) +
            "; vehicle ID is " + to_string(id)
        );
        // clang-format on
    }

    uniform_int_distribution<size_t> connectionsDistribution(0, connections.size() - 1);

    auto itConnection = connections.begin();
    advance(itConnection, connectionsDistribution(*gen));

    Env::Connection &connection = *itConnection;

    Env::Edge::ID nextNextEdgeID;
    try {
        nextNextEdgeID = nextEdgeMap.at(nextEdgeID);
    } catch(out_of_range &e) {
        throw out_of_range("PathPolicy::pickConnection: Edge " + to_string(nextEdgeID) + " does not belong to the path of vehicle " + to_string(id));
    }

    if(nextNextEdgeID == END) {
        vector<Env::Lane>               &lanes = nextEdge.lanes;
        uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

        auto itLane = lanes.begin();
        advance(itLane, lanesDistribution(*gen));

        Env::Lane &lane = *itLane;

        return make_shared<PathPolicy::Action>(connection, lane);
    } else {
        const Env::Edge &nextNextEdge = env.getEdge(nextNextEdgeID);

        list<reference_wrapper<Env::Connection>> nextConnections =
            nextEdge.getOutgoingConnections(nextNextEdge);

        if(nextConnections.empty()) {
            // clang-format off
            throw out_of_range(
                "PathPolicy::pickConnection: Edge " + to_string(nextEdge.id) +
                " does not have any outgoing connections to edge " + to_string(nextNextEdge.id) +
                "; vehicle ID is " + to_string(id)
            );
            // clang-format on
        }

        uniform_int_distribution<size_t> nextConnectionsDistribution(0, nextConnections.size() - 1);

        auto itNextConnection = nextConnections.begin();
        advance(itNextConnection, nextConnectionsDistribution(*gen));

        const Env::Connection &nextConnection = *itNextConnection;

        return make_shared<PathPolicy::Action>(connection, nextConnection.fromLane);
    }
}

PathPolicy::ShortestPathFactory::ShortestPathFactory(const Env::Env &env):
    ShortestPathFactory(env, 0) {}

PathPolicy::ShortestPathFactory::ShortestPathFactory(
    const Env::Env            &env,
    random_device::result_type seed
):
    gen(make_shared<mt19937>(seed)) {
    Alg::Graph G = env.toGraph();

    vector<Alg::Graph::Node> startNodes;
    for(const Env::TAZ &taz: env.getTAZs())
        for(const Env::Edge &source: taz.sources)
            startNodes.push_back(source.u);

    sp.solve(G, startNodes);
}

shared_ptr<Policy> PathPolicy::ShortestPathFactory::create(
    Vehicle::ID id,
    Time,
    const Env::TAZ &fromTAZ,
    const Env::TAZ &toTAZ
) {
    const size_t NUMBER_TRIES = 100;
    for(size_t i = 0; i < NUMBER_TRIES; ++i) {
        uniform_int_distribution<size_t> fromDistribution(0, fromTAZ.sources.size() - 1);
        uniform_int_distribution<size_t> toDistribution(0, toTAZ.sinks.size() - 1);

        auto itFrom = fromTAZ.sources.begin();
        advance(itFrom, fromDistribution(*gen));

        auto itTo = toTAZ.sinks.begin();
        advance(itTo, toDistribution(*gen));

        const Env::Edge &from = *itFrom;
        const Env::Edge &to   = *itTo;

        if(!sp.hasVisited(from.u, to.v)) {
            continue;
        }

        Alg::Graph::Path path = sp.getPath(from.u, to.v);

        return make_shared<PathPolicy>(id, path, gen);
    }

    // clang-format off
    throw logic_error(
        "PathPolicy::ShortestPathFactory::create: "s +
        "Could not find a path between TAZs " + to_string(fromTAZ.id) + 
        " and " + to_string(toTAZ.id)
    );
    // clang-format on
}
