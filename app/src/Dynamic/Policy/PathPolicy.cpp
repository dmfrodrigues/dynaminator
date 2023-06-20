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

PathPolicy::Action::Action(
    Env::Connection &connection_,
    Env::Lane       &lane_
):
    Vehicle::Policy::Action{connection_, lane_} {}

void PathPolicy::Action::reward(Time) {
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
    const Env::Edge &edge = vehicle.from;

    Env::Edge::ID nextEdgeID = nextEdgeMap.at(edge.id);
    if(nextEdgeID == END) {
        const vector<shared_ptr<Env::Lane>> &lanes = vehicle.from.lanes;

        uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

        auto it = lanes.begin();
        advance(it, lanesDistribution(*gen));

        Env::Lane &lane = *(*it);

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

std::shared_ptr<Vehicle::Policy::Action> PathPolicy::pickConnection(Env::Env &env) {
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

    const Env::Edge &nextEdge = env.getEdge(nextEdgeID);

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
        const vector<shared_ptr<Env::Lane>> &lanes = nextEdge.lanes;
        uniform_int_distribution<size_t>     lanesDistribution(0, lanes.size() - 1);

        auto itLane = lanes.begin();
        advance(itLane, lanesDistribution(*gen));

        Env::Lane &lane = **itLane;

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

void PathPolicy::feedback(const Env::Edge &, Time) {
}

PathPolicy::ShortestPathFactory::ShortestPathFactory(
    const Alg::ShortestPath::ShortestPathManyMany &sp_
):
    sp(sp_),
    gen(make_shared<mt19937>(random_device()())) {}

PathPolicy::ShortestPathFactory::ShortestPathFactory(
    const Alg::ShortestPath::ShortestPathManyMany &sp_,
    random_device::result_type                     seed
):
    sp(sp_),
    gen(make_shared<mt19937>(seed)) {}

shared_ptr<Vehicle::Policy> PathPolicy::ShortestPathFactory::create(
    Vehicle::ID id,
    Time,
    const Env::Edge &from,
    const Env::Edge &to
) {
    Alg::Graph::Path path = sp.getPath(from.u, to.v);

    return make_shared<PathPolicy>(id, path, gen);
}
