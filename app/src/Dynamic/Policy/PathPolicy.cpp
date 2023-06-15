#include "Dynamic/Policy/PathPolicy.hpp"

#include <cassert>
#include <memory>
#include <random>
#include <stdexcept>

#include "Alg/Graph.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Vehicle.hpp"

using namespace std;
using namespace Dynamic;

typedef Alg::Graph::Path Path;

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

        nextEdge[u] = v;
    }
}

const Env::Connection &PathPolicy::pickConnection(const Env::Env &env) {
    const Env::Vehicle &vehicle = env.getVehicle(id);
    const Env::Edge    &edge    = vehicle.position.edge;

    Env::Edge::ID nextEdgeID;
    try {
        nextEdgeID = nextEdge.at(edge.id);
    } catch(out_of_range &e) {
        throw out_of_range("PathPolicy::pickConnection: Edge " + to_string(edge.id) + " does not belong to the path of vehicle " + to_string(id));
    }

    if(nextEdgeID == END) {
        return Env::Connection::LEAVE;
    }

    const Env::Edge &nextEdge = env.getEdge(nextEdgeID);

    list<reference_wrapper<Env::Connection>> connections =
        edge.getOutgoingConnections(nextEdge);

    if(connections.empty()) {
        // clang-format off
        throw out_of_range(
            "PathPolicy::pickConnection: Edge " + to_string(edge.id) +
            " does not have any outgoing connections to edge " + to_string(nextEdge.id) +
            "; vehicle ID is " + to_string(id)
        );
        // clang-format on
    }

    std::uniform_int_distribution<size_t> distribution(0, connections.size() - 1);

    size_t n = distribution(*gen);

    auto it = connections.begin();
    advance(it, n);

    return *it;
}

void PathPolicy::feedback(const Env::Edge &e, Time t) {
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
