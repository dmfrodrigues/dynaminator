#include "Dynamic/Policy/RandomPolicy.hpp"

#include <list>
#include <random>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;

RandomPolicy::RandomPolicy(Vehicle::ID id_, shared_ptr<mt19937> gen_):
    id(id_), gen(gen_) {}

const Env::Lane &RandomPolicy::pickInitialLane(
    const Vehicle &vehicle,
    const Env::Env &
) {
    const vector<shared_ptr<Env::Lane>> &lanes = vehicle.from.lanes;

    uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

    auto it = lanes.begin();
    advance(it, lanesDistribution(*gen));

    const Env::Lane &lane = *(*it);

    return lane;
}

Vehicle::Policy::Intention RandomPolicy::pickConnection(
    const Env::Env &env
) {
    const Env::Vehicle &vehicle = env.getVehicle(id);

    const Env::Edge &edge = vehicle.position.lane.edge;

    list<std::reference_wrapper<Env::Connection>> connections = edge.getOutgoingConnections();

    if(connections.size() == 0) {
        return {Env::Connection::LEAVE, Env::Lane::INVALID};
    }

    uniform_int_distribution<size_t> connectionsDistribution(0, connections.size() - 1);

    auto it = connections.begin();
    advance(it, connectionsDistribution(*gen));

    const Env::Connection &connection = *it;

    const vector<shared_ptr<Env::Lane>> &lanes = connection.toLane.edge.lanes;

    uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

    auto itLane = lanes.begin();
    advance(itLane, lanesDistribution(*gen));

    const Env::Lane &lane = *(*itLane);

    return {connection, lane};
}

void RandomPolicy::feedback(const Env::Edge &e, Time t) {
}

RandomPolicy::Factory::Factory() {
    random_device rd;
    gen = make_shared<mt19937>(rd());
}

RandomPolicy::Factory::Factory(random_device::result_type seed) {
    gen = make_shared<mt19937>(seed);
}

shared_ptr<Vehicle::Policy> RandomPolicy::Factory::create(
    Vehicle::ID id,
    Time,
    const Env::Edge &,
    const Env::Edge &
) {
    return make_shared<RandomPolicy>(id, gen);
}