#include "Dynamic/Policy/RandomPolicy.hpp"

#include <list>
#include <random>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;

RandomPolicy::Action::Action(
    Env::Connection &connection_,
    Env::Lane       &lane_
):
    Vehicle::Policy::Action{connection_, lane_} {}

void RandomPolicy::Action::reward(Reward) {
    // Do nothing
}

RandomPolicy::RandomPolicy(Vehicle::ID id_, shared_ptr<mt19937> gen_):
    id(id_), gen(gen_) {}

Env::Lane &RandomPolicy::pickInitialLane(
    Vehicle &vehicle,
    Env::Env &
) {
    vector<Env::Lane> &lanes = vehicle.from.lanes;

    uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

    auto it = lanes.begin();
    advance(it, lanesDistribution(*gen));

    Env::Lane &lane = *it;

    return lane;
}

shared_ptr<Vehicle::Policy::Action> RandomPolicy::pickConnection(
    Env::Env &env
) {
    Env::Vehicle &vehicle = env.getVehicle(id);

    Env::Edge &edge = vehicle.position.lane.edge;

    list<reference_wrapper<Env::Connection>> connections = edge.getOutgoingConnections();

    if(connections.size() == 0) {
        return make_shared<RandomPolicy::Action>(Env::Connection::LEAVE, Env::Lane::INVALID);
    }

    uniform_int_distribution<size_t> connectionsDistribution(0, connections.size() - 1);

    auto it = connections.begin();
    advance(it, connectionsDistribution(*gen));

    Env::Connection &connection = *it;

    vector<Env::Lane> &lanes = connection.toLane.edge.lanes;

    uniform_int_distribution<size_t> lanesDistribution(0, lanes.size() - 1);

    auto itLane = lanes.begin();
    advance(itLane, lanesDistribution(*gen));

    Env::Lane &lane = *itLane;

    return make_shared<RandomPolicy::Action>(connection, lane);
}

RandomPolicy::Factory::Factory():
    gen(make_shared<mt19937>(0)) {}

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
