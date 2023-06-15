#include "Dynamic/Policy/RandomPolicy.hpp"

#include <list>
#include <random>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;

RandomPolicy::RandomPolicy(Vehicle::ID id_, shared_ptr<mt19937> gen_):
    id(id_), gen(gen_) {}

const Env::Connection &RandomPolicy::pickConnection(
    const Env::Env &env
) {
    const Env::Vehicle &vehicle = env.getVehicle(id);

    const Env::Edge &edge = vehicle.position.edge;

    list<std::reference_wrapper<Env::Connection>> connections = edge.getOutgoingConnections();

    if(connections.size() == 0) {
        return Env::Connection::LEAVE;
    }

    uniform_int_distribution<size_t> distribution(0, connections.size() - 1);

    size_t n = distribution(*gen);

    auto it = connections.begin();
    advance(it, n);

    const Env::Connection &connection = *it;

    return connection;
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
