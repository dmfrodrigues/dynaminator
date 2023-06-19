#include "Dynamic/Vehicle.hpp"

using namespace std;
using namespace Dynamic;

Vehicle::Vehicle(
    ID                 id_,
    Time               depart_,
    const Env::Edge   &from_,
    const Env::Edge   &to_,
    shared_ptr<Policy> policy_
):
    id(id_), depart(depart_), from(from_), to(to_), policy(policy_) {}

Env::Lane &Vehicle::pickInitialLane(const Env::Env &env) const {
    return policy->pickInitialLane(*this, env);
}
