#include "Dynamic/Vehicle.hpp"

#include "Dynamic/Env/TAZ.hpp"
#include "Dynamic/Policy/Policy.hpp"

using namespace std;
using namespace Dynamic;

Vehicle::Vehicle(
    ID                 id_,
    Time               depart_,
    Env::TAZ          &fromTAZ_,
    Env::TAZ          &toTAZ_,
    shared_ptr<Policy> policy_
):
    id(id_),
    depart(depart_),
    fromTAZ(fromTAZ_),
    toTAZ(toTAZ_),
    policy(policy_) {}

Env::Lane &Vehicle::pickInitialLane(Env::Env &env) {
    return policy->pickInitialLane(*this, env);
}

bool Vehicle::operator<(const Vehicle &other) const {
    if(depart != other.depart)
        return depart < other.depart;
    else
        return id < other.id;
}
