#include "Dynamic/Demand/Demand.hpp"

using namespace std;
using namespace Dynamic;

Vehicle &Demand::addVehicle(
    Time                             depart,
    Env::TAZ                        &fromTAZ,
    Env::TAZ                        &toTAZ,
    Env::Edge                       &from,
    Env::Edge                       &to,
    shared_ptr<Env::Vehicle::Policy> policy
) {
    return addVehicle(nextID, depart, fromTAZ, toTAZ, from, to, policy);
}

Vehicle &Demand::addVehicle(
    Vehicle::ID                      id,
    Time                             depart,
    Env::TAZ                        &fromTAZ,
    Env::TAZ                        &toTAZ,
    Env::Edge                       &from,
    Env::Edge                       &to,
    shared_ptr<Env::Vehicle::Policy> policy
) {
    vehicles.emplace_back(id, depart, fromTAZ, toTAZ, from, to, policy);
    nextID = max(nextID, id + 1);
    return vehicles.back();
}

const vector<Vehicle> &Demand::getVehicles() const {
    return vehicles;
}
