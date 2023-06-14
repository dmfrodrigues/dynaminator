#include "Dynamic/Demand/Demand.hpp"

using namespace std;
using namespace Dynamic;

Vehicle &Demand::addVehicle(Time depart, const Env::Edge &from, const Env::Edge &to, shared_ptr<Env::Vehicle::Policy> policy) {
    return addVehicle(nextID, depart, from, to, policy);
}

Vehicle &Demand::addVehicle(Vehicle::ID id, Time depart, const Env::Edge &from, const Env::Edge &to, shared_ptr<Env::Vehicle::Policy> policy) {
    vehicles.emplace_back(id, depart, from, to, policy);
    nextID = max(nextID, id + 1);
    return vehicles.back();
}

const vector<Vehicle> &Demand::getVehicles() const {
    return vehicles;
}
