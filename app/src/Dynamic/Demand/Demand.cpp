#include "Dynamic/Demand.hpp"

using namespace std;
using namespace Dynamic;

bool Demand::Vehicle::operator<(const Vehicle &veh) const {
    return emissionTime < veh.emissionTime;
}

bool Demand::Vehicle::operator>(const Vehicle &veh) const {
    return veh < *this;
}

Demand::Vehicle::ID Demand::addVehicle(Time emissionTime, EdgeID u, EdgeID v) {
    Vehicle::ID ret = nextID;
    addVehicle(nextID, emissionTime, u, v);
    return ret;
}

void Demand::addVehicle(Vehicle::ID id, Time emissionTime, EdgeID u, EdgeID v) {
    vehicles.emplace_back(Vehicle{id, emissionTime, u, v});
    nextID = max(nextID, id + 1);
}

const Demand::Vehicles &Demand::getVehicles() const {
    return vehicles;
}
