#include "Dynamic/Demand.hpp"

using namespace std;
using namespace Dynamic;

bool Demand::Vehicle::operator<(const Vehicle &veh) const {
    return emissionTime < veh.emissionTime;
}

bool Demand::Vehicle::operator>(const Vehicle &veh) const {
    return veh < *this;
}

void Demand::addVehicle(Time emissionTime, EdgeID u, EdgeID v) {
    addVehicle(nextID, emissionTime, u, v);
}

void Demand::addVehicle(Vehicle::ID id, Time emissionTime, EdgeID u, EdgeID v) {
    vehicles.push({id, emissionTime, u, v});
    nextID = max(nextID, id + 1);
}

Demand::VehicleQueue Demand::getVehicles() const {
    return vehicles;
}
