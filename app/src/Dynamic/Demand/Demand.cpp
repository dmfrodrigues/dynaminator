#include "Dynamic/Demand.hpp"

using namespace std;
using namespace Dynamic;

bool Demand::Vehicle::operator<(const Vehicle &veh) const {
    return emissionTime < veh.emissionTime;
}

bool Demand::Vehicle::operator>(const Vehicle &veh) const {
    return veh < *this;
}

void Demand::addVehicle(Time emissionTime, Network::Node u, Network::Node v) {
    vehicles.push(Vehicle{emissionTime, u, v});
}

Demand::VehicleQueue Demand::getVehicles() const {
    return vehicles;
}
