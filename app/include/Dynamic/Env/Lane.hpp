#pragma once

#include <cstddef>
#include <deque>
#include <list>
#include <map>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

class Env;
class Edge;
class Connection;
class Vehicle;
class EventUpdateVehicle;
class EventPopQueue;

struct Lane {
    friend Env;
    friend Vehicle;
    friend EventUpdateVehicle;
    friend EventPopQueue;

    typedef size_t Index;

    Edge &edge;
    Index index;

   private:
    typedef long EdgeID;

    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;

    std::set<VehicleID> moving;

    std::deque<std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention>> stopped;

   public:
    Lane(Edge &edge, Index index);

    bool operator==(const Lane &other) const;
    bool operator!=(const Lane &other) const;
    bool operator<(const Lane &other) const;

    std::list<std::reference_wrapper<Connection>> getOutgoingConnections() const;
    std::list<std::reference_wrapper<Connection>> getOutgoingConnections(
        const Edge &nextEdge
    ) const;

    Speed calculateSpeed() const;

    static Lane INVALID;
};
}  // namespace Dynamic::Env
