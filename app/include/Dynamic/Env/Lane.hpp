#pragma once

#include <cstddef>
#include <functional>
#include <list>
#include <map>
#include <queue>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "utils/orderstat.hpp"

namespace Dynamic::Env {

class Env;
class Edge;
class Connection;
class Vehicle;
class EventUpdateVehicle;
class EventMoveVehicle;
class EventPopQueue;

struct Lane {
    friend Env;
    friend Vehicle;
    friend EventUpdateVehicle;
    friend EventPopQueue;
    friend EventMoveVehicle;

    typedef size_t Index;

    Edge &edge;
    Index index;

   private:
    typedef long EdgeID;

    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;

    std::set<VehicleID> moving;

    // std::queue<std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention>> stopped;

    struct cmp {
        bool operator()(
            const std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention> &a,
            const std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention> &b
        ) const {
            return a.first.get().id < b.first.get().id;
        }
    };

    // clang-format off
    utils::orderstat::queue<
        std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention>,
        cmp
    > stopped;
    // clang-format on

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
