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

class Lane {
    friend Env;
    friend Vehicle;
    friend EventUpdateVehicle;
    friend EventPopQueue;
    friend EventMoveVehicle;

   public:
    typedef size_t Index;

    Edge &edge;
    Index index;

   private:
    typedef long EdgeID;

    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;
    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> incomingConnections;

    std::set<VehicleID> moving;

    // std::queue<std::pair<std::reference_wrapper<Vehicle>, Vehicle::Policy::Intention>> stopped;

    struct cmp {
        bool operator()(
            const std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Vehicle::Policy::Action>> &a,
            const std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Vehicle::Policy::Action>> &b
        ) const {
            return a.first.get().id < b.first.get().id;
        }
    };

    // clang-format off
    utils::orderstat::queue<
        std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Vehicle::Policy::Action>>,
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

    std::list<std::reference_wrapper<Connection>> getIncomingConnections() const;

    Speed calculateSpeed() const;

    static Lane INVALID;
};
}  // namespace Dynamic::Env

namespace std {
template<>
struct hash<Dynamic::Env::Lane> {
    size_t operator()(const Dynamic::Env::Lane &lane) const;
};
}  // namespace std
