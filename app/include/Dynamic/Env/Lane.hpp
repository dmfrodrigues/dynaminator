#pragma once

#include <cstddef>
#include <functional>
#include <list>
#include <map>
#include <queue>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
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
class Action;

class Lane {
    friend Env;

   public:
    typedef size_t Index;

    /**
     * @brief Frequency at which vehicles leave a queue.
     */
    static constexpr double JUNCTION_CAPACITY = 1600.0 / 60.0 / 60.0;
    static const double     JUNCTION_PERIOD;
    static const double     QUEUE_SPEED;

   private:
    typedef long EdgeID;

    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;
    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> incomingConnections;

   public:
    Edge &edge;
    Index index;

    std::queue<Dynamic::Vehicle> uninstantiated;

    std::set<VehicleID> moving;

    // TODO: move implementation to cpp file
    struct cmp {
        bool operator()(
            const std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Action>> &a,
            const std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Action>> &b
        ) const {
            return a.first.get().id < b.first.get().id;
        }
    };

    // clang-format off
    utils::orderstat::queue<
        std::pair<std::reference_wrapper<Vehicle>, std::shared_ptr<Action>>,
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

    Length queueLength() const;
    Length queuePosition() const;

    bool isFull() const;

    void processNextWaitingVehicle(Env &env);

    static Lane INVALID;
};
}  // namespace Dynamic::Env

namespace std {
template<>
struct hash<Dynamic::Env::Lane> {
    size_t operator()(const Dynamic::Env::Lane &lane) const;
};
}  // namespace std
