#pragma once

#include <chrono>
#include <functional>
#include <initializer_list>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>

#include "Alg/Graph.hpp"
#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/TAZ.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "Dynamic/Policy/Policy.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Log/ProgressLogger.hpp"
#include "utils/shared_ptr.hpp"

namespace SUMO {
class NetState;
}

namespace Dynamic {

class Demand;

class SUMOAdapter;

namespace Env {

class Env {
    friend Vehicle;

   public:
    typedef double Length;
    typedef double Speed;

    typedef long Node;

    typedef long VehicleID;
    typedef long ConnectionID;

   private:
    RewardFunction &rewardFunction;

    Time                                     t;
    std::map<TrafficLight::ID, TrafficLight> trafficLights;
    std::map<Edge::ID, Edge>                 edges;
    std::map<Connection::ID, Connection>     connections;
    std::map<TAZ::ID, TAZ>                   tazs;
    std::map<Vehicle::ID, Vehicle>           vehicles;
    // clang-format off
    std::priority_queue<
        std::shared_ptr<Event>,
        std::vector<std::shared_ptr<Event>>,
        utils::shared_ptr::greater<Event>
    > eventQueue;
    // clang-format on

    std::shared_ptr<Policy::Logger> policyLogger;

    bool discardVehicles = false;
    Time despawnTime     = std::numeric_limits<Time>::infinity();

    size_t numberProcessedEvents = 0;

    size_t numberDespawnedVehicles = 0;

   public:
    Env(
        RewardFunction &rewardFunction,
        Time            startTime = 0
    );

    Time getTime() const;

    size_t getNumberProcessedEvents() const;

    size_t getNumberVehicles() const;
    size_t getQueueSize() const;

    void pushEvent(std::shared_ptr<Event> event);

    TrafficLight &addTrafficLight(TrafficLight::ID id, Time offset);

    TrafficLight &getTrafficLight(const TrafficLight::ID &id);

    Connection &addConnection(Connection::ID id, Lane &from, Lane &to);

    Edge       &getEdge(const Edge::ID &id);
    const Edge &getEdge(const Edge::ID &id) const;

    std::list<std::reference_wrapper<Edge>> getEdges();

    Connection       &getConnection(const Connection::ID &id);
    const Connection &getConnection(const Connection::ID &id) const;

    TAZ &addTAZ(TAZ::ID id);

    TAZ &getTAZ(TAZ::ID id);

    std::list<std::reference_wrapper<const TAZ>> getTAZs() const;

    Vehicle &addVehicle(Dynamic::Vehicle dynamicVehicle, Time t, const Position &position, Speed speed);

    Vehicle &getVehicle(const Vehicle::ID &id);

    std::list<std::reference_wrapper<Vehicle>>       getVehicles();
    std::list<std::reference_wrapper<const Vehicle>> getVehicles() const;

    Alg::Graph toGraph() const;

    Edge &addEdge(Edge::ID id, Node u, Node v, Length length, Speed speed, Edge::Priority priority, size_t nLanes);

    void initializeTrafficLights(Time begin);

    void addDemand(const Demand &demand);

    void runUntil(Time t);

    void updateAllVehicles(Time t);

    void log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta);
    void log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta, Policy::Logger &policyLogger);

    void dump(SUMO::NetState &netState, const SUMOAdapter &adapter, Time tStartSim, Time delta, size_t numberDumps, bool closeAfterAllDumps = false);

    void setDiscardVehicles(bool discardVehicles);
    void discardVehicle(const Vehicle &vehicle);

    void setDespawnTime(Time despawnTime);
    Time getDespawnTime() const;

    size_t &numberOfDespawnedVehicles();
};

}  // namespace Env

}  // namespace Dynamic
