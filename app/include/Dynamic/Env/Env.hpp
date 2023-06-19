#pragma once

#include <chrono>
#include <initializer_list>
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
#include "Dynamic/Env/TrafficLight.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "Log/ProgressLogger.hpp"

namespace Dynamic {

class Demand;

class Vehicle;

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
    Time                                                      t;
    std::map<TrafficLight::ID, std::shared_ptr<TrafficLight>> trafficLights;
    std::map<Edge::ID, std::shared_ptr<Edge>>                 edges;
    std::map<Connection::ID, std::shared_ptr<Connection>>     connections;
    std::map<Vehicle::ID, std::shared_ptr<Vehicle>>           vehicles;
    // clang-format off
    std::priority_queue<
        std::shared_ptr<Event>,
        std::vector<std::shared_ptr<Event>>,
        bool (*)(const std::shared_ptr<Event> &, const std::shared_ptr<Event> &)
    > eventQueue;
    // clang-format on

   public:
    Env(Time startTime = 0);

    Time getTime() const;

    size_t getNumberVehicles() const;
    size_t getQueueSize() const;

    void pushEvent(std::shared_ptr<Event> event);

    TrafficLight &addTrafficLight(TrafficLight::ID id, Time offset);

    TrafficLight &getTrafficLight(const TrafficLight::ID &id);

    Connection &addConnection(Connection::ID id, Lane &from, Lane &to);

    Edge       &getEdge(const Edge::ID &id);
    const Edge &getEdge(const Edge::ID &id) const;

    Connection       &getConnection(const Connection::ID &id);
    const Connection &getConnection(const Connection::ID &id) const;

    Vehicle &addVehicle(Dynamic::Vehicle dynamicVehicle, Time t, const Position &position, Speed speed);
    void     removeVehicle(const Vehicle::ID &id);

    const Vehicle                                         &getVehicle(const Vehicle::ID &id) const;
    const std::map<Vehicle::ID, std::shared_ptr<Vehicle>> &getVehicles() const;

    Alg::Graph toGraph() const;

    Edge &addEdge(Edge::ID id, Node u, Node v, Length length, Speed speed, size_t nLanes);

    void initializeTrafficLights(Time begin);

    void addDemand(const Demand &demand);

    void runUntil(Time t);

    void updateAllVehicles(Time t);

    void log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta);
};

}  // namespace Env

}  // namespace Dynamic
