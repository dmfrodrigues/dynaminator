#include "Dynamic/Env/Env.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdarg>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Alg/Graph.hpp"
#include "Dynamic/Demand/Demand.hpp"
#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventDump.hpp"
#include "Dynamic/Env/Event/EventLog.hpp"
#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Event/EventSpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateTrafficLight.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Log/ProgressLogger.hpp"

using namespace std;
using namespace Dynamic::Env;

typedef chrono::steady_clock clk;

// clang-format off
Env::Env(
    Dynamic::RewardFunction &rewardFunction_,
    Time startTime
):
    rewardFunction(rewardFunction_),
    t(startTime)
{}
// clang-format on

Dynamic::Time Env::getTime() const {
    return t;
}

size_t Env::getNumberProcessedEvents() const { return numberProcessedEvents; }

Alg::Graph Env::toGraph() const {
    Alg::Graph G;

    for(const auto &[_, edge]: edges) {
        Time                     travelTime = edge.length / edge.maxSpeed;
        Alg::Graph::Edge::Weight w          = travelTime;
        G.addEdge(edge.id, edge.u, edge.v, w);

        unordered_map<Edge::ID, Connection::ID> toNodes;
        for(const Connection &connection: edge.getOutgoingConnections()) {
            const Edge &to = connection.toLane.edge;

            Alg::Graph::Edge::Weight wConn = connection.getMinExpectedStopTimeTL();

            G.addEdge(connection.id + 1000000, edge.v, to.u, wConn);
        }
    }

    return G;
}

Edge &Env::addEdge(Edge::ID id, Node u, Node v, Length length, Speed speed, Edge::Priority priority, size_t nLanes) {
    if(edges.count(id))
        throw runtime_error("Edge " + to_string(id) + "already exists");
    return edges[id] = Edge(id, u, v, length, speed, priority, nLanes);
}

void Env::initializeTrafficLights(Time begin) {
    for(auto &[_, trafficLight]: trafficLights) {
        const auto &p = trafficLight.getPhase(begin);

        const auto &[phase, tStart] = p;

        pushEvent(make_shared<EventUpdateTrafficLight>(
            tStart,
            trafficLight,
            phase
        ));
    }
}

void Env::addDemand(const Demand &demand) {
    vector<Dynamic::Vehicle> vehs = demand.getVehicles();
    for(const Dynamic::Vehicle &vehicle: vehs) {
        eventQueue.push(make_shared<EventSpawnVehicle>(
            vehicle.depart,
            vehicle
        ));
    }
}

size_t Env::getNumberVehicles() const {
    return vehicles.size();
}

size_t Env::getQueueSize() const {
    return eventQueue.size();
}

void Env::pushEvent(shared_ptr<Event> event) {
    eventQueue.push(event);
}

TrafficLight &Env::addTrafficLight(TrafficLight::ID id, Time offset) {
    auto [it, success] = trafficLights.emplace(id, TrafficLight(id, offset));
    if(!success) throw runtime_error("TrafficLight already exists");

    TrafficLight &trafficLight = it->second;

    return trafficLight;
}

TrafficLight &Env::getTrafficLight(const TrafficLight::ID &id) {
    try {
        return trafficLights.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getTrafficLight: TrafficLight " + to_string(id) + " not found");
    }
}

Connection &Env::addConnection(Connection::ID id, Lane &fromLane, Lane &toLane) {
    auto [it, success] = connections.emplace(id, Connection(id, fromLane, toLane));
    if(!success) throw runtime_error("Connection with ID " + to_string(id) + "already exists");

    Connection &connection = it->second;

    // clang-format off
    if(
        !fromLane.outgoingConnections[toLane.edge.id].emplace(toLane.index, connection).second ||
        !toLane.incomingConnections[fromLane.edge.id].emplace(fromLane.index, connection).second
    )
        throw runtime_error(
            "Connection between lanes " +
            fromLane.idAsString() + " and " +
            toLane.idAsString() + " already exists"
        );
    // clang-format on

    return connection;
}

const Edge &Env::getEdge(const Edge::ID &id) const {
    try {
        return edges.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getEdge: Edge " + to_string(id) + " not found");
    }
}

Edge &Env::getEdge(const Edge::ID &id) {
    try {
        return edges.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getEdge: Edge " + to_string(id) + " not found");
    }
}

list<reference_wrapper<Edge>> Env::getEdges() {
    list<reference_wrapper<Edge>> edgesList;
    for(auto &[_, edge]: edges) {
        edgesList.push_back(edge);
    }
    return edgesList;
}

Vehicle &Env::getVehicle(const Vehicle::ID &id) {
    try {
        return vehicles.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getVehicle: Vehicle " + to_string(id) + " not found");
    }
}

list<reference_wrapper<Vehicle>> Env::getVehicles() {
    list<reference_wrapper<Vehicle>> vehiclesList;
    for(auto &[_, vehicle]: vehicles) {
        vehiclesList.push_back(vehicle);
    }
    return vehiclesList;
}

list<reference_wrapper<const Vehicle>> Env::getVehicles() const {
    list<reference_wrapper<const Vehicle>> vehiclesList;
    for(auto &[_, vehicle]: vehicles) {
        vehiclesList.push_back(vehicle);
    }
    return vehiclesList;
}

Connection       &Env::getConnection(const Connection::ID &id) { return connections.at(id); }
const Connection &Env::getConnection(const Connection::ID &id) const { return connections.at(id); }

TAZ &Env::addTAZ(TAZ::ID id) {
    auto [it, success] = tazs.emplace(id, TAZ(id));
    if(!success) throw runtime_error("TAZ already exists");

    TAZ &taz = it->second;

    return taz;
}

TAZ &Env::getTAZ(TAZ::ID id) {
    try {
        return tazs.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getTAZ: TAZ " + to_string(id) + " not found");
    }
}

list<reference_wrapper<const TAZ>> Env::getTAZs() const {
    list<reference_wrapper<const TAZ>> tazsList;
    for(auto &[_, taz]: tazs) {
        tazsList.push_back(taz);
    }
    return tazsList;
}

Vehicle &Env::addVehicle(Dynamic::Vehicle dynamicVehicle, Time t_, const Position &position, Speed speed) {
    auto [it, success] = vehicles.emplace(dynamicVehicle.id, Vehicle(dynamicVehicle, t_, position, speed, Vehicle::State::MOVING));
    if(!success) throw runtime_error("Vehicle already exists");

    Vehicle &vehicle = it->second;

    vehicle.position.lane.moving.insert(vehicle.id);

    return vehicle;
}

void Env::runUntil(Time tEnd) {
    while(!eventQueue.empty()) {
        if(eventQueue.top()->t > tEnd) break;

        shared_ptr<Event> event = eventQueue.top();
        eventQueue.pop();

        if(event->t < t) {
            spdlog::warn("Event time {} is less than current time {}", event->t, t);
        }

        t = max(t, event->t);

        ++numberProcessedEvents;

        event->process(*this);

        event.reset();
    }
}

void Env::updateAllVehicles(Time t_) {
    runUntil(t_);
    for(auto &[_, vehicle]: vehicles) {
        if(vehicle.state == Vehicle::State::LEFT) continue;
        EventMoveVehicle event(t_, vehicle);
        event.process(*this);
    }
    runUntil(t_);
}

void Env::log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta) {
    policyLogger = make_shared<Policy::Logger>();
    Env::log(logger, tStartSim, tEndSim, delta, *policyLogger);
}

void Env::log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta, Policy::Logger &pLogger) {
    logger << fixed << setprecision(6);

    logger << Log::ProgressLogger::Elapsed(0)
           << Log::ProgressLogger::Progress(0)
           << Log::ProgressLogger::ETA(1)
           << Log::ProgressLogger::StartText()
           << "t"
           << "\t#vehTot"
           << "\t#veh"
           << "\t#dspawn"
           << "\ttravelTime"
           << "\ttravelTInterval"
           << "\t#procE"
           << "\t";
    pLogger.header(logger);
    logger << Log::ProgressLogger::EndMessage();

    clk::time_point now = clk::now();
    for(Time time = tStartSim; time <= tEndSim; time += delta) {
        pushEvent(make_shared<EventLog>(
            time,
            tStartSim,
            tEndSim,
            now,
            logger,
            pLogger
        ));
    }
}

void Env::dump(SUMO::NetState &netState, const SUMOAdapter &adapter, Time tStartSim, Time delta, size_t numberDumps, bool closeAfterAllDumps) {
    for(size_t i = 0; i < numberDumps; ++i) {
        pushEvent(make_shared<EventDump>(
            tStartSim + (Time)i * delta,
            netState,
            adapter,
            closeAfterAllDumps && (i == numberDumps - 1)
        ));
    }
}

void Env::setDiscardVehicles(bool discardVehicles_) {
    discardVehicles = discardVehicles_;
}

void Env::discardVehicle(const Vehicle &vehicle) {
    if(discardVehicles) {
        vehicles.erase(vehicle.id);
    }
}

void Env::setDespawnTime(Time despawnTime_) {
    despawnTime = despawnTime_;
}

Dynamic::Time Env::getDespawnTime() const {
    return despawnTime;
}

size_t &Env::numberOfDespawnedVehicles() {
    return numberDespawnedVehicles;
}
