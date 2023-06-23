#include "Dynamic/Env/Env.hpp"

#include <chrono>
#include <cstdarg>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Alg/Graph.hpp"
#include "Dynamic/Demand/Demand.hpp"
#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventLog.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateTrafficLight.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "Log/ProgressLogger.hpp"

using namespace std;
using namespace Dynamic::Env;

typedef chrono::high_resolution_clock hrc;

// clang-format off
Env::Env(Time startTime):
    t(startTime)
{}
// clang-format on

Dynamic::Time Env::getTime() const {
    return t;
}

Alg::Graph Env::toGraph() const {
    Alg::Graph G;

    for(const auto &[_, edge]: edges) {
        Time                     t = edge.length / edge.calculateSpeed();
        Alg::Graph::Edge::Weight w = t;
        G.addEdge(edge.id, edge.u, edge.v, w);

        unordered_map<Edge::ID, Connection::ID> toNodes;
        for(const Connection &connection: edge.getOutgoingConnections()) {
            const Edge &to = connection.toLane.edge;
            if(!toNodes.count(to.id))
                toNodes[to.u] = connection.id;
        }

        for(const auto &[toNodeID, connectionID]: toNodes) {
            Alg::Graph::Edge::Weight w = 0;
            G.addEdge(connectionID + 1000000, edge.v, toNodeID, w);
        }
    }

    return G;
}

Edge &Env::addEdge(Edge::ID id, Node u, Node v, Length length, Speed speed, size_t nLanes) {
    if(edges.count(id))
        throw runtime_error("Edge " + to_string(id) + "already exists");
    return edges[id] = Edge(id, u, v, length, speed, nLanes);
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
        eventQueue.push(make_shared<EventTrySpawnVehicle>(
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
    if(!success) throw runtime_error("Connection already exists");

    Connection &connection = it->second;

    fromLane.outgoingConnections[toLane.edge.id].push_back(connection);
    toLane.incomingConnections[fromLane.edge.id].push_back(connection);

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
            cerr << "[WARN] " << __PRETTY_FUNCTION__ << ": event time " << event->t << " is less than current time " << t << endl;
        }

        t = max(t, event->t);

        // cout << "Processing event at time " << event->t << "(" << t << ")" << endl;

        event->process(*this);

        event.reset();
    }
}

void Env::updateAllVehicles(Time t_) {
    runUntil(t_);
    for(auto &[_, vehicle]: vehicles) {
        EventMoveVehicle event(t_, vehicle);
        event.process(*this);
    }
    runUntil(t_);
}

void Env::log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta) {
    hrc::time_point now = hrc::now();
    for(Time time = tStartSim; time <= tEndSim; time += delta) {
        eventQueue.push(make_shared<EventLog>(
            time,
            tStartSim,
            tEndSim,
            now,
            logger
        ));
    }
}
