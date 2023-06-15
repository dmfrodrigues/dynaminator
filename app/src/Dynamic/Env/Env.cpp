#include "Dynamic/Env/Env.hpp"

#include <chrono>
#include <cstdarg>
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
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Log/ProgressLogger.hpp"

using namespace std;
using namespace Dynamic::Env;

typedef chrono::high_resolution_clock hrc;

// clang-format off
Env::Env(Time startTime):
    t(startTime),
    eventQueue{[](
        const shared_ptr<Event> &a,
        const shared_ptr<Event> &b
    ) -> bool {
        return *a > *b;
    }}
{}
// clang-format on

Dynamic::Time Env::getTime() const {
    return t;
}

Alg::Graph Env::toGraph() const {
    Alg::Graph G;

    for(const auto &[_, edgePtr]: edges) {
        const Edge &edge = *edgePtr;

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
    return *(edges.emplace(id, shared_ptr<Edge>(new Edge(id, u, v, length, speed, nLanes))).first->second);
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

Connection &Env::addConnection(Connection::ID id, const Edge::Lane &fromLane, const Edge::Lane &toLane) {
    auto [it, success] = connections.emplace(id, make_shared<Connection>(id, fromLane, toLane));
    if(!success) throw runtime_error("Connection already exists");

    Connection &connection = *it->second;

    edges.at(fromLane.edge.id)->outgoingConnections[toLane.edge.id].push_back(connection);

    return connection;
}

const Edge &Env::getEdge(const Edge::ID &id) const {
    try {
        return *edges.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getEdge: Edge " + to_string(id) + " not found");
    }
}

Edge &Env::getEdge(const Edge::ID &id) {
    try {
        return *edges.at(id);
    } catch(const out_of_range &e) {
        throw out_of_range("Env::getEdge: Edge " + to_string(id) + " not found");
    }
}

const Vehicle &Env::getVehicle(const Vehicle::ID &id) const { return *vehicles.at(id); }

const std::map<Vehicle::ID, std::shared_ptr<Vehicle>> &Env::getVehicles() const { return vehicles; }

Connection       &Env::getConnection(const Connection::ID &id) { return *connections.at(id); }
const Connection &Env::getConnection(const Connection::ID &id) const { return *connections.at(id); }

Vehicle &Env::addVehicle(Dynamic::Vehicle dynamicVehicle, Time t, const Position &position, Speed speed) {
    auto [it, success] = vehicles.emplace(dynamicVehicle.id, make_shared<Vehicle>(dynamicVehicle, t, position, speed));
    if(!success) throw runtime_error("Vehicle already exists");

    Vehicle &vehicle = *it->second;

    edges.at(vehicle.position.edge.id)->vehicles.insert(vehicle.id);

    return *it->second;
}

void Env::removeVehicle(const Vehicle::ID &id) {
    Vehicle &vehicle = *vehicles.at(id);

    Edge &edge = *edges.at(vehicle.position.edge.id);

    if(edge.vehicles.erase(id) < 1)
        throw logic_error("Env::removeVehicle: Vehicle " + to_string(id) + " not found on edge " + to_string(edge.id));

    if(vehicles.erase(id) < 1)
        throw logic_error("Env::removeVehicle: Vehicle " + to_string(id) + " does not exist");
}

void Env::runUntil(Time tEnd) {
    while(!eventQueue.empty()) {
        if(eventQueue.top()->t > tEnd) break;

        shared_ptr<Event> event = eventQueue.top();
        eventQueue.pop();

        t = max(t, event->t);

        // cout << "Processing event at time " << event->t << "(" << t << ")" << endl;

        event->process(*this);

        event.reset();
    }
}

void Env::updateAllVehicles(Time t_) {
    runUntil(t_);
    for(const auto &[_, vehicle]: vehicles)
        eventQueue.push(make_shared<EventUpdateVehicle>(t_, *vehicle));
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
