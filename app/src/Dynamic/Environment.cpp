#include "Dynamic/Environment.hpp"

#include <cstdarg>
#include <iostream>
#include <memory>
#include "Alg/Graph.hpp"

using namespace std;
using namespace Dynamic;

/// === Environment ===========================================================

Environment::Edge::Edge(ID id_, Node u_, Node v_, Length length_, size_t nLanes_, Speed speed_):
    id(id_), u(u_), v(v_), length(length_), nLanes(nLanes_), speed(speed_) {}

Environment::Speed Environment::Edge::calculateSpeed() const {
    return 50.0 / 3.6;
}

list<Environment::Connection::ID> Environment::Edge::getOutgoingConnections() const {
    list<Connection::ID> ret;
    for(const auto &[_, conns]: outgoingConnections)
        ret.insert(ret.end(), conns.begin(), conns.end());
    return ret;
}

list<Environment::Connection::ID> Environment::Edge::getOutgoingConnections(Edge::ID destinationEdgeID) const {
    if(outgoingConnections.count(destinationEdgeID))
        return outgoingConnections.at(destinationEdgeID);
    else
        return {};
}

Environment::Event::Event(Time t_):
    t(t_) {}

Time Environment::Event::getTime() const { return t; }

bool Environment::Event::operator<(const Event &event) const {
    return t < event.t;
}

bool Environment::Event::operator>(const Event &event) const {
    return event < *this;
}

// clang-format off
Environment::Environment(Time startTime):
    t(startTime),
    eventQueue{[](
        const shared_ptr<Event> &a,
        const shared_ptr<Event> &b
    ) -> bool {
        return *a > *b;
    }}
{}
// clang-format on

Alg::Graph Environment::toGraph() const {
    Alg::Graph G;

    for(const auto &[_, edge]: edges) {
        Time t = edge.length / edge.calculateSpeed();
        Alg::Graph::Edge::Weight w = t;
        G.addEdge(edge.id, edge.u, edge.v, w);
    }

    return G;
}

Environment::Edge &Environment::addEdge(Edge::ID id, Node u, Node v, Length length, size_t nLanes, Speed speed) {
    return edges.emplace(id, Edge(id, u, v, length, nLanes, speed)).first->second;
}

void Environment::addDemand(const Demand &demand) {
    Demand::Vehicles vehicles = demand.getVehicles();
    for(const Demand::Vehicle &vehicle: vehicles){

        // cerr << "Environment::addDemand: adding vehicle " << vehicle.id
        // << " at time " << vehicle.emissionTime
        // << " from " << vehicle.u
        // << " to " << vehicle.v
        // << "\n";

        eventQueue.push(make_shared<EventTrySpawnVehicle>(
            vehicle.emissionTime,
            vehicle
        ));
    }
}

map<Environment::Edge::ID, Environment::Edge>       &Environment::getEdges() { return edges; }
map<Environment::Vehicle::ID, Environment::Vehicle> &Environment::getVehicles() { return vehicles; }

void Environment::runUntil(Time t) {
    while(!eventQueue.empty()) {
        if(eventQueue.top()->getTime() > t) break;

        shared_ptr<Event> event = eventQueue.top();
        eventQueue.pop();

        // cerr << "Processing event at time " << event->getTime() << endl;

        t = max(t, event->getTime());

        event->process(*this);
    }
}

void Environment::updateAllVehicles(Time t_) {
    for(const auto &[_, vehicle]: vehicles)
        eventQueue.push(make_shared<EventUpdateVehicle>(t_, vehicle.id));
    runUntil(t_);
}

/// === EventComposite ========================================================

Environment::EventComposite::EventComposite(Time t_):
    Event(t_) {}
Environment::EventComposite::EventComposite(Time t_, initializer_list<shared_ptr<Event>> events_):
    Event(t_), events(events_) {}

void Environment::EventComposite::addEvent(shared_ptr<Event> event) { events.push_back(event); }
void Environment::EventComposite::process(Environment &env) const {
    for(auto &event: events) event->process(env);
}

/// === EventTrySpawnVehicle ==================================================

Environment::EventTrySpawnVehicle::EventTrySpawnVehicle(Time t_, const Demand::Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void Environment::EventTrySpawnVehicle::process(Environment &env) const {
    // clang-format off
    Vehicle &envVehicle = env.vehicles.emplace(vehicle.id, Vehicle{
        vehicle.id, 
        getTime(), 
        Position{vehicle.u, 0}, 
        env.edges.at(vehicle.u).calculateSpeed()
    }).first->second;
    // clang-format on

    env.edges.at(envVehicle.position.edge).vehicles.insert(envVehicle.id);

    Time Dt      = env.edges.at(envVehicle.position.edge).length / envVehicle.speed;
    Time tFuture = env.t + Dt;

    // cerr << "    EventTrySpawnVehicle: vehicle " << vehicle.id
    // << " at time " << getTime()
    // << ", creating future events for time " << tFuture
    // << "=" << env.t << "+" << Dt
    // << endl;

    // clang-format off
    env.eventQueue.push(shared_ptr<Event>(new EventComposite(
        tFuture,
        {
            make_shared<EventUpdateVehicle>(tFuture, envVehicle.id),
            make_shared<EventPickConnection>(tFuture, envVehicle.id)
        }
    )));
    // clang-format on
}

/// === EventUpdateVehicle ====================================================

Environment::EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle::ID vehicleID_):
    Event(t_), vehicleID(vehicleID_) {}

void Environment::EventUpdateVehicle::process(Environment &env) const {
    // cerr << "    EventUpdateVehicle: vehicle " << vehicleID << " at time " << getTime() << endl;

    Vehicle &vehicle = env.vehicles.at(vehicleID);

    Time Dt = getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;
}

/// === EventPickConnection ===================================================

Environment::EventPickConnection::EventPickConnection(Time t_, Vehicle::ID vehicleID_):
    Event(t_), vehicleID(vehicleID_) {}

void Environment::EventPickConnection::process(Environment &env) const {
    // cerr << "    EventPickConnection: vehicle " << vehicleID << " at time " << getTime() << endl;

    Vehicle &vehicle = env.vehicles.at(vehicleID);
    Edge    &edge    = env.edges.at(vehicle.position.edge);

    // Pick random connection
    list<Connection::ID> connections = edge.getOutgoingConnections();
    
    Edge *toEdge = nullptr;

    if(connections.empty()) {
        cerr << "Warning: "
             << "Vehicle " << vehicleID
             << " reached a dead end at edge " << edge.id
             << "; sending vehicle to beginning of same edge."
             << endl;

        toEdge = &edge;

        return;
    } else {
        size_t i  = rand() % connections.size();
        auto   it = connections.begin();
        advance(it, i);
        Connection::ID connectionID = *it;
        Connection &connection = env.connections.at(connectionID);
        toEdge = &env.edges.at(connection.toID);
    }

    // Apply connection
    edge.vehicles.erase(vehicleID);

    vehicle.position = {
        toEdge->id,
        0};
    vehicle.speed = toEdge->calculateSpeed();

    Time Dt      = toEdge->length / vehicle.speed;
    Time tFuture = getTime() + Dt;

    // clang-format off
    env.eventQueue.push(shared_ptr<Event>(new EventComposite(
        tFuture,
        {
            make_shared<EventUpdateVehicle>(tFuture, vehicle.id),
            make_shared<EventPickConnection>(tFuture, vehicle.id)
        }
    )));
    // clang-format on
}
