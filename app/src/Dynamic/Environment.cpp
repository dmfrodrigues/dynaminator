#include "Dynamic/Environment.hpp"

#include <cstdarg>
#include <memory>

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

Environment::Environment(Time startTime):
    t(startTime) {}

Environment::Edge &Environment::addEdge(Edge::ID id, Node u, Node v, Length length, size_t nLanes, Speed speed) {
    return edges.emplace(id, Edge(id, u, v, length, nLanes, speed)).first->second;
}

map<Environment::Edge::ID, Environment::Edge>       &Environment::getEdges() { return edges; }
map<Environment::Vehicle::ID, Environment::Vehicle> &Environment::getVehicles() { return vehicles; }

void Environment::updateAllVehicles(Time t_) {
    for(const auto &[_, vehicle]: vehicles)
        eventQueue.push(make_shared<EventUpdateVehicle>(t_, vehicle.id));
    while(eventQueue.top()->getTime() <= t_) {
        shared_ptr<Event> event = eventQueue.top();
        eventQueue.pop();
        event->process(*this);
    }
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
    Vehicle &vehicle = env.vehicles.at(vehicleID);

    Time Dt = getTime() - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;
}

/// === EventPickConnection ===================================================

Environment::EventPickConnection::EventPickConnection(Time t_, Vehicle::ID vehicleID_):
    Event(t_), vehicleID(vehicleID_) {}

void Environment::EventPickConnection::process(Environment &env) const {
    Vehicle &vehicle = env.vehicles.at(vehicleID);
    Edge    &edge    = env.edges.at(vehicle.position.edge);

    // Pick random connection
    list<Connection::ID> connections = edge.getOutgoingConnections();

    size_t i  = rand() % connections.size();
    auto   it = connections.begin();
    advance(it, i);
    Connection::ID connectionID = *it;

    Connection &connection = env.connections.at(connectionID);
    Edge       &toEdge     = env.edges.at(connection.toID);

    // Apply connection
    edge.vehicles.erase(vehicleID);

    vehicle.position = {
        toEdge.id,
        0};
    vehicle.speed = toEdge.calculateSpeed();

    Time Dt      = toEdge.length / vehicle.speed;
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
