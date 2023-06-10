#include "Dynamic/Environment.hpp"

#include <chrono>
#include <cstdarg>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Alg/Graph.hpp"
#include "Dynamic/Dynamic.hpp"
#include "Log/ProgressLogger.hpp"

using namespace std;
using namespace Dynamic;

typedef chrono::high_resolution_clock hrc;

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

bool Environment::Connection::operator==(const Connection &connection) const {
    return id == connection.id;
}

const Environment::Connection Environment::Connection::STOP = {-1};
const Environment::Connection Environment::Connection::LEAVE = {-2};

Environment::Event::Event(Time t_):
    t(t_) {}

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
        Time                     t = edge.length / edge.calculateSpeed();
        Alg::Graph::Edge::Weight w = t;
        G.addEdge(edge.id, edge.u, edge.v, w);
    }

    for(const auto &[_, connection]: connections) {
        Alg::Graph::Edge::Weight w = 0;

        Node fromEdgeV = edges.at(connection.fromID).v;
        Node toEdgeU   = edges.at(connection.toID).u;

        G.addEdge(connection.id + 1000000, fromEdgeV, toEdgeU, w);
    }

    return G;
}

Environment::Edge &Environment::addEdge(Edge::ID id, Node u, Node v, Length length, size_t nLanes, Speed speed) {
    return edges.emplace(id, Edge(id, u, v, length, nLanes, speed)).first->second;
}

void Environment::addDemand(const Demand &demand) {
    Demand::Vehicles vehicles = demand.getVehicles();
    for(const Demand::Vehicle &vehicle: vehicles) {
        eventQueue.push(make_shared<EventTrySpawnVehicle>(
            vehicle.emissionTime,
            vehicle
        ));
    }
}

const map<Environment::Edge::ID, Environment::Edge>       &Environment::getEdges() const { return edges; }
const map<Environment::Vehicle::ID, Environment::Vehicle> &Environment::getVehicles() const { return vehicles; }

void Environment::runUntil(Time tEnd) {
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

void Environment::updateAllVehicles(Time t_) {
    runUntil(t_);
    for(const auto &[_, vehicle]: vehicles)
        eventQueue.push(make_shared<EventUpdateVehicle>(t_, vehicle.id));
    runUntil(t_);
}

void Environment::log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta) {
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
    try {
        // clang-format off
        Vehicle &envVehicle = env.vehicles.emplace(vehicle.id, Vehicle{
            vehicle.id, 
            env.t, 
            Position{vehicle.u, 0}, 
            env.edges.at(vehicle.u).calculateSpeed()
        }).first->second;
        // clang-format on

        env.edges.at(envVehicle.position.edge).vehicles.insert(envVehicle.id);

        Time Dt      = env.edges.at(envVehicle.position.edge).length / envVehicle.speed;
        Time tFuture = env.t + Dt;

        // cerr << "    EventTrySpawnVehicle: vehicle " << vehicle.id
        // << " at time " << env.t
        // << ", creating future events for time " << tFuture
        // << "=" << env.t << "+" << Dt
        // << endl;

        // clang-format off
        env.eventQueue.push(make_shared<EventComposite>(
            tFuture,
            initializer_list<shared_ptr<Event>>{
                make_shared<EventUpdateVehicle>(tFuture, envVehicle.id),
                make_shared<EventPickConnection>(tFuture, envVehicle.id)
            }
        ));
        // clang-format on
    } catch(const out_of_range &e) {
        throw out_of_range("Environment::EventTrySpawnVehicle: edge " + to_string(vehicle.u) + " not found");
    }
}

/// === EventUpdateVehicle ====================================================

Environment::EventUpdateVehicle::EventUpdateVehicle(Time t_, Vehicle::ID vehicleID_):
    Event(t_), vehicleID(vehicleID_) {}

void Environment::EventUpdateVehicle::process(Environment &env) const {
    Vehicle &vehicle = env.vehicles.at(vehicleID);

    Time Dt = env.t - vehicle.lastUpdateTime;
    vehicle.position.offset += vehicle.speed * Dt;

    vehicle.lastUpdateTime = env.t;
}

/// === EventPickConnection ===================================================

Environment::EventPickConnection::EventPickConnection(Time t_, Vehicle::ID vehicleID_):
    Event(t_), vehicleID(vehicleID_) {}

void Environment::EventPickConnection::process(Environment &env) const {
    // cerr << "    EventPickConnection: vehicle " << vehicleID << " at time " << env.t << endl;

    Vehicle &vehicle = env.vehicles.at(vehicleID);
    Edge    &edge    = env.edges.at(vehicle.position.edge);

    // Pick random connection
    list<Connection::ID> connections = edge.getOutgoingConnections();

    if(connections.empty()) {
        // cerr << "Warning: "
        //      << "Vehicle " << vehicleID
        //      << " reached a dead end at edge " << edge.id
        //      << "; sending vehicle to beginning of same edge."
        //      << endl;

        return;
    } else {
        size_t i  = rand() % connections.size();
        auto   it = connections.begin();
        advance(it, i);
        Connection::ID connectionID = *it;
        Connection    &connection   = env.connections.at(connectionID);
        Edge          &toEdge       = env.edges.at(connection.toID);

        // Apply connection
        edge.vehicles.erase(vehicleID);

        // clang-format off
        vehicle.position = {
            toEdge.id,
            0
        };
        // clang-format on

        vehicle.speed = toEdge.calculateSpeed();

        Time Dt      = toEdge.length / vehicle.speed;
        Time tFuture = env.t + Dt;

        // clang-format off
        env.eventQueue.push(make_shared<EventComposite>(
            tFuture,
            initializer_list<shared_ptr<Event>>{
                make_shared<EventUpdateVehicle>(tFuture, vehicle.id),
                make_shared<EventPickConnection>(tFuture, vehicle.id)
            }
        ));
        // clang-format on
    }
}

/// === EventLog ===============================================================
Environment::EventLog::EventLog(Time t_, Time tStartSim_, Time tEndSim_, hrc::time_point tStart_, Log::ProgressLogger &logger_):
    Event(t_),
    tStartSim(tStartSim_),
    tEndSim(tEndSim_),
    tStart(tStart_),
    logger(logger_) {}

void Environment::EventLog::process(Environment &env) const {
    const hrc::time_point now = hrc::now();

    double elapsed = (double)chrono::duration_cast<chrono::nanoseconds>(now - tStart).count() * 1e-9;

    double progress = (env.t - tStartSim) / (tEndSim - tStartSim);

    double eta = (progress <= 0.0 ? 1.0 : elapsed * (1.0 - progress) / progress);

    logger << Log::ProgressLogger::Elapsed(elapsed)
           << Log::ProgressLogger::Progress(progress)
           << Log::ProgressLogger::ETA(eta)
           << Log::ProgressLogger::StartText()
           << env.t + 10
           << "\t" << env.vehicles.size()
           << "\t" << env.eventQueue.size()
           << Log::ProgressLogger::EndMessage();
}
