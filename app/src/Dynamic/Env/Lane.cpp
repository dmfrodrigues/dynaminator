#include "Dynamic/Env/Lane.hpp"

#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

const double Lane::JUNCTION_PERIOD = 1.0 / JUNCTION_CAPACITY;
const double Lane::QUEUE_SPEED     = JUNCTION_CAPACITY * Vehicle::LENGTH;
const Length Lane::K_JAM           = 1.0 / Dynamic::Env::Vehicle::LENGTH;

Lane::Lane(Edge &edge, Index index):
    edge(edge), index(index) {}

bool Lane::operator==(const Lane &other) const {
    return edge == other.edge && index == other.index;
}

bool Lane::operator!=(const Lane &other) const {
    return !(*this == other);
}

bool Lane::operator<(const Lane &other) const {
    if(edge != other.edge)
        return edge < other.edge;
    else
        return index < other.index;
}

list<reference_wrapper<Connection>> Lane::getOutgoingConnections() const {
    list<reference_wrapper<Connection>> ret;
    for(auto &[destinationEdgeID, connections]: outgoingConnections)
        for(Connection &connection: connections)
            ret.push_back(connection);
    return ret;
}

list<reference_wrapper<Connection>> Lane::getOutgoingConnections(
    const Edge &nextEdge
) const {
    if(!outgoingConnections.count(nextEdge.id))
        return list<reference_wrapper<Connection>>();
    return outgoingConnections.at(nextEdge.id);
}

list<reference_wrapper<Connection>> Lane::getIncomingConnections() const {
    list<reference_wrapper<Connection>> ret;
    for(auto &[sourceEdgeID, connections]: incomingConnections)
        for(Connection &connection: connections)
            ret.push_back(connection);
    return ret;
}

Speed Lane::calculateSpeed() const {
    Length L = max(queuePosition(), Dynamic::Env::Vehicle::LENGTH);

    Length K = (double)(moving.size() + stopped.size()) / edge.length;

    Speed v = edge.calculateSpeed() * (1.0 - K / K_JAM);

    v = max(v, QUEUE_SPEED);

    return v;
}

Length Lane::queueLength() const {
    return Vehicle::LENGTH * (Length)stopped.size();
}

Length Lane::queuePosition() const {
    return edge.length - queueLength();
}

bool Lane::isFull() const {
    return queueLength() >= edge.length;
}

void Lane::processNextWaitingVehicle(Env &env) {
    if(!uninstantiated.empty()) {
        Dynamic::Vehicle instantiatedVehicle = uninstantiated.front();
        uninstantiated.pop();

        EventTrySpawnVehicle event(
            env.getTime(),
            instantiatedVehicle
        );
        event.process(env);

        return;
    }

    vector<reference_wrapper<Lane>> incomingLanes;
    for(Connection &connection: getIncomingConnections()) {
        incomingLanes.push_back(connection.fromLane);
    }

    sort(
        incomingLanes.begin(),
        incomingLanes.end(),
        [](const Lane &a, const Lane &b) -> bool {
            return a.edge.priority > b.edge.priority;
        }
    );
    for(Lane &incomingLane: incomingLanes) {
        if(!incomingLane.stopped.empty()) {
            auto [vehicle, action] = incomingLane.stopped.front();

            if(action->connection.toLane == *this) {
                /*
                 * We don't need if connection.canPass(), because EventPopQueue
                 * already checks connection.canPass() before popping the queue.
                 */
                EventPopQueue event(
                    env.getTime(),
                    incomingLane
                );
            }

            return;
        }
    }

    /*
     * TODO: A vehicle is moving to a new lane. A vehicle can only move to the
     * new lane if !lane.isFull(). If lane.isFull(), the vehicle should be
     * enqueued at its current lane, and wait for the destination lane to pull
     * that vehicle.
     *
     * But for this logic to work, we also need to implement the logic to
     */
}

Lane Lane::INVALID = {Edge::INVALID, 0};

size_t std::hash<Lane>::operator()(const Lane &lane) const {
    size_t h = hash<Edge::ID>()(lane.edge.id) << 4;
    h ^= lane.index;
    return h;
}
