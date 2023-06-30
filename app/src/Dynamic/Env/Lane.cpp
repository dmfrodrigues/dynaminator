#include "Dynamic/Env/Lane.hpp"

#include <optional>

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
    for(auto &[toEdgeID, connections]: outgoingConnections)
        for(auto &[toLaneIndex, connection]: connections)
            ret.push_back(connection);
    return ret;
}

list<reference_wrapper<Connection>> Lane::getOutgoingConnections(
    const Edge &nextEdge
) const {
    if(!outgoingConnections.count(nextEdge.id))
        return list<reference_wrapper<Connection>>();
    list<reference_wrapper<Connection>> ret;
    for(auto &[targetLaneIndex, connection]: outgoingConnections.at(nextEdge.id))
        ret.push_back(connection);
    return ret;
}

Connection &Lane::getOutgoingConnection(const Lane &nextLane) const {
    return outgoingConnections.at(nextLane.edge.id).at(nextLane.index);
}

list<reference_wrapper<Connection>> Lane::getIncomingConnections() const {
    list<reference_wrapper<Connection>> ret;
    for(auto &[fromEdgeID, connections]: incomingConnections)
        for(auto &[fromLaneIndex, connection]: connections)
            ret.push_back(connection);
    return ret;
}

Speed Lane::calculateSpeed() const {
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

    set<reference_wrapper<Connection>, less<Connection>> incomingConnections;
    for(Connection &connection: getIncomingConnections()) {
        const auto &incomingQueue = connection.fromLane.stopped;

        if(incomingQueue.empty()) continue;

        const Action &a = *incomingQueue.front().second;

        if(a.connection.toLane != *this) continue;

        incomingConnections.insert(connection);
    }

    optional<reference_wrapper<Connection>> connection = nullopt;

    while(!incomingConnections.empty() && !connection.has_value()) {
        connection = *incomingConnections.begin();

        for(
            auto it = ++incomingConnections.begin();
            it != incomingConnections.end();
            ++it
        ) {
            Connection &otherConnection = it->get();
            if(connection.value().get().yieldsTo(otherConnection)) {
                incomingConnections.erase(connection.value());
                connection = nullopt;
                break;
            }
        }
    }

    if(!connection.has_value())
        return;

    Lane &incomingLane = connection.value().get().fromLane;

    auto [vehicle, action] = incomingLane.stopped.front();

    assert(action->connection.toLane == *this);

    /*
     * We don't need to check if connection.canPass(), because
     * EventPopQueue already checks connection.canPass() before
     * popping the queue.
     */
    EventPopQueue event(
        env.getTime(),
        incomingLane
    );
    event.process(env);
}

Lane Lane::INVALID = {Edge::INVALID, 0};

size_t std::hash<Lane>::operator()(const Lane &lane) const {
    size_t h = hash<Edge::ID>()(lane.edge.id) << 4;
    h ^= lane.index;
    return h;
}
