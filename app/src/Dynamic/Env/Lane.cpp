#include "Dynamic/Env/Lane.hpp"

#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

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
    return edge.calculateSpeed();
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
    }
}

Lane Lane::INVALID = {Edge::INVALID, 0};

size_t std::hash<Lane>::operator()(const Lane &lane) const {
    size_t h = hash<Edge::ID>()(lane.edge.id) << 4;
    h ^= lane.index;
    return h;
}
