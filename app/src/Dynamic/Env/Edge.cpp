#include "Dynamic/Env/Edge.hpp"

#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Edge::Edge(
    ID       id_,
    Node     u_,
    Node     v_,
    Length   length_,
    Speed    maxSpeed_,
    Priority priority_,
    size_t   nLanes
):
    id(id_),
    u(u_),
    v(v_),
    length(length_),
    maxSpeed(maxSpeed_),
    priority(priority_) {
    for(Lane::Index i = 0; i < nLanes; i++)
        lanes.emplace_back(*this, i);
}

Edge::Edge(const Edge &e):
    Edge(
        e.id,
        e.u,
        e.v,
        e.length,
        e.maxSpeed,
        e.priority,
        e.lanes.size()
    ) {}

Edge::Edge() {}

Edge &Edge::operator=(const Edge &e) {
    id       = e.id;
    u        = e.u;
    v        = e.v;
    length   = e.length;
    maxSpeed = e.maxSpeed;

    size_t nLanes = e.lanes.size();
    lanes.clear();
    for(Lane::Index i = 0; i < nLanes; i++)
        lanes.emplace_back(*this, i);

    return *this;
}

Speed Edge::calculateSpeed() const {
    return maxSpeed;
}

list<reference_wrapper<const Connection>> Edge::getOutgoingConnections() const {
    list<reference_wrapper<const Connection>> ret;
    for(const Lane &lane: lanes) {
        for(Connection &connection: lane.getOutgoingConnections())
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections() {
    list<reference_wrapper<Connection>> ret;
    for(Lane &lane: lanes) {
        for(Connection &connection: lane.getOutgoingConnections())
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<const Connection>> Edge::getOutgoingConnections(const Edge &destinationEdge) const {
    list<reference_wrapper<const Connection>> ret;
    for(const Lane &lane: lanes) {
        for(Connection &connection: lane.getOutgoingConnections(destinationEdge))
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections(const Edge &destinationEdge) {
    list<reference_wrapper<Connection>> ret;
    for(Lane &lane: lanes) {
        for(Connection &connection: lane.getOutgoingConnections(destinationEdge))
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<Connection>> Edge::getIncomingConnections() {
    list<reference_wrapper<Connection>> ret;
    for(Lane &lane: lanes) {
        for(Connection &connection: lane.getIncomingConnections())
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<const Connection>> Edge::getIncomingConnections() const {
    list<reference_wrapper<const Connection>> ret;
    for(const Lane &lane: lanes) {
        for(Connection &connection: lane.getIncomingConnections())
            ret.push_back(connection);
    }
    return ret;
}

bool Edge::operator==(const Edge &e) const {
    return id == e.id;
}

bool Edge::operator!=(const Edge &e) const {
    return !(*this == e);
}

bool Edge::operator<(const Edge &e) const {
    return id < e.id;
}

Edge Edge::INVALID = {-1, NODE_INVALID, NODE_INVALID, 0, 0, -1000, 0};

size_t std::hash<Edge>::operator()(const Edge &edge) const {
    return hash<Edge::ID>()(edge.id);
}
