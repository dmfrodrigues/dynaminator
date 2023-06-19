#include "Dynamic/Env/Edge.hpp"

#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Edge::Edge(ID id_, Node u_, Node v_, Length length_, Speed speed_, size_t nLanes):
    id(id_), u(u_), v(v_), length(length_), speed(speed_) {
    for(Lane::Index i = 0; i < nLanes; i++)
        lanes.emplace_back(make_shared<Lane>(*this, i));
}

Speed Edge::calculateSpeed() const {
    return 50.0 / 3.6;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections() const {
    list<reference_wrapper<Connection>> ret;
    for(const auto &lanePtr: lanes) {
        const Lane &lane = *lanePtr;
        for(Connection &connection: lane.getOutgoingConnections())
            ret.push_back(connection);
    }
    return ret;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections(const Edge &destinationEdge) const {
    list<reference_wrapper<Connection>> ret;
    for(const auto &lanePtr: lanes) {
        const Lane &lane = *lanePtr;
        for(Connection &connection: lane.getOutgoingConnections(destinationEdge))
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

Edge Edge::INVALID = {-1, NODE_INVALID, NODE_INVALID, 0, 0, 0};
