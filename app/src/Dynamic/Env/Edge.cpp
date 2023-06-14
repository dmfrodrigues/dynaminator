#include "Dynamic/Env/Edge.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Edge::Edge(ID id_, Node u_, Node v_, Length length_, size_t nLanes_, Speed speed_):
    id(id_), u(u_), v(v_), length(length_), nLanes(nLanes_), speed(speed_) {}

Speed Edge::calculateSpeed() const {
    return 50.0 / 3.6;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections() const {
    list<reference_wrapper<Connection>> ret;
    for(const auto &[_, conns]: outgoingConnections)
        ret.insert(ret.end(), conns.begin(), conns.end());
    return ret;
}

list<reference_wrapper<Connection>> Edge::getOutgoingConnections(const Edge &destinationEdge) const {
    if(outgoingConnections.count(destinationEdge.id))
        return outgoingConnections.at(destinationEdge.id);
    else
        return {};
}

bool Edge::operator==(const Edge &e) const {
    return id == e.id;
}

bool Edge::operator!=(const Edge &e) const {
    return !(*this == e);
}

const Edge Edge::INVALID = {-1, NODE_INVALID, NODE_INVALID, 0, 0, 0};
