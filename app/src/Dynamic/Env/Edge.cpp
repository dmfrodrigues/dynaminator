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
