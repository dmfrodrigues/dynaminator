#include "Dynamic/Env/Lane.hpp"

#include "Dynamic/Env/Edge.hpp"

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

list<reference_wrapper<Connection>> Lane::getOutgoingConnections(
    const Edge &nextEdge
) const {
    return outgoingConnections.at(nextEdge.id);
}

Lane Lane::INVALID = {Edge::INVALID, 0};
