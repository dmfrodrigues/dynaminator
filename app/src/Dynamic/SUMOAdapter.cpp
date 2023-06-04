#include "Dynamic/SUMOAdapter.hpp"
#include "Static/SUMOAdapter.hpp"

using namespace std;
using namespace Dynamic;

SUMOAdapter::SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter) : Static::SUMOAdapter(staticSUMOAdapter) {}

// clang-format off
pair<
    Environment::Node,
    Environment::Node
> SUMOAdapter::addSumoTAZ(const SUMO::TAZ::ID &a) {
    // clang-format on
    auto [src, sink] = Static::SUMOAdapter::addSumoTAZ(a);
    return {Environment::Node(src), Environment::Node(sink)};
}

// clang-format off
pair<
    Environment::Edge::ID, pair<
        Environment::Node,
        Environment::Node
    >
> SUMOAdapter::addSumoEdge(
    const SUMO::Network::Edge::ID &a
) {
    // clang-format on
    auto [edge, nodes] = Static::SUMOAdapter::addSumoEdge(a);
    return {Environment::Edge::ID(edge), {Environment::Node(nodes.first), Environment::Node(nodes.second)}};
}

Environment::Edge::ID SUMOAdapter::addEdge() {
    return Environment::Edge::ID(Static::SUMOAdapter::addEdge());
}

bool SUMOAdapter::isEdge(const SUMO::Network::Edge::ID &a) const {
    return Static::SUMOAdapter::isEdge(a);
}

const pair<Environment::Node, Environment::Node> SUMOAdapter::toTAZNode(const SUMO::TAZ::ID &a) const {
    return {Environment::Node(Static::SUMOAdapter::toTAZNode(a).first), Environment::Node(Static::SUMOAdapter::toTAZNode(a).second)};
}

const SUMO::TAZ::ID SUMOAdapter::toSumoTAZ(const Environment::Node &a) const {
    return Static::SUMOAdapter::toSumoTAZ(a);
}

const Environment::Edge::ID SUMOAdapter::toEdge(const SUMO::Network::Edge::ID &a) const {
    return Environment::Edge::ID(Static::SUMOAdapter::toEdge(a));
}

const SUMO::Network::Edge::ID SUMOAdapter::toSumoEdge(const Environment::Edge::ID &a) const {
    return Static::SUMOAdapter::toSumoEdge(a);
}

const pair<Environment::Node, Environment::Node> SUMOAdapter::toNodes(const SUMO::Network::Edge::ID &a) const {
    return {
        Environment::Node(Static::SUMOAdapter::toNodes(a).first),
        Environment::Node(Static::SUMOAdapter::toNodes(a).second)
    };
}

bool SUMOAdapter::isSumoEdge(const Environment::Edge::ID &a) const {
    return Static::SUMOAdapter::isSumoEdge(a);
}

vector<SUMO::Network::Edge::ID> SUMOAdapter::getSumoEdges() const {
    return Static::SUMOAdapter::getSumoEdges();
}

SUMO::Network::Edge::ID SUMOAdapter::fromNodeToSumoEdge(const Environment::Node &a) const {
    return Static::SUMOAdapter::fromNodeToSumoEdge(a);
}

void SUMOAdapter::clear() {
    Static::SUMOAdapter::clear();
}
