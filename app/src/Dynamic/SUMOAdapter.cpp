#include "Dynamic/SUMOAdapter.hpp"
#include "Static/SUMOAdapter.hpp"

using namespace std;
using namespace Dynamic;

SUMOAdapter::SUMOAdapter() : Static::SUMOAdapter() {}

SUMOAdapter::SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter) : Static::SUMOAdapter(staticSUMOAdapter) {}

// clang-format off
void SUMOAdapter::addSumoTAZ(
    const SUMO::TAZ::ID &a,
    const list<SUMO::TAZ::Source> &sources,
    const list<SUMO::TAZ::Sink> &sinks
) {
    // clang-format on
    Static::SUMOAdapter::addSumoTAZ(a);
    sumoTAZSourcesSinks[a] = {sources, sinks};
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

// clang-format off
std::pair<
    std::list<SUMO::TAZ::Source>,
    std::list<SUMO::TAZ::Sink>
> SUMOAdapter::toTAZEdges(const SUMO::TAZ::ID &a) const {
    // clang-format on
    const auto &[sources, sinks] = sumoTAZSourcesSinks.at(a);
    return {sources, sinks};
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

SUMOAdapter::operator Static::SUMOAdapter &() {
    return *this;
}
