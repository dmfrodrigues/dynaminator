#include "Dynamic/SUMOAdapter.hpp"

#include <string>

#include "Static/SUMOAdapter.hpp"

using namespace std;
using namespace Dynamic;

SUMOAdapter::SUMOAdapter():
    Static::SUMOAdapter() {}

SUMOAdapter::SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter):
    Static::SUMOAdapter(staticSUMOAdapter) {}

// clang-format off
Env::TAZ::ID SUMOAdapter::addSumoTAZ(
    const SUMO::TAZ::ID &a,
    const list<SUMO::TAZ::Source> &sources,
    const list<SUMO::TAZ::Sink> &sinks
) {
    // clang-format on
    Static::SUMOAdapter::addSumoTAZ(a);
    sumoTAZSourcesSinks[a] = {sources, sinks};
    return stoi(a);
}

Env::TAZ::ID SUMOAdapter::toTAZ(const SUMO::TAZ::ID &a) const {
    return stoi(a);
}

// clang-format off
pair<
    Env::Edge::ID, pair<
        Env::Node,
        Env::Node
    >
> SUMOAdapter::addSumoEdge(
    const SUMO::Network::Edge::ID &a
) {
    // clang-format on
    auto [edge, nodes] = Static::SUMOAdapter::addSumoEdge(a);
    return {Env::Edge::ID(edge), {Env::Node(nodes.first), Env::Node(nodes.second)}};
}

Env::Edge::ID SUMOAdapter::addEdge() {
    return Env::Edge::ID(Static::SUMOAdapter::addEdge());
}

bool SUMOAdapter::isEdge(const SUMO::Network::Edge::ID &a) const {
    return Static::SUMOAdapter::isEdge(a);
}

Env::TrafficLight::ID SUMOAdapter::addSumoTL(const SUMO::Network::TrafficLightLogic::ID &a) {
    return sumoTLToTL[a] = nextTL++;
}

Env::TrafficLight::ID SUMOAdapter::toTL(const SUMO::Network::TrafficLightLogic::ID &a) const {
    return sumoTLToTL.at(a);
}

const pair<Env::Node, Env::Node> SUMOAdapter::toTAZNode(const SUMO::TAZ::ID &a) const {
    return {Env::Node(Static::SUMOAdapter::toTAZNode(a).first), Env::Node(Static::SUMOAdapter::toTAZNode(a).second)};
}

// clang-format off
pair<
    list<SUMO::TAZ::Source>,
    list<SUMO::TAZ::Sink>
> SUMOAdapter::toTAZEdges(const SUMO::TAZ::ID &a) const {
    // clang-format on
    const auto &[sources, sinks] = sumoTAZSourcesSinks.at(a);
    return {sources, sinks};
}

const SUMO::TAZ::ID SUMOAdapter::toSumoTAZ(const Env::Node &a) const {
    return Static::SUMOAdapter::toSumoTAZ(a);
}

const Env::Edge::ID SUMOAdapter::toEdge(const SUMO::Network::Edge::ID &a) const {
    return Env::Edge::ID(Static::SUMOAdapter::toEdge(a));
}

const SUMO::Network::Edge::ID SUMOAdapter::toSumoEdge(const Env::Edge::ID &a) const {
    return Static::SUMOAdapter::toSumoEdge(a);
}

const pair<Env::Node, Env::Node> SUMOAdapter::toNodes(const SUMO::Network::Edge::ID &a) const {
    // clang-format off
    return {
        Env::Node(Static::SUMOAdapter::toNodes(a).first),
        Env::Node(Static::SUMOAdapter::toNodes(a).second)
    };
    // clang-format on
}

bool SUMOAdapter::isSumoEdge(const Env::Edge::ID &a) const {
    return Static::SUMOAdapter::isSumoEdge(a);
}

vector<SUMO::Network::Edge::ID> SUMOAdapter::getSumoEdges() const {
    return Static::SUMOAdapter::getSumoEdges();
}

SUMO::Network::Edge::ID SUMOAdapter::fromNodeToSumoEdge(const Env::Node &a) const {
    return Static::SUMOAdapter::fromNodeToSumoEdge(a);
}

void SUMOAdapter::clear() {
    Static::SUMOAdapter::clear();
}

SUMOAdapter::operator Static::SUMOAdapter &() {
    return *this;
}

void SUMOAdapter::dump() const {
    Static::SUMOAdapter::dump();
}
