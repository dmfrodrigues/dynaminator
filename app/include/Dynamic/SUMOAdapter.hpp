#pragma once

#include <unordered_map>
#include <utility>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "Static/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Dynamic {
class SUMOAdapter: private Static::SUMOAdapter {
   private:
    // clang-format off
    std::unordered_map<
        SUMO::TAZ::ID,
        std::pair<
            std::list<SUMO::TAZ::Source>,
            std::list<SUMO::TAZ::Sink>
        >
    > sumoTAZSourcesSinks;
    //  clang-format on
    Env::TrafficLight::ID nextTL = 1;

    std::unordered_map<SUMO::Network::TrafficLightLogic::ID, Env::TrafficLight::ID> sumoTLToTL;

   public:
    SUMOAdapter();
    SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter);

    Env::TAZ::ID addSumoTAZ(
        const SUMO::TAZ::ID                &a,
        const std::list<SUMO::TAZ::Source> &sources,
        const std::list<SUMO::TAZ::Sink>   &sinks
    );

    Env::TAZ::ID toTAZ(const SUMO::TAZ::ID &a) const;

    // clang-format off
    std::pair<
        Env::Edge::ID, std::pair<
            Env::Node,
            Env::Node
        >
    > addSumoEdge(const SUMO::Network::Edge::ID &a);
    // clang-format on
    Env::Edge::ID addEdge();
    bool          isEdge(const SUMO::Network::Edge::ID &a) const;

    Env::TrafficLight::ID addSumoTL(const SUMO::Network::TrafficLightLogic::ID &a);
    Env::TrafficLight::ID toTL(const SUMO::Network::TrafficLightLogic::ID &a) const;

    // clang-format off
    std::pair<
        std::list<SUMO::TAZ::Source>,
        std::list<SUMO::TAZ::Sink>
    > toTAZEdges(const SUMO::TAZ::ID &a) const;
    // clang-format on
    const std::pair<Env::Node, Env::Node> toTAZNode(const SUMO::TAZ::ID &a) const;
    const SUMO::TAZ::ID                   toSumoTAZ(const Env::Node &a) const;

    const Env::Edge::ID                   toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID         toSumoEdge(const Env::Edge::ID &a) const;
    const std::pair<Env::Node, Env::Node> toNodes(const SUMO::Network::Edge::ID &a) const;

    bool isSumoEdge(const Env::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;

    SUMO::Network::Edge::ID fromNodeToSumoEdge(const Env::Node &a) const;

    void clear();

    operator Static::SUMOAdapter &();

    void dump() const;
};
}  // namespace Dynamic
