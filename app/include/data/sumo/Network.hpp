#pragma once

#include <map>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#pragma GCC diagnostic pop

#include "data/sumo/SUMO.hpp"

namespace SUMO {
class Network {
   public:
    struct Junction {
        typedef std::string ID;

        ID id;
        Coord pos;

        static const ID INVALID;
    };

    struct Edge {
        typedef std::string ID;

        typedef int Priority;
        static const Priority PRIORITY_UNSPECIFIED = -1000;

        enum Function {
            NORMAL,
            INTERNAL,
            CONNECTOR,
            CROSSING,
            WALKINGAREA
        };

        struct Lane {
           public:
            typedef std::string ID;
            typedef double Speed;
            typedef double Length;

            ID id;
            Index index;
            Speed speed;
            Length length;
            Shape shape;
        };

        ID id;
        Junction::ID from = Junction::INVALID;
        Junction::ID to = Junction::INVALID;
        Priority priority = Edge::PRIORITY_UNSPECIFIED;
        Function function = NORMAL;
        Shape shape;
        std::map<Index, Lane> lanes;
    };

    struct TrafficLightLogic {
        typedef std::string ID;
        enum Type {
            STATIC,
            ACTUATED,
            DELAY_BASED
        };
        typedef std::string ProgramID;

        ID id;
        Type type;
        ProgramID programId;
        Time offset;

        struct Phase {
            enum State {
                RED,
                YELLOW_STOP,
                GREEN_NOPRIORITY,
                GREEN_PRIORITY,
                GREEN_RIGHT,
                YELLOW_START,
                OFF_YIELD,
                OFF
            };

            Time duration;
            std::vector<State> state;
        };

        std::map<Time, Phase> phases;
    };

   private:
    std::unordered_map<Junction::ID, Junction> junctions;
    std::unordered_map<Edge::ID, Edge> edges;
    std::unordered_map<TrafficLightLogic::ID, TrafficLightLogic> trafficLights;

    Junction loadJunction(const rapidxml::xml_node<> *it) const;
    Edge loadEdge(const rapidxml::xml_node<> *it) const;
    TrafficLightLogic loadTrafficLightLogic(const rapidxml::xml_node<> *it) const;

   public:
    static SUMO::Network loadFromFile(const std::string &path);

    std::vector<Junction> getJunctions() const;
    std::vector<Edge> getEdges() const;

    void saveStatsToFile(const std::string &path) const;
};
}  // namespace SUMO

namespace utils {
template <>
class stringifier<SUMO::Network::Edge::Function> {
   public:
    static SUMO::Network::Edge::Function fromString(const std::string &s);
    static std::string toString(const SUMO::Network::Edge::Function &t);
};

template <>
class stringifier<SUMO::Network::TrafficLightLogic::Type> {
   public:
    static SUMO::Network::TrafficLightLogic::Type fromString(const std::string &s);
    static std::string toString(const SUMO::Network::TrafficLightLogic::Type &t);
};

template <>
class stringifier<SUMO::Network::TrafficLightLogic::Phase::State> {
   public:
    static SUMO::Network::TrafficLightLogic::Phase::State fromString(const std::string &s);
    static std::string toString(const SUMO::Network::TrafficLightLogic::Phase::State &t);
};

template <>
class stringifier<std::vector<SUMO::Network::TrafficLightLogic::Phase::State>> {
   public:
    static std::vector<SUMO::Network::TrafficLightLogic::Phase::State> fromString(const std::string &s);
    static std::string toString(const std::vector<SUMO::Network::TrafficLightLogic::Phase::State> &t);
};
}  // namespace utils
