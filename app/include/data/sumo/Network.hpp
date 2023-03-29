#pragma once

#include <map>
#include <string>
#include <vector>

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

   private:
    std::unordered_map<Junction::ID, Junction> junctions;
    std::unordered_map<Edge::ID, Edge> edges;

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
    static std::string toString(const SUMO::Network::Edge::Function &s);
};
}  // namespace utils
