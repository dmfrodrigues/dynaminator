#pragma once

#include <map>
#include <string>
#include <vector>

#include "geo/Coord.hpp"

class SumoNetwork {
   public:
    typedef std::vector<Coord> Shape;

    static Shape stringToShape(const std::string &s);

    struct Junction {
        typedef std::string IdStr;
        typedef int Id; 

        Id id;
        IdStr idStr;
        Coord pos;

        static const Id INVALID = -1;
    };

    struct Edge {
        typedef std::string IdStr;
        typedef int Id;

        typedef int Priority;
        static const Priority PRIORITY_UNSPECIFIED = -1000;

        enum Function {
            NORMAL,
            INTERNAL,
            CONNECTOR,
            CROSSING,
            WALKINGAREA
        };
        static Function stringToFunction(const std::string &s);

        struct Lane {
           public:
            typedef std::string IdStr;
            typedef int Id;
            typedef int Index;
            typedef double Speed;
            typedef double Length;

            Id id;
            IdStr idStr;
            Index index;
            Speed speed;
            Length length;
            Shape shape;
        };

        Id id;
        IdStr idStr;
        Junction::Id from = -1;
        Junction::Id to = -1;
        Priority priority = Edge::PRIORITY_UNSPECIFIED;
        Function function = NORMAL;
        Shape shape;
        std::map<Lane::Index, Lane> lanes;
    };

   private:
    std::unordered_map<Junction::Id, Junction> junctions;
    std::unordered_map<Junction::IdStr, Junction::Id> junctionStr2Id;

    std::unordered_map<Edge::Id, Edge> edges;

   public:
    static SumoNetwork loadFromFile(const std::string &path);

    std::vector<Junction> getJunctions() const;
    std::vector<Edge> getEdges() const;
};
