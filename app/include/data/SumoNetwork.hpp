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
        typedef std::string Id;

        Id id;
        Coord pos;
    };

    struct Edge {
        typedef std::string Id;

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
            typedef std::string Id;
            typedef int Index;
            typedef double Speed;
            typedef double Length;

            Id id;
            Index index;
            Speed speed;
            Length length;
            Shape shape;
        };

        Id id;
        Junction::Id from;
        Junction::Id to;
        Priority priority = Edge::PRIORITY_UNSPECIFIED;
        Function function = NORMAL;
        Shape shape;
        std::map<Lane::Index, Lane> lanes;
    };

   private:
    std::unordered_map<Junction::Id, Junction> junctions;
    std::unordered_map<Edge::Id, Edge> edges;

   public:
    static SumoNetwork loadFromFile(const std::string &path);
};
