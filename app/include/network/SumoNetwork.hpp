#pragma once

#include <string>
#include <vector>

#include "geo/Coord.hpp"

class SumoNetwork {
   public:
    typedef std::vector<Coord> Shape;

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
        static Function charArrayToFunction(char *arr);

        // struct Lane {
        //    public:
        //     typedef std::string Id;
        //     typedef int Index;
        //     typedef double Speed;
        //     typedef double Length;

        //    private:
        //     Id id;
        //     Index index;
        //     Speed speed;
        //     Length length;
        //     Shape shape;
        // };

        Id id;
        Junction::Id from;
        Junction::Id to;
        Priority priority = Edge::PRIORITY_UNSPECIFIED;
        Function function = NORMAL;
        // std::vector<Lane> lanes;
    };

   private:
    std::unordered_map<Junction::Id, Junction> junctions;
    std::unordered_map<Edge::Id, Edge> edges;

   public:
    static SumoNetwork loadFromFile(const std::string &path);
};
