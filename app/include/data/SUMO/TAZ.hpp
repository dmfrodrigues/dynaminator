#pragma once

#include <list>

#include "data/SUMO/Network.hpp"

namespace SUMO {
struct TAZ {
    typedef std::string ID;
    typedef double Weight;

    struct Source {
        SUMO::Network::Edge::ID id;
        Weight weight;
    };
    struct Sink {
        SUMO::Network::Edge::ID id;
        Weight weight;
    };

    ID id;
    std::list<Source> sources;
    std::list<Sink> sinks;

    static std::unordered_map<TAZ::ID, TAZ> loadFromFile(const std::string &path);
};

typedef std::unordered_map<TAZ::ID, TAZ> TAZs;
}  // namespace SUMO
