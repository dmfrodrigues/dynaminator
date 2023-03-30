#pragma once

#include "data/sumo/Network.hpp"

#include <list>

class SumoTAZs {
   public:
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
    };

   private:
    std::unordered_map<TAZ::ID, TAZ> tazs;

   public:
    void addTAZ(TAZ taz);

    std::vector<TAZ> getTAZs() const;

    static SumoTAZs loadFromFile(const std::string &path);
};