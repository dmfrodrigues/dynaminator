#pragma once

#include "data/SumoNetwork.hpp"

#include <list>

class SumoTAZs {
   public:
    struct TAZ {
        typedef std::string Id;
        typedef double Weight;

        struct Source {
            SumoNetwork::Junction::Id id;
            Weight weight;
        };
        struct Sink {
            SumoNetwork::Junction::Id id;
            Weight weight;
        };

        Id id;
        std::list<Source> sources;
        std::list<Sink> sinks;
    };

   private:
    std::unordered_map<TAZ::Id, TAZ> tazs;

   public:
    void addTAZ(TAZ taz);

    std::vector<TAZ> getTAZs() const;

    static SumoTAZs loadFromFile(const std::string &path);
};
