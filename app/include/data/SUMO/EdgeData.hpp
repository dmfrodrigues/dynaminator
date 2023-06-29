#pragma once

#include <map>
#include <string>

#include "Static/SUMOAdapter.hpp"
#include "Static/Solution.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/SUMO.hpp"
#include "utils/stringify.hpp"

namespace SUMO {
class EdgeData {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        EdgeData load(T var1, args... var2);
    };

    class Attributes {
        friend EdgeData;

        std::map<std::string, std::string> attributes;

       public:
        template<typename T>
        void setAttribute(const std::string &name, const T &value) {
            attributes[name] = utils::stringify::stringify<T>::toString(value);
        }

        template<typename T>
        T getAttribute(const std::string &name) const {
            return utils::stringify::stringify<T>::fromString(attributes.at(name));
        }
    };
    class Interval {
        friend EdgeData;

       public:
        typedef SUMO::ID ID;

        class Edge {
            friend EdgeData;

           public:
            class Lane {
                friend EdgeData;

               public:
                Attributes attributes;

               private:
                Lane(SUMO::Network::Edge::Lane::ID id);
            };

            Attributes attributes;

           private:
            Edge(SUMO::Network::Edge::ID id);

            std::map<SUMO::Network::Edge::Lane::ID, Lane> lanes;

           public:
            bool  hasLane(SUMO::Network::Edge::Lane::ID id) const;
            Lane &createLane(SUMO::Network::Edge::Lane::ID id);
            Lane &getLane(SUMO::Network::Edge::Lane::ID id);
        };

        Attributes attributes;

       private:
        Interval(SUMO::Time begin, SUMO::Time end);
        Interval(SUMO::Time begin, SUMO::Time end, ID id);

        std::map<SUMO::Network::Edge::ID, Edge> edges;

       public:
        bool  hasEdge(SUMO::Network::Edge::ID id) const;
        Edge &createEdge(SUMO::Network::Edge::ID id);
        Edge &getEdge(SUMO::Network::Edge::ID id);
    };

   private:
    std::vector<Interval>          intervals;
    std::map<SUMO::ID, Interval *> idToInterval;

   public:
    Interval &createInterval(SUMO::Time begin, SUMO::Time end);
    Interval &createInterval(SUMO::Time begin, SUMO::Time end, SUMO::ID id);

    void saveToFile(const std::string &filename) const;
};

// clang-format off
template<>
class EdgeData::Loader<
    const NetworkTAZs&,
    const Static::BPRNetwork&,
    const Static::Solution&,
    const Static::SUMOAdapter&
> {
   public:
    EdgeData load(
        const NetworkTAZs &sumo,
        const Static::BPRNetwork &network,
        const Static::Solution &x,
        const Static::SUMOAdapter &adapter
    );
};
// clang-format on

}  // namespace SUMO
