#pragma once

#include <map>
#include <string>

#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "utils/stringify.hpp"

namespace SUMO {
class EdgeData {
   public:
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
            friend Interval;

           public:
            Attributes attributes;

           private:
            Edge(SUMO::Network::Edge::ID id);
        };

        Attributes attributes;

       private:
        Interval(SUMO::Time begin, SUMO::Time end);
        Interval(SUMO::Time begin, SUMO::Time end, ID id);

        std::map<SUMO::Network::Edge::ID, Edge> edges;

       public:
        Edge &createEdge(SUMO::Network::Edge::ID id);
    };

   private:
    std::vector<Interval>          intervals;
    std::map<SUMO::ID, Interval *> idToInterval;

   public:
    Interval &createInterval(SUMO::Time begin, SUMO::Time end);
    Interval &createInterval(SUMO::Time begin, SUMO::Time end, SUMO::ID id);

    void saveToFile(const std::string &filename) const;
};
}  // namespace SUMO