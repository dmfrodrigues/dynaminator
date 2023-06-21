#pragma once

#include <functional>
#include <list>
#include <map>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Node.hpp"

namespace Dynamic::Env {

class Env;

class Lane;

class Connection;

class Vehicle;

class Edge {
    friend Env;

   public:
    typedef long ID;

    ID     id;
    Node   u, v;
    Length length;
    Speed  speed;

    std::vector<Lane> lanes;

   protected:
    Edge(
        ID     id,
        Node   u,
        Node   v,
        Length length,
        Speed  speed,
        size_t nLanes
    );

   public:
    virtual Speed calculateSpeed() const;

    std::list<std::reference_wrapper<Connection>>       getOutgoingConnections();
    std::list<std::reference_wrapper<const Connection>> getOutgoingConnections() const;

    std::list<std::reference_wrapper<Connection>>       getOutgoingConnections(const Edge &destinationEdge);
    std::list<std::reference_wrapper<const Connection>> getOutgoingConnections(const Edge &destinationEdge) const;

    std::list<std::reference_wrapper<Connection>>       getIncomingConnections();
    std::list<std::reference_wrapper<const Connection>> getIncomingConnections() const;

    bool operator==(const Edge &e) const;
    bool operator!=(const Edge &e) const;
    bool operator<(const Edge &e) const;

    static Edge INVALID;
};
}  // namespace Dynamic::Env

namespace std {
template<>
struct hash<Dynamic::Env::Edge> {
    size_t operator()(const Dynamic::Env::Edge &e) const;
};
}  // namespace std
