#pragma once

#include <functional>
#include <list>
#include <map>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Node.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

class Env;

class Connection;

class Vehicle;

class Edge {
    friend Env;
    friend Vehicle;

   public:
    typedef long ID;

    struct Lane {
        typedef size_t Index;

        Edge::ID id;
        Index    index;
    };

    ID     id;
    Node   u, v;
    Length length;
    size_t nLanes;
    Speed  speed;

   private:
    std::map<Edge::ID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;

    std::set<VehicleID> vehicles;

   protected:
    Edge(
        ID     id,
        Node   u,
        Node   v,
        Length length,
        size_t nLanes,
        Speed  speed
    );

   public:
    virtual Speed calculateSpeed() const;

    std::list<std::reference_wrapper<Connection>> getOutgoingConnections() const;
    std::list<std::reference_wrapper<Connection>> getOutgoingConnections(const Edge &destinationEdge) const;

    bool operator==(const Edge &e) const;
    bool operator!=(const Edge &e) const;

    static const Edge INVALID;
};
}  // namespace Dynamic::Env
