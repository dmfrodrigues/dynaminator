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

class Lane;

class Connection;

class Vehicle;

class Edge {
    friend Env;
    friend Vehicle;

   public:
    typedef long ID;

    ID     id;
    Node   u, v;
    Length length;
    Speed  speed;

    std::vector<std::shared_ptr<Lane>> lanes;

   private:
    std::map<Edge::ID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;

    std::set<VehicleID> vehicles;

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

    std::list<std::reference_wrapper<Connection>> getOutgoingConnections() const;
    std::list<std::reference_wrapper<Connection>> getOutgoingConnections(const Edge &destinationEdge) const;

    bool operator==(const Edge &e) const;
    bool operator!=(const Edge &e) const;

    static const Edge INVALID;
};
}  // namespace Dynamic::Env
