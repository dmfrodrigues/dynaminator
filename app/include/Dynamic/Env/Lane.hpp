#pragma once

#include <cstddef>
#include <deque>
#include <list>
#include <map>

namespace Dynamic::Env {

class Env;
class Edge;
class Connection;
class Vehicle;

struct Lane {
    friend Env;

    typedef size_t Index;

    Edge &edge;
    Index index;

   private:
    typedef long EdgeID;

    std::map<EdgeID, std::list<std::reference_wrapper<Connection>>> outgoingConnections;

   public:
    std::deque<std::reference_wrapper<Vehicle>> queue;

    Lane(Edge &edge, Index index);

    bool operator==(const Lane &other) const;
    bool operator!=(const Lane &other) const;

    std::list<std::reference_wrapper<Connection>> getOutgoingConnections() const;
    std::list<std::reference_wrapper<Connection>> getOutgoingConnections(
        const Edge &nextEdge
    ) const;

    static Lane INVALID;
};
}  // namespace Dynamic::Env
