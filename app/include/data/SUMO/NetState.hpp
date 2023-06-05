#pragma once

#include <map>
#include <string>

#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"

namespace SUMO {
class NetState {
   public:
    struct Timestep {
       public:
        struct Edge {
            struct Vehicle {
                typedef std::string ID;

                ID     id;
                Length pos;
                Speed  speed;
            };

            Network::Edge::ID id;

            std::vector<Vehicle> vehicles;

            Vehicle &addVehicle(Vehicle::ID id, Length pos, Speed speed);
        };

        Time              time;
        std::vector<Edge> edges;

        Edge &addEdge(Network::Edge::ID id);
    };

   private:
    std::vector<Timestep> timesteps;

   public:
    Timestep &addTimestep(Time time);

    void saveToFile(const std::string &filePath);
};
}  // namespace SUMO
