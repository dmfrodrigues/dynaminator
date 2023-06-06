#pragma once

#include <map>
#include <string>

#include "Dynamic/Environment.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"

namespace SUMO {
class NetState {
   public:
    struct Timestep {
        template<typename T, typename... args>
        class Loader {
           public:
            Timestep load(T var1, args... var2);
        };

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

        Time                              time;
        std::map<Network::Edge::ID, Edge> edges;

        Edge &addEdge(Network::Edge::ID id);
    };

   private:
    std::vector<Timestep> timesteps;

   public:
    Timestep &addTimestep(Time time);
    Timestep &addTimestep(const Timestep &timestep);

    void saveToFile(const std::string &filePath);
};

// clang-format off
template<>
class NetState::Timestep::Loader<
    Dynamic::Environment &,
    const Dynamic::SUMOAdapter &,
    Dynamic::Time
> {
    // clang-format on
   public:
    Timestep load(
        Dynamic::Environment       &env,
        const Dynamic::SUMOAdapter &adapter,
        Dynamic::Time               t
    );
};

}  // namespace SUMO
