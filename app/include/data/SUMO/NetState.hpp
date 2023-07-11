#pragma once

#include <ctpl_stl.h>

#include <fstream>
#include <future>
#include <map>
#include <string>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "utils/synchronizer.hpp"

namespace SUMO {
class NetState: private std::mutex {
    std::queue<std::future<std::stringstream>> futuresQueue;

    const size_t maxQueueSize = 8;

    ctpl::thread_pool pool = ctpl::thread_pool(1);

   public:
    struct Timestep {
        template<typename T, typename... args>
        class Loader {
           public:
            Timestep load(T var1, args... var2);
        };

        struct Edge {
            struct Lane {
                Network::Edge::Lane::ID id;

                struct Vehicle {
                    typedef std::string ID;

                    ID     id;
                    Length pos;
                    Speed  speed;
                };

                std::vector<Vehicle> vehicles;

                Vehicle &addVehicle(Vehicle::ID id, Length pos, Speed speed);

                Index index() const;
            };

            Network::Edge::ID id;

            std::map<Network::Edge::Lane::ID, Lane> lanes;

            Lane &addLane(Network::Edge::Lane::ID id);
        };

        Time                              time;
        std::map<Network::Edge::ID, Edge> edges;

        Edge &addEdge(Network::Edge::ID id);

        void            toXML(rapidxml::xml_document<> &doc) const;
        static Timestep fromXML(rapidxml::xml_node<> &node);
    };

    NetState &operator<<(const Timestep &ts);
    NetState &operator>>(Timestep &ts);

   private:
    std::ofstream os;

    std::ifstream        is;
    std::queue<Timestep> tsBuffer;

   public:
    NetState(const std::string &filePath, std::ios_base::openmode openMode);

    operator bool() const;

    void close();

    ~NetState();
};

// clang-format off
template<>
class NetState::Timestep::Loader<
    Dynamic::Env::Env &,
    const Dynamic::SUMOAdapter &,
    Dynamic::Time
> {
    // clang-format on
   public:
    Timestep load(
        Dynamic::Env::Env          &env,
        const Dynamic::SUMOAdapter &adapter,
        Dynamic::Time               t
    );
};

}  // namespace SUMO
