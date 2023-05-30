#pragma once

#include <functional>
#include <queue>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Network.hpp"

namespace Dynamic {
class Demand {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Demand load(T arg1, args... arg2);
    };

    struct Vehicle {
        Time          emissionTime;
        Network::Node u, v;

       public:
        bool operator<(const Vehicle &veh) const;
        bool operator>(const Vehicle &veh) const;
    };

    typedef std::priority_queue<
        Vehicle,
        std::vector<Vehicle>,
        std::greater<Vehicle>
    > VehicleQueue;

   private:
    VehicleQueue vehicles;

   public:
    void addVehicle(Time emissionTime, Network::Node u, Network::Node v);

    VehicleQueue getVehicles() const;
};
}  // namespace Dynamic
