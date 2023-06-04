#pragma once

#include <functional>
#include <queue>

#include "Dynamic/Dynamic.hpp"
#include "Static/Demand.hpp"

namespace Dynamic {
class Demand {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Demand load(T arg1, args... arg2);
    };

    struct Vehicle {
        typedef VehicleID ID;

        ID   id;
        Time emissionTime;
        EdgeID u, v;

       public:
        bool operator<(const Vehicle &veh) const;
        bool operator>(const Vehicle &veh) const;
    };

    // clang-format off
    typedef std::priority_queue<
        Vehicle,
        std::vector<Vehicle>,
        std::greater<Vehicle>
    > VehicleQueue;
    // clang-format on

    Vehicle::ID nextID = 1;

   private:
    VehicleQueue vehicles;

   public:
    void addVehicle(Vehicle::ID id, Time emissionTime, EdgeID u, EdgeID v);
    void addVehicle(Time emissionTime, EdgeID u, EdgeID v);

    VehicleQueue getVehicles() const;

    class UniformLoader: public Demand::Loader<const Static::Demand &> {
        double scale;
        Time   startTime, endTime;

       public:
        UniformLoader(double scale, Time startTime, Time endTime);

        Demand load(const Static::Demand &staticDemand);
    };

    class PoissonLoader: public Demand::Loader<const Static::Demand &> {
        double scale;
        Time   startTime, endTime;

       public:
        PoissonLoader(double scale, Time startTime, Time endTime);

        Demand load(const Static::Demand &staticDemand);
    };
};

}  // namespace Dynamic
