#pragma once

#include <functional>
#include <queue>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Static/Demand.hpp"

namespace Dynamic {

class SUMOAdapter;
class Environment;

class Demand {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Demand load(T arg1, args... arg2);
    };

    struct Vehicle {
        typedef VehicleID ID;

        ID     id;
        Time   emissionTime;
        EdgeID u, v;

       public:
        bool operator<(const Vehicle &veh) const;
        bool operator>(const Vehicle &veh) const;
    };

    // clang-format off
    typedef std::vector<
        Vehicle
    > Vehicles;
    // clang-format on

    Vehicle::ID nextID = 1;

   private:
    Vehicles vehicles;

   public:
    void        addVehicle(Vehicle::ID id, Time emissionTime, EdgeID u, EdgeID v);
    Vehicle::ID addVehicle(Time emissionTime, EdgeID u, EdgeID v);

    const Vehicles &getVehicles() const;

    // clang-format off
    class UniformLoader: public Demand::Loader<
        const Static::Demand &,
        const Environment &,
        const Dynamic::SUMOAdapter &
    > {
        // clang-format on

        double scale;
        Time   beginTime, endTime;

       public:
        UniformLoader(double scale, Time beginTime, Time endTime);

        Demand load(
            const Static::Demand       &staticDemand,
            const Environment          &env,
            const Dynamic::SUMOAdapter &sumoAdapter
        );
    };

    class PoissonLoader: public Demand::Loader<const Static::Demand &> {
        double scale;
        Time   beginTime, endTime;

       public:
        PoissonLoader(double scale, Time beginTime, Time endTime);

        Demand load(const Static::Demand &staticDemand);
    };
};

}  // namespace Dynamic
