#pragma once

#include <functional>
#include <queue>
#include <set>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Vehicle.hpp"
#include "Static/Demand.hpp"

namespace Dynamic {

class SUMOAdapter;

class Demand {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Demand load(T arg1, args... arg2);
    };

    Vehicle::ID nextID = 1;

   private:
    std::vector<Vehicle> vehicles;

   public:
    Vehicle &addVehicle(
        Vehicle::ID                      id,
        Time                             depart,
        Env::TAZ                        &fromTAZ,
        Env::TAZ                        &toTAZ,
        std::shared_ptr<Vehicle::Policy> policy
    );
    Vehicle &addVehicle(
        Time                             depart,
        Env::TAZ                        &fromTAZ,
        Env::TAZ                        &toTAZ,
        std::shared_ptr<Vehicle::Policy> policy
    );

    const std::vector<Vehicle> &getVehicles() const;
};

}  // namespace Dynamic
