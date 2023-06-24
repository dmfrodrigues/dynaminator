#pragma once

#include <memory>
#include <string>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic {

namespace Env {
class Env;
class Lane;
class TAZ;
}  // namespace Env

class Policy;

class Vehicle {
   public:
    typedef long ID;

    const ID   id;               /// @brief Vehicle ID.
    const Time depart;           /// @brief Departure time.
    Env::TAZ  &fromTAZ, &toTAZ;  /// @brief Origin and destination TAZs.

   protected:
    std::shared_ptr<Policy> policy;  /// @brief Policy.

   public:
    Vehicle(
        ID                      id,
        Time                    depart,
        Env::TAZ               &fromTAZ,
        Env::TAZ               &toTAZ,
        std::shared_ptr<Policy> policy
    );

    Env::Lane &pickInitialLane(Env::Env &env);

    bool operator<(const Vehicle &other) const;
};
}  // namespace Dynamic
