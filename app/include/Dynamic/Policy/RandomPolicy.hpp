#pragma once

#include <random>

#include "Dynamic/Vehicle.hpp"

namespace Dynamic {

class RandomPolicy: public Vehicle::Policy {
    Vehicle::ID id;

    std::shared_ptr<std::mt19937> gen;

   public:
    RandomPolicy(Vehicle::ID id, std::shared_ptr<std::mt19937> gen);

    virtual const Env::Connection &pickConnection(
        const Env::Env &env
    ) override;

    class Factory: public Vehicle::Policy::Factory {
        std::shared_ptr<std::mt19937> gen;

       public:
        Factory();
        Factory(std::random_device::result_type seed);
        virtual std::shared_ptr<Policy> create(
            Vehicle::ID id
        ) override;
    };
};

}  // namespace Dynamic
