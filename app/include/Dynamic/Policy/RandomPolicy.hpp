#pragma once

#include <random>

#include "Dynamic/Vehicle.hpp"

namespace Dynamic {

/**
 * @brief Policy that picks a random connection.
 *
 * This is a simple policy, used only for testing. It selects a random
 * connection from the set of allowed connections. If there is no possible
 * connection (e.g., if the vehicle is on a dead-end), it returns the `LEAVE`
 * connection to instruct the environment to remove the vehicle (as if the
 * vehicle had arrived at its destination).
 */
class RandomPolicy: public Vehicle::Policy {
   public:
    struct Action: public Vehicle::Policy::Action {
        Action(Env::Connection &connection, Env::Lane &lane);

        virtual void reward(Time t) override;
    };

   private:
    Vehicle::ID id;

    std::shared_ptr<std::mt19937> gen;

   public:
    RandomPolicy(Vehicle::ID id, std::shared_ptr<std::mt19937> gen);

    virtual Env::Lane &pickInitialLane(
        const Vehicle  &vehicle,
        const Env::Env &env
    ) override;

    virtual std::shared_ptr<Vehicle::Policy::Action> pickConnection(
        const Env::Env &env
    ) override;

    virtual void feedback(
        const Env::Edge &e,
        Time             t
    ) override;

    /**
     * @brief Factory for random policy.
     *
     * Instantiates RandomPolicy.
     */
    class Factory: public Vehicle::Policy::Factory {
        std::shared_ptr<std::mt19937> gen;

       public:
        Factory();
        Factory(std::random_device::result_type seed);
        virtual std::shared_ptr<Policy> create(
            Vehicle::ID      id,
            Time             depart,
            const Env::Edge &from,
            const Env::Edge &to
        ) override;
    };
};

}  // namespace Dynamic
