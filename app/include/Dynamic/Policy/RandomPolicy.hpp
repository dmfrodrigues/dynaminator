#pragma once

#include <random>

#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Policy/Policy.hpp"
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
class RandomPolicy: public Policy {
   public:
    struct Action: public Env::Action {
        Action(Env::Connection &connection, Env::Lane &lane);

        virtual void reward(Reward r) override;
    };

   private:
    Vehicle::ID id;

    std::shared_ptr<std::mt19937> gen;

   public:
    RandomPolicy(Vehicle::ID id, std::shared_ptr<std::mt19937> gen);

    virtual Env::Lane &pickInitialLane(
        Vehicle  &vehicle,
        Env::Env &env
    ) override;

    virtual std::shared_ptr<Env::Action> pickConnection(
        Env::Env &env
    ) override;

    /**
     * @brief Factory for random policy.
     *
     * Instantiates RandomPolicy.
     */
    class Factory: public Policy::Factory {
        std::shared_ptr<std::mt19937> gen;

       public:
        Factory();
        Factory(std::random_device::result_type seed);
        virtual std::shared_ptr<Policy> create(
            Vehicle::ID     id,
            Time            depart,
            const Env::TAZ &fromTAZ,
            const Env::TAZ &toTAZ
        ) override;
    };
};

}  // namespace Dynamic
