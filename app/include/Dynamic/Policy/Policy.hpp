#pragma once

#include <memory>

#include "Dynamic/Dynamic.hpp"
#include "Log/ProgressLogger.hpp"

namespace Dynamic {
namespace Env {
class Env;
class Connection;
class TAZ;
class Lane;
class Action;
}  // namespace Env

class Vehicle;

class Action;

/**
 * @brief Vehicle path selection policy.
 *
 * A vehicle policy is the strategy used by a vehicle to select the next
 * connection to take.
 *
 * This sort of software design follows the Strategy pattern, but I decided
 * to use the term "policy" for its parallel with the concept of policy in
 * reinforcement learning, where the policy is expected to be always
 * changing and improving based on feedback.
 */
class Policy {
   private:
    typedef long VehicleID;

   public:
    /**
     * @brief Policy factory.
     *
     * Used by demand loaders to generate policies for new vehicles.
     */
    class Factory {
       public:
        /**
         * @brief Create a policy for a vehicle.
         *
         * @param id        Vehicle ID.
         * @param depart    Departure time.
         * @param from      Origin edge.
         * @param to        Destination edge.
         * @return std::shared_ptr<Policy>  Policy.
         */
        virtual std::shared_ptr<Policy> create(
            VehicleID       id,
            Time            depart,
            const Env::TAZ &fromTAZ,
            const Env::TAZ &toTAZ
        ) = 0;
    };

    /**
     * @brief Logger.
     *
     * Allows a policy or a policy factory to log information to a ProgressLogger.
     */
    class Logger {
       public:
        virtual void log(Log::ProgressLogger &logger);
    };

    /**
     * @brief Pick an initial lane.
     *
     * @param vehicle   Vehicle.
     * @param env       Environment.
     * @return const Env::Lane&     Picked lane.
     */
    virtual Env::Lane &pickInitialLane(
        Vehicle  &vehicle,
        Env::Env &env
    ) = 0;

    /**
     * @brief Pick a connection.
     *
     * @param env   Environment.
     * @return const Env::Connection&   Picked connection.
     */
    virtual std::shared_ptr<Env::Action> pickConnection(Env::Env &env) = 0;
};

}  // namespace Dynamic
