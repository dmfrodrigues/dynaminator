#pragma once

#include <memory>
#include <string>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic {

namespace Env {
class Env;
class Edge;
class Lane;
class Connection;
class TAZ;
}  // namespace Env

class Vehicle {
   public:
    typedef long ID;

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
       public:
        typedef Time Reward;

        struct Action {
            Env::Connection &connection;
            Env::Lane       &lane;

            Action();
            Action(Env::Connection &connection, Env::Lane &lane);

            virtual void reward(Reward r) = 0;

            bool operator<(const Action &other) const;
        };

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
                Vehicle::ID      id,
                Time             depart,
                const Env::Edge &from,
                const Env::Edge &to
            ) = 0;
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
        virtual std::shared_ptr<Action> pickConnection(Env::Env &env) = 0;
    };

    const ID   id;               /// @brief Vehicle ID.
    const Time depart;           /// @brief Departure time.
    Env::TAZ  &fromTAZ, &toTAZ;  /// @brief Origin and destination TAZs.
    Env::Edge &from, &to;        /// @brief Origin and destination edges.

   protected:
    std::shared_ptr<Policy> policy;  /// @brief Policy.

   public:
    Vehicle(
        ID                      id,
        Time                    depart,
        Env::TAZ               &fromTAZ,
        Env::TAZ               &toTAZ,
        Env::Edge              &from,
        Env::Edge              &to,
        std::shared_ptr<Policy> policy
    );

    Env::Lane &pickInitialLane(Env::Env &env);

    bool operator<(const Vehicle &other) const;
};
}  // namespace Dynamic
