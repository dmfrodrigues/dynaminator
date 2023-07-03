#pragma once

#include "Dynamic/Demand.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Vehicle.hpp"

/**
 * @brief Try to spawn vehicle.
 *
 * This event instructs the environment to try to spawn a vehicle at time
 * `t`. The vehicle is spawned only if the edge has available space. If not,
 * then a new EventSpawnVehicle is scheduled for a later time at which
 * spawning should be retried.
 */
class Dynamic::Environment::EventSpawnVehicle: public Event {
    Vehicle vehicle;

   public:
    EventSpawnVehicle(Time t, const Vehicle &vehicle);

    virtual void process(Environment &env) const;
};

/**
 * @brief Update vehicle position and speed.
 *
 * This event takes the latest vehicle information, and applies the
 * respective movement equations to update the vehicle position and speed
 * according to the current time as indicated by the Environment.
 *
 * This step is uniquely linked to the environment model, since the movement
 * equations are part of the environment model.
 */
class Dynamic::Environment::EventUpdateVehicle: public Event {
    Vehicle::ID vehicleID;

   public:
    EventUpdateVehicle(Time t, Vehicle::ID vehicleID);

    virtual void process(Environment &env) const;
};

/**
 * @brief Force vehicle to pick connection in current edge.
 *
 * This event is triggered when a vehicle reaches the end of an edge/lane,
 * and must pick a connection to continue its path.
 *
 * This is where different vehicle routing policies can affect the final
 * result. When processing this event, a policy that is external to the
 * environment is applied.
 *
 * This event is equivalent to forcing the vehicle driver to choose what
 * they want to do. This is also the part where vehicle drivers can express
 * their objectives and path preferences.
 */
class Dynamic::Environment::EventPickConnection: public Event {
    Vehicle::ID vehicleID;

   public:
    EventPickConnection(Time t, Vehicle::ID vehicleID);

    virtual void process(Environment &env) const;
};

/**
 * @brief Log progress of the simulation.
 *
 * This event is especially useful to track simulation progress in real-time.
 */
class Dynamic::Environment::EventLog: public Event {
    Time                                           tStartSim, tEndSim;
    std::chrono::high_resolution_clock::time_point tStart;
    Log::ProgressLogger                           &logger;

   public:
    EventLog(
        Time                                           t,
        Time                                           tStartSim,
        Time                                           tEndSim,
        std::chrono::high_resolution_clock::time_point tStart,
        Log::ProgressLogger                           &logger
    );

    virtual void process(Environment &env) const;
};
