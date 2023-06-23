#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Position.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Action Intention;

EventTrySpawnVehicle::EventTrySpawnVehicle(Time t_, const Dynamic::Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventTrySpawnVehicle::process(Env &env) {
    Lane &initialLane = vehicle.pickInitialLane(env);

    // clang-format off
    Vehicle &envVehicle = env.addVehicle(
        vehicle,
        env.getTime(), 
        Position{initialLane, 0}, 
        initialLane.calculateSpeed()
    );
    // clang-format on

    Time Dt      = envVehicle.position.lane.edge.length / envVehicle.speed;
    Time tFuture = env.getTime() + Dt;

    // cerr
    //     << "Vehicle " << vehicle.id
    //     << " spawned at time " << env.getTime()
    //     << ", tFuture=" << tFuture
    //     << " because L=" << envVehicle.position.lane.edge.length
    //     << " and v=" << envVehicle.speed
    //     << endl;

    // cerr << "    EventTrySpawnVehicle: vehicle " << vehicle.id
    // << " at time " << env.t
    // << ", creating future events for time " << tFuture
    // << "=" << env.t << "+" << Dt
    // << endl;

    // clang-format off
    env.pushEvent(make_shared<EventUpdateVehicle>(
        tFuture,
        envVehicle
    ));
    // clang-format on
}
