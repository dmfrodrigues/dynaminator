#include "Dynamic/Env/Event/EventSpawnVehicle.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Position.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

EventSpawnVehicle::EventSpawnVehicle(
    Time                    t_,
    const Dynamic::Vehicle &vehicle_
):
    Event(t_),
    vehicle(vehicle_) {}

void EventSpawnVehicle::process(Env &env) {
    Lane &initialLane = vehicle.pickInitialLane(env);

    if(initialLane.isFull()) {
        initialLane.uninstantiated.push(vehicle);

        if(initialLane.uninstantiated.size() % 10 == 0) {
            // cerr
            //     << "[WARN][t=" << env.getTime() << "] "
            //     << "Uninstantiated queue of lane " << initialLane.idAsString()
            //     << " has size " << initialLane.uninstantiated.size()
            //     << endl;
        }

        return;
    }

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

    // clang-format off
    env.pushEvent(make_shared<EventUpdateVehicle>(
        tFuture,
        envVehicle
    ));
    // clang-format on
}
