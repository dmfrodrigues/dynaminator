#include "Dynamic/Env/Event/EventPickConnection.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Intention Intention;

EventPickConnection::EventPickConnection(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventPickConnection::process(Env &env) const {
    Intention intention = vehicle.pickConnection(env);

    vehicle.move(env, intention);

    if(
        intention.connection != Connection::LEAVE && intention.connection != Connection::STOP
    ) {
        Time Dt      = intention.connection.toLane.edge.length / vehicle.speed;
        Time tFuture = env.getTime() + Dt;

        // clang-format off
        env.pushEvent(make_shared<EventComposite>(
            tFuture,
            initializer_list<shared_ptr<Event>>{
                make_shared<EventUpdateVehicle>(tFuture, vehicle),
                make_shared<EventPickConnection>(tFuture, vehicle)
            }
        ));
        // clang-format on
    }
}
