#include "Dynamic/Env/Event/EventPickConnection.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventComposite.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

EventPickConnection::EventPickConnection(Time t_, Vehicle &vehicle_):
    Event(t_), vehicle(vehicle_) {}

void EventPickConnection::process(Env &env) const {
    const Connection &connection = vehicle.pickConnection(env);

    vehicle.move(env, connection);

    if(
        connection != Connection::LEAVE && connection != Connection::STOP
    ) {
        Time Dt      = connection.toLane.edge.length / vehicle.speed;
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
