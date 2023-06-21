#include "Dynamic/Env/Vehicle.hpp"

#include <stdexcept>

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic::Env;

typedef Dynamic::Vehicle::Policy::Action Intention;

Vehicle::Policy::Action::Action(
    Connection &connection_,
    Lane       &lane_
):
    connection(connection_), lane(lane_) {}

Vehicle::Policy::Action::Action():
    connection(Env::Connection::LEAVE), lane(Env::Lane::INVALID) {}

bool Vehicle::Policy::Action::operator<(const Action &other) const {
    if(connection != other.connection)
        return connection < other.connection;
    else
        return lane < other.lane;
}

Vehicle::Vehicle(
    const Dynamic::Vehicle &vehicle,
    Time                    t,
    Position                position_,
    Speed                   speed_,
    State                   state_
):
    Dynamic::Vehicle(vehicle),
    lastUpdateTime(t),
    position(position_),
    speed(speed_),
    state(state_) {}

shared_ptr<Vehicle::Policy::Action> Vehicle::pickConnection(Env &env) const {
    return policy->pickConnection(env);
}

bool Vehicle::move(Env &env, shared_ptr<Vehicle::Policy::Action> &action) {
    static int           LEAVE_BAD  = 0;
    static int           LEAVE_GOOD = 0;
    static map<int, int> LEAVE_EDGES;
    if(action->connection == Connection::LEAVE) {
        if(position.lane.edge != to) {  // Leaving network at wrong place
            // cerr << "Vehicle::move: Warning: vehicle " << id << " is leaving network at wrong place" << endl;
            LEAVE_EDGES[position.lane.edge.id]++;
            ++LEAVE_BAD;
            action->reward(-1.0e9);
        } else {
            LEAVE_GOOD++;
        }

        if((LEAVE_GOOD + LEAVE_BAD) % 1000 == 0) {
            cout << "LEAVE_GOOD = " << LEAVE_GOOD << ", LEAVE_BAD = " << LEAVE_BAD << ", ratio of good is " << (double)LEAVE_GOOD / (LEAVE_GOOD + LEAVE_BAD) << endl;
            cerr << "LEAVES: " << endl;
            vector<pair<int, int>> LEAVE_ORDERED;
            for(const auto [a, b]: LEAVE_EDGES)
                LEAVE_ORDERED.emplace_back(b, a);
            sort(LEAVE_ORDERED.rbegin(), LEAVE_ORDERED.rend());
            for(const auto [a, b]: LEAVE_ORDERED) {
                if(a < 50) break;
                cerr << b << ": " << a << endl;
            }
        }

        env.removeVehicle(id);
        return false;
    } else if(action->connection == Connection::STOP) {
        return false;
    }

    if(position.lane != action->connection.fromLane) {
        // clang-format off
        throw logic_error(
            "Vehicle::move: vehicle " + to_string(id) +
            " is at lane " + to_string(position.lane.edge.id) + "_" + to_string(position.lane.index) +
            ", but connection " + to_string(action->connection.id) +
            " starts at lane " + to_string(action->connection.fromLane.edge.id) + "_" + to_string(action->connection.fromLane.index)
        );
        // clang-format on
    }

    if(action->connection.toLane.edge != action->lane.edge)
        throw logic_error("Vehicle::move: intention destination lane " + to_string(action->lane.edge.id) + " is not on same edge as connection.to " + to_string(action->connection.toLane.edge.id));

    Lane &fromLane = action->connection.fromLane;
    assert(fromLane.moving.erase(id) == 1);

    if(
        !position.lane.stopped.empty()
        || !action->connection.canPass()
    ) {
        position.lane.stopped.emplace(*this, action);

        position.offset = position.lane.edge.length;

        speed = 0;

        state = State::STOPPED;

        return false;
    }

    Lane &toLane = action->lane;

    // clang-format off
    position = {
        toLane,
        0
    };
    // clang-format on
    speed = toLane.calculateSpeed();

    toLane.moving.insert(id);

    // Reward
    // FIXME: this part is very ooga booga, and not generic at all; ideally we would allow using a generic reward function.
    {
        Time leftLane = env.getTime();
        Time r        = leftLane - enteredLane;
        action->reward(-r);
    }

    enteredLane = env.getTime();

    return true;
}

bool Dynamic::Env::Vehicle::operator<(const Dynamic::Env::Vehicle &other) const {
    return Dynamic::Vehicle::operator<(other);
}
