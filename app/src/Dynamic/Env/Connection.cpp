#include "Dynamic/Env/Connection.hpp"

#include <stdexcept>
#include <string>

#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/TrafficLight.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Connection::Connection(ID id_, Lane &fromLane_, Lane &toLane_):
    id(id_), fromLane(fromLane_), toLane(toLane_) {}

bool Connection::operator==(const Connection &connection) const {
    return id == connection.id;
}

bool Connection::operator!=(const Connection &connection) const {
    return !(*this == connection);
}

bool Connection::canPass() const {
    if(!trafficLight.has_value())
        return true;

    TrafficLight &tl = trafficLight.value();

    TrafficLight::Phase::State state = tl.currentPhase->state.at(tlLinkIndex.value());

    switch(state) {
        case TrafficLight::Phase::State::GREEN:
        case TrafficLight::Phase::State::YELLOW:
            // cerr << "Found a green traffic light" << endl;
            return true;
        case TrafficLight::Phase::State::RED:
            return false;
        default:
            throw logic_error("Connection::canPass: unknown state " + to_string(static_cast<int>(state)));
    }
}

bool Connection::operator<(const Connection &other) const {
    return id < other.id;
}

Connection Connection::STOP  = {-1, Lane::INVALID, Lane::INVALID};
Connection Connection::LEAVE = {-2, Lane::INVALID, Lane::INVALID};
