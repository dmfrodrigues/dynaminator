#include "Dynamic/Env/Connection.hpp"

#include <stdexcept>
#include <string>

#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/TrafficLight.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

const Time Connection::CRITICAL_GAP = 3.0;

Connection::Connection(ID id_, Lane &fromLane_, Lane &toLane_):
    id(id_), fromLane(fromLane_), toLane(toLane_) {}

void Connection::addMoreImportant(Connection &otherConnection) {
    moreImportant.insert(otherConnection);
    otherConnection.lessImportant.insert(*this);
}

bool Connection::operator==(const Connection &connection) const {
    return id == connection.id;
}

bool Connection::operator!=(const Connection &connection) const {
    return !(*this == connection);
}

bool Connection::canPass() const {
    if(toLane.isFull()) return false;

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

Time Connection::getMinExpectedStopTimeTL() const {
    if(!trafficLight.has_value())
        return 0;

    TrafficLight &tl = trafficLight.value();

    Time ret = 0;
    for(const auto &[_, phase]: tl.phases) {
        const TrafficLight::Phase::State &state = phase.state.at(tlLinkIndex.value());

        Time wait = 0;

        switch(state) {
            case TrafficLight::Phase::State::RED:
                wait = phase.duration / 2.0;
                break;
            case TrafficLight::Phase::State::YELLOW:
            case TrafficLight::Phase::State::GREEN:
            default:
                break;
        }

        ret += wait * phase.duration;
    }

    ret /= tl.duration();

    return ret;
}

bool Connection::yieldsTo(const Connection &other) const {
    const reference_wrapper<Connection> otherWrapper(const_cast<Connection &>(other));
    return moreImportant.find(otherWrapper) != moreImportant.end();
}

Time Connection::mustYieldUntil() const {
    Time t = -std::numeric_limits<Time>::infinity();
    for(const Connection &connection: moreImportant) {
        t = max(t, connection.lastUsed);
    }
    return t + CRITICAL_GAP;
}

Connection Connection::STOP    = {-1, Lane::INVALID, Lane::INVALID};
Connection Connection::LEAVE   = {-2, Lane::INVALID, Lane::INVALID};
Connection Connection::INVALID = {-3, Lane::INVALID, Lane::INVALID};

size_t std::hash<Connection>::operator()(const Connection &connection) const {
    return hash<Connection::ID>()(connection.id);
}
