#include "Dynamic/Env/Connection.hpp"

#include "Dynamic/Env/Lane.hpp"

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

Connection Connection::STOP  = {-1, Lane::INVALID, Lane::INVALID};
Connection Connection::LEAVE = {-2, Lane::INVALID, Lane::INVALID};
