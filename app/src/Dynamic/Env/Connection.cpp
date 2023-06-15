#include "Dynamic/Env/Connection.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Connection::Connection(ID id_, const Edge::Lane &fromLane_, const Edge::Lane &toLane_):
    id(id_), fromLane(fromLane_), toLane(toLane_) {}

bool Connection::operator==(const Connection &connection) const {
    return id == connection.id;
}

bool Connection::operator!=(const Connection &connection) const {
    return !(*this == connection);
}

const Connection Connection::STOP  = {-1, Edge::Lane::INVALID, Edge::Lane::INVALID};
const Connection Connection::LEAVE = {-2, Edge::Lane::INVALID, Edge::Lane::INVALID};
