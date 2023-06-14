#include "Dynamic/Env/Connection.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Connection::Connection(ID id_, const Edge &from_, const Edge &to_):
    id(id_), from(from_), to(to_) {}

bool Connection::operator==(const Connection &connection) const {
    return id == connection.id;
}

bool Connection::operator!=(const Connection &connection) const {
    return !(*this == connection);
}

const Connection Connection::STOP  = {-1, Edge::INVALID, Edge::INVALID};
const Connection Connection::LEAVE = {-2, Edge::INVALID, Edge::INVALID};
