#include "Dynamic/Policy/Action.hpp"

#include "Dynamic/Env/Connection.hpp"
#include "Dynamic/Env/Lane.hpp"

using namespace std;
using namespace Dynamic::Env;

Action::Action(
    Connection &connection_,
    Lane       &lane_
):
    connection(connection_), lane(lane_) {}

Action::Action():
    connection(Connection::LEAVE), lane(Lane::INVALID) {}

bool Action::operator<(const Action &other) const {
    if(connection != other.connection)
        return connection < other.connection;
    else
        return lane < other.lane;
}
