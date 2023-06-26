#pragma once

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Connection;
class Lane;

class Action {
   public:
    typedef Time Reward;

    Connection &connection;
    Lane       &lane;

    Action();
    Action(Connection &connection, Lane &lane);

    virtual void reward(Reward r) = 0;

    bool operator<(const Action &other) const;
};
}  // namespace Dynamic::Env
