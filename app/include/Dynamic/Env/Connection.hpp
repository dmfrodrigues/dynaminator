#pragma once

#include "Dynamic/Env/Edge.hpp"

namespace Dynamic::Env {

class Env;

struct Connection {
    typedef long ID;

    ID id;

    const Lane &fromLane, &toLane;

    Connection(ID id, const Lane &fromLane, const Lane &toLane);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    static const Connection STOP;
    static const Connection LEAVE;
};

}  // namespace Dynamic::Env
