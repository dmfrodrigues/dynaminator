#pragma once

#include "Dynamic/Env/Edge.hpp"

namespace Dynamic::Env {

class Env;

struct Connection {
    typedef long ID;

    ID id;

    const Edge &from, &to;

    // Edge::Lane::Index fromLaneIndex, toLaneIndex;

    Connection(ID id, const Edge &from, const Edge &to);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    static const Connection STOP;
    static const Connection LEAVE;
};

}  // namespace Dynamic::Env
