#pragma once

#include <functional>

namespace Dynamic::Env {

class Env;
class Lane;
class TrafficLight;

struct Connection {
    typedef long ID;

    ID id;

    Lane &fromLane, &toLane;

    std::optional<std::reference_wrapper<TrafficLight>> trafficLight;
    std::optional<size_t>                               tlLinkIndex;

    Connection(ID id, Lane &fromLane, Lane &toLane);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    static Connection STOP;
    static Connection LEAVE;
};

}  // namespace Dynamic::Env
