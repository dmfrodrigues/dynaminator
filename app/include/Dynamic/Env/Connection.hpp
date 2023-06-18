#pragma once

#include <functional>

namespace Dynamic::Env {

class Env;
class Lane;
class TrafficLight;

struct Connection {
    typedef long ID;

    ID id;

    const Lane &fromLane, &toLane;

    std::optional<std::reference_wrapper<TrafficLight>> trafficLight;
    std::optional<size_t>                               tlLinkIndex;

    Connection(ID id, const Lane &fromLane, const Lane &toLane);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    static const Connection STOP;
    static const Connection LEAVE;
};

}  // namespace Dynamic::Env
