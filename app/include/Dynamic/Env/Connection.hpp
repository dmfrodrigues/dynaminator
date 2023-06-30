#pragma once

#include <functional>
#include <list>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Env;
class Lane;
class TrafficLight;

class Connection {
    friend Env;

   public:
    typedef long ID;

    ID id;

    Lane &fromLane, &toLane;

    std::optional<std::reference_wrapper<TrafficLight>> trafficLight;
    std::optional<size_t>                               tlLinkIndex;

   private:
    Connection(ID id, Lane &fromLane, Lane &toLane);

    std::list<std::reference_wrapper<Connection>> lessImportant;  /// @brief Connections that are less important than this one; this one causes lessImportant to block
    std::list<std::reference_wrapper<Connection>> moreImportant;  /// @brief Connections that are more important than this one; this one is blocked by moreImportant

   public:
    void addMoreImportant(Connection &otherConnection);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    bool canPass() const;

    bool operator<(const Connection &other) const;

    Time getMinExpectedStopTimeTL() const;

    static Connection STOP;
    static Connection LEAVE;
    static Connection INVALID;
};

}  // namespace Dynamic::Env
