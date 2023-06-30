#pragma once

#include <functional>
#include <limits>
#include <unordered_set>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Env;
class Lane;
class TrafficLight;
class Connection;

}  // namespace Dynamic::Env

namespace std {
template<>
struct hash<Dynamic::Env::Connection> {
    size_t operator()(const Dynamic::Env::Connection &connection) const;
};
}  // namespace std

namespace Dynamic::Env {

class Connection {
    friend Env;

   public:
    typedef long ID;

    static const Time CRITICAL_GAP;

    ID id;

    Lane &fromLane, &toLane;

    std::optional<std::reference_wrapper<TrafficLight>> trafficLight;
    std::optional<size_t>                               tlLinkIndex;

    Time lastUsed = -std::numeric_limits<Time>::infinity();

   private:
    Connection(ID id, Lane &fromLane, Lane &toLane);

    typedef std::unordered_set<std::reference_wrapper<Connection>, std::hash<Connection>, std::equal_to<Connection>> SetConnections;

    SetConnections lessImportant;  /// @brief Connections that are less important than this one; this one causes lessImportant to block
    SetConnections moreImportant;  /// @brief Connections that are more important than this one; this one is blocked by moreImportant

   public:
    void addMoreImportant(Connection &otherConnection);

    bool operator==(const Connection &connection) const;
    bool operator!=(const Connection &connection) const;

    bool canPass() const;

    bool operator<(const Connection &other) const;

    Time getMinExpectedStopTimeTL() const;

    bool yieldsTo(const Connection &other) const;

    static Connection STOP;
    static Connection LEAVE;
    static Connection INVALID;
};

}  // namespace Dynamic::Env
