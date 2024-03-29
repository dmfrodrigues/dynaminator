#pragma once

#include <functional>
#include <list>
#include <map>
#include <memory>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "data/SUMO/Additionals/Location.hpp"
#include "data/SUMO/SUMO.hpp"

namespace SUMO {
class Network {
   public:
    Additionals::Location location;

    struct Junction;
    struct Connection;

    struct Edge {
        typedef SUMO::ID ID;
        typedef int      Priority;

        static const Priority PRIORITY_UNSPECIFIED = -1000;

        enum Function {
            NORMAL,
            INTERNAL,
            CONNECTOR,
            CROSSING,
            WALKINGAREA
        };

        struct Lane {
           public:
            typedef SUMO::ID ID;
            typedef size_t   Index;

            const Network &net;
            const Edge    &edge;

            ID     id;
            Index  index;
            Speed  speed;
            Length length;
            Shape  shape;

            std::vector<std::reference_wrapper<const Connection>> getOutgoing() const;

            Vector2 getIncomingDirection() const;
            Vector2 getOutgoingDirection() const;

            Shape getShape() const;

            bool operator==(const Lane &other) const;
            bool operator<(const Lane &other) const;
        };

        const Network &net;

        const ID                                              id;
        std::optional<SUMO::ID>                               fromID   = std::nullopt;
        std::optional<SUMO::ID>                               toID     = std::nullopt;
        std::optional<std::reference_wrapper<const Junction>> from     = std::nullopt;
        std::optional<std::reference_wrapper<const Junction>> to       = std::nullopt;
        Priority                                              priority = Edge::PRIORITY_UNSPECIFIED;
        Function                                              function = NORMAL;
        Shape                                                 shape    = {};

        std::vector<Lane> lanes = {};

        Length length() const;
        Speed  speed() const;

        Shape getShape() const;

        std::vector<std::reference_wrapper<const Edge>>       getOutgoing() const;
        std::vector<std::reference_wrapper<const Connection>> getOutgoingConnections() const;

        bool operator==(const Edge &other) const;
        bool operator<(const Edge &other) const;
    };

    struct Junction {
        typedef SUMO::ID ID;

        enum Type {
            PRIORITY,
            TRAFFIC_LIGHT,
            RIGHT_BEFORE_LEFT,
            LEFT_BEFORE_RIGHT,
            UNREGULATED,
            TRAFFIC_LIGHT_UNREGULATED,
            PRIORITY_STOP,
            ALLWAY_STOP,
            RAIL_SIGNAL,
            ZIPPER,
            RAIL_CROSSING,
            TRAFFIC_LIGHT_RIGHT_ON_RED,
            DEAD_END,

            INTERNAL,

            UNKNOWN,
            DISTRICT
        };

        struct Request {
            const Network     &net;
            const Junction::ID junctionID;

            Index             index;
            std::vector<bool> response;
            std::vector<bool> foes;
            bool              cont;

            const Junction &junction() const;

            std::vector<std::reference_wrapper<const Connection>> getResponse() const;
        };

        const ID    id;
        const Coord pos;
        Type        type  = UNKNOWN;
        Shape       shape = {};

        std::vector<std::reference_wrapper<const Edge::Lane>> incLanes = {};
        std::vector<std::reference_wrapper<const Edge::Lane>> intLanes = {};

        std::map<Index, Request> requests = {};

        /**
         * @brief Get connections by order.
         *
         * The order of connections is as specified in
         * https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html#requests
         *
         * @return std::vector<const Connection *>
         */
        std::vector<std::reference_wrapper<const Connection>> getConnections() const;

        std::vector<std::reference_wrapper<const Edge::Lane>> outLanes() const;

        bool operator==(const Junction &other) const;
    };

    struct TrafficLightLogic {
        typedef SUMO::ID ID;
        enum Type {
            STATIC,
            ACTUATED,
            DELAY_BASED
        };
        typedef std::string ProgramID;

        const ID        id;
        const Type      type;
        const ProgramID programId;
        const Time      offset;

        struct Phase {
            // https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
            enum State {
                RED,
                YELLOW_STOP,
                GREEN_NOPRIORITY,
                GREEN_PRIORITY,
                GREEN_RIGHT,
                YELLOW_START,
                OFF_YIELD,
                OFF
            };

            const Time duration;

            const std::vector<State> state;
        };

        // The key is the time at which the phase starts.
        std::map<Time, Phase> phases = {};

        Time   getGreenTime(size_t linkIndex) const;
        Time   getCycleTime() const;
        size_t getNumberStops(size_t linkIndex) const;

        const Phase &getPhase(Time time) const;
    };

    typedef std::unordered_map<TrafficLightLogic::ID, TrafficLightLogic> TrafficLights;

    /**
     * @brief Connection
     *
     * The meaning of indices in SUMO is explained in this link:
     * https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html#indices_of_a_connection
     *
     * From what I understand, in a connection the linkIndex reports to the TL
     * index that controls the connection.
     *
     * The junction index is used to determine connections with conflicts. This
     * junction index is deduced from the order of ...
     */
    struct Connection {
        const Edge &from, &to;
        const Index fromLaneIndex, toLaneIndex;

        enum Direction {
            INVALID,
            STRAIGHT,
            TURN,  /// @brief Turnaround (as in a deadend)
            LEFT,
            RIGHT,
            PARTIALLY_LEFT,
            PARTIALLY_RIGHT
        };
        Direction dir;

        enum State {
            DEAD_END,  // Not very used in SUMO files
            EQUAL,
            MINOR_LINK,
            MAJOR_LINK,
            CONTROLLER_OFF,
            YELLOW_FLASHING,
            YELLOW_MINOR_LINK,  // Not very used in SUMO files
            YELLOW_MAJOR_LINK,  // Not very used in SUMO files
            RED,                // Not very used in SUMO files
            GREEN_MINOR,        // Not very used in SUMO files
            GREEN_MAJOR         // Not very used in SUMO files
        };
        const State state;

        std::optional<std::reference_wrapper<const Edge::Lane>> via = std::nullopt;

        std::optional<std::reference_wrapper<const TrafficLightLogic>> tl        = std::nullopt;
        std::optional<size_t>                                          linkIndex = std::nullopt;

        const Edge::Lane &fromLane() const;
        const Edge::Lane &toLane() const;

        const Junction &getJunction() const;

        size_t getJunctionIndex() const;

        Time   getGreenTime() const;
        Time   getCycleTime() const;
        size_t getNumberStops() const;

        const Junction::Request &getRequest() const;

        bool operator==(const Connection &other) const;

        const TrafficLightLogic::Phase::State &getTrafficLightState(Time t) const;
    };

   private:
    // clang-format off
    typedef std::unordered_map<
        SUMO::Network::Edge::ID, std::unordered_map<
            SUMO::Index, std::unordered_map<
                SUMO::Network::Edge::ID, std::unordered_map<
                    SUMO::Index,
                    SUMO::Network::Connection
                >
            >
        >
    > Connections;
    // clang-format on

    std::unordered_map<Junction::ID, Junction> junctions;
    std::unordered_map<Edge::ID, Edge>         edges;

    // clang-format off
    std::unordered_map<
        Junction::ID, std::unordered_map<
            Junction::ID,
            std::list<std::reference_wrapper<Edge>>
        >
    > edgesByJunctions;
    // clang-format on

    std::unordered_map<Edge::Lane::ID, std::pair<std::string, Edge::Lane::Index>> lanes;
    TrafficLights                                                                 trafficLights;
    Connections                                                                   connections;

    Junction          &loadJunction(const rapidxml::xml_node<> *it);
    Edge              &loadEdge(const rapidxml::xml_node<> *it);
    TrafficLightLogic &loadTrafficLightLogic(const rapidxml::xml_node<> *it);
    Connection        &loadConnection(const rapidxml::xml_node<> *it);

   public:
    static std::shared_ptr<SUMO::Network> loadFromFile(const std::string &path);

    std::vector<Junction> getJunctions() const;
    const Junction       &getJunction(const Junction::ID &id) const;

    std::vector<Edge> getEdges() const;
    const Edge       &getEdge(const Edge::ID &id) const;

    std::vector<std::reference_wrapper<const Connection>>                                                                                                        getConnections(const Edge &e1, const Edge &e2) const;
    std::unordered_map<SUMO::Network::Edge::ID, std::unordered_map<SUMO::Network::Edge::ID, std::list<std::reference_wrapper<const SUMO::Network::Connection>>>> getConnections() const;
    const TrafficLights                                                                                                                                         &getTrafficLights() const;

    void saveStatsToFile(const std::string &path) const;
};

typedef std::vector<SUMO::Network::Edge::ID> Route;
}  // namespace SUMO

namespace utils::stringify {
template<>
class stringify<SUMO::Network::Edge::Function> {
   public:
    static SUMO::Network::Edge::Function fromString(const std::string &s);

    static std::string toString(const SUMO::Network::Edge::Function &t);
};

template<>
class stringify<SUMO::Network::Junction::Type> {
   public:
    static SUMO::Network::Junction::Type fromString(const std::string &s);

    static std::string toString(const SUMO::Network::Junction::Type &t);
};

template<>
class stringify<SUMO::Network::TrafficLightLogic::Type> {
   public:
    static SUMO::Network::TrafficLightLogic::Type fromString(const std::string &s);

    static std::string toString(const SUMO::Network::TrafficLightLogic::Type &t);
};

template<>
class stringify<SUMO::Network::TrafficLightLogic::Phase::State> {
   public:
    static SUMO::Network::TrafficLightLogic::Phase::State fromString(const std::string &s);

    static std::string toString(const SUMO::Network::TrafficLightLogic::Phase::State &t);
};

template<>
class stringify<std::vector<SUMO::Network::TrafficLightLogic::Phase::State>> {
   public:
    static std::vector<SUMO::Network::TrafficLightLogic::Phase::State> fromString(const std::string &s);

    static std::string toString(const std::vector<SUMO::Network::TrafficLightLogic::Phase::State> &t);
};

template<>
class stringify<SUMO::Network::Connection::Direction> {
   public:
    static SUMO::Network::Connection::Direction fromString(const std::string &s);

    static std::string toString(const SUMO::Network::Connection::Direction &t);
};

template<>
class stringify<SUMO::Network::Connection::State> {
   public:
    static SUMO::Network::Connection::State fromString(const std::string &s);

    static std::string toString(const SUMO::Network::Connection::State &t);
};
}  // namespace utils::stringify
