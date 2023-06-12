#pragma once

#include <chrono>
#include <initializer_list>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>

#include "Alg/Graph.hpp"
#include "Dynamic/Dynamic.hpp"
#include "Log/ProgressLogger.hpp"

namespace Dynamic {

class Demand;

class Environment {
   public:
    template<typename T, typename... Args>
    class Loader {
       public:
        Environment *load(T arg1, Args... arg2);
    };

    typedef double Length;
    typedef double Speed;

    typedef long Node;

    typedef long VehicleID;
    typedef long ConnectionID;

    struct Edge {
        friend Environment;

        typedef EdgeID ID;

        ID     id;
        Node   u, v;
        Length length;
        size_t nLanes;
        Speed  speed;

        std::set<VehicleID>                   vehicles;
        std::map<ID, std::list<ConnectionID>> outgoingConnections;

        struct Lane {
            typedef size_t Index;
        };

       protected:
        Edge(ID id, Node u, Node v, Length length, size_t nLanes, Speed speed);

       public:
        virtual Speed calculateSpeed() const;

        std::list<ConnectionID> getOutgoingConnections() const;
        std::list<ConnectionID> getOutgoingConnections(Edge::ID destinationEdgeID) const;
    };

    struct Connection {
        typedef ConnectionID ID;

        ID id;

        Edge::ID          fromID, toID;
        Edge::Lane::Index fromLaneIndex, toLaneIndex;

        bool operator==(const Connection &connection) const;

        static const Connection STOP;
        static const Connection LEAVE;
    };

    struct Position {
        Edge::ID edge;
        Length   offset;
    };

    struct Vehicle {
        typedef VehicleID ID;

        class Policy {
           public:
            virtual const Connection &pickConnection(
                const Environment &env
            ) = 0;
        };

        ID       id;
        Time     lastUpdateTime;
        Position position;
        Speed    speed;

        std::shared_ptr<Policy> policy;
    };

    class Event {
        friend Environment;

        Time t;

       public:
        Event(Time t);
        virtual void process(Environment &env) const = 0;

        bool operator<(const Event &event) const;
        bool operator>(const Event &event) const;
    };

   private:
    Time                                 t;
    std::map<Edge::ID, Edge>             edges;
    std::map<Connection::ID, Connection> connections;
    std::map<Vehicle::ID, Vehicle>       vehicles;
    // clang-format off
    std::priority_queue<
        std::shared_ptr<Event>,
        std::vector<std::shared_ptr<Event>>,
        bool (*)(const std::shared_ptr<Event> &, const std::shared_ptr<Event> &)
    > eventQueue;
    // clang-format on

   public:
    Environment(Time startTime = 0);

    Alg::Graph toGraph() const;

    Edge &addEdge(Edge::ID id, Node u, Node v, Length length, size_t nLanes, Speed speed);

    void addDemand(const Demand &demand);

    const std::map<Edge::ID, Edge> &getEdges() const;

    const std::map<Vehicle::ID, Vehicle> &getVehicles() const;

    const std::map<Connection::ID, Connection> &getConnections() const;

    void runUntil(Time t);

    void updateAllVehicles(Time t);

    void log(Log::ProgressLogger &logger, Time tStartSim, Time tEndSim, Time delta);

    class EventComposite: public Event {
        std::vector<std::shared_ptr<Event>> events;

       public:
        EventComposite(Time t);
        EventComposite(Time t, std::initializer_list<std::shared_ptr<Event>> initList);
        void         addEvent(std::shared_ptr<Event> event);
        virtual void process(Environment &env) const;
    };

    /**
     * @brief Try to spawn vehicle.
     *
     * This event instructs the environment to try to spawn a vehicle at time
     * `t`. The vehicle is spawned only if the edge has available space. If not,
     * then a new EventTrySpawnVehicle is scheduled for a later time at which
     * spawning should be retried.
     */
    class EventTrySpawnVehicle;

    /**
     * @brief Update vehicle position and speed.
     *
     * This event takes the latest vehicle information, and applies the
     * respective movement equations to update the vehicle position and speed
     * according to the current time as indicated by the Environment.
     *
     * This step is uniquely linked to the environment model, since the movement
     * equations are part of the environment model.
     */
    class EventUpdateVehicle;

    /**
     * @brief Force vehicle to pick connection in current edge.
     *
     * This event is triggered when a vehicle reaches the end of an edge/lane,
     * and must pick a connection to continue its path.
     *
     * This is where different vehicle routing policies can affect the final
     * result. When processing this event, a policy that is external to the
     * environment is applied.
     *
     * This event is equivalent to forcing the vehicle driver to choose what
     * they want to do. This is also the part where vehicle drivers can express
     * their objectives and path preferences.
     */
    class EventPickConnection;

    class EventLog;
};

}  // namespace Dynamic
