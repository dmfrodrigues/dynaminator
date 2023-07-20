#pragma once

#include <functional>
#include <map>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Policy/Action.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {
class Vehicle: public Dynamic::Vehicle {
   public:
    enum class State : int {
        MOVING  = 1,  /// @brief Vehicle is moving
        STOPPED = 2,  /// @brief Vehicle is stopped at a queue
        LEFT    = 3   /// @brief Vehicle has left the simulation
    };

    class Path {
        std::vector<std::pair<Time, std::reference_wrapper<const Lane>>> v;

        // clang-format off
        std::map<
            std::reference_wrapper<const Lane>,
            size_t,
            std::less<Lane>
        > ms;
        // clang-format on

       public:
        typedef std::vector<std::pair<Time, std::reference_wrapper<const Lane>>>::iterator       iterator;
        typedef std::vector<std::pair<Time, std::reference_wrapper<const Lane>>>::const_iterator const_iterator;

        iterator begin();
        iterator end();

        const_iterator begin() const;
        const_iterator end() const;

        const std::pair<Time, std::reference_wrapper<const Lane>> &front() const;
        const std::pair<Time, std::reference_wrapper<const Lane>> &back() const;

        void emplace_connection(Time t, const Connection &connection);
        void emplace_lane(Time t, const Lane &lane);

        size_t count(const Lane &lane) const;

        size_t size() const;
        bool   empty() const;
    };

    static const Length LENGTH;

    Time     lastUpdateTime;
    Position position;
    Speed    speed;
    State    state;

    std::shared_ptr<Action> prevAction;

    Time enteredLane;

    Path path;

    Vehicle(
        const Dynamic::Vehicle &vehicle,
        Time                    t,
        Position                position,
        Speed                   speed,
        State                   state
    );

    std::shared_ptr<Action> pickConnection(Env &env) const;

    void moveToAnotherEdge(Env &env, std::shared_ptr<Action> action);

    bool operator==(const Dynamic::Env::Vehicle &other) const;
};
}  // namespace Dynamic::Env
