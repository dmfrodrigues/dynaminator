#pragma once

#include <functional>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "utils/reference_wrapper.hpp"

namespace Dynamic {
class QLearner {
   public:
    typedef Time                              Reward;
    typedef std::reference_wrapper<Env::Lane> State;
    // clang-format off
    typedef std::pair<
        std::reference_wrapper<Env::Connection>,
        std::reference_wrapper<Env::Lane>
    > Action;
    // clang-format on

   private:
    struct ActionCmp {
        bool operator()(const Action& a, const Action& b) const {
            if(a.first.get() != b.first.get()) return a.first.get() < b.first.get();
            return a.second.get() < b.second.get();
        }
    };

    Reward alpha, gamma;

    // clang-format off
    std::unordered_map<
        State,
        std::map<
            Action,
            Reward,
            ActionCmp
        >,
        utils::reference_wrapper::hash    <State::type>,
        utils::reference_wrapper::equal_to<State::type>
    > Q;
    // clang-format on

    Reward estimateInitialValue(State state, Action action);

    void updateMatrix(State state, Action action, Reward reward);

   public:
    QLearner(Reward alpha, Reward gamma, Env::Env& env);

    void setAlpha(Reward alpha);
};
}  // namespace Dynamic
