#pragma once

#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"

namespace Dynamic {
class RewardFunctionGreedy: public RewardFunction {
   public:
    Reward operator()(const Env::Env &env, const Env::Vehicle &vehicle) override;

    static RewardFunctionGreedy INSTANCE;
};
}  // namespace Dynamic
