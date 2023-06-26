#pragma once

#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"

namespace Dynamic {
class RewardFunctionDifference: public RewardFunction {
   public:
    virtual ~RewardFunctionDifference() = default;

    virtual Reward operator()(const Env::Env &env, const Env::Vehicle &vehicle) override;

    static RewardFunctionDifference INSTANCE;
};
}  // namespace Dynamic
