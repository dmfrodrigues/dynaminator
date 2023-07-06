#pragma once

#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"

namespace Dynamic {
class RewardFunctionDifference: public RewardFunction {
    double w;

   public:
    RewardFunctionDifference(double greediness = 0.0);
    virtual ~RewardFunctionDifference() = default;

    virtual Reward operator()(const Env::Env &env, const Env::Vehicle &vehicle) override;

    static RewardFunctionDifference INSTANCE;
};
}  // namespace Dynamic
