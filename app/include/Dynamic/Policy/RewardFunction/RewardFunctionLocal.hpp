#pragma once

#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"

namespace Dynamic {
class RewardFunctionLocal: public RewardFunction {
   public:
    virtual ~RewardFunctionLocal() = default;

    virtual Reward operator()(const Env::Env &env, const Env::Vehicle &vehicle) override;

    static RewardFunctionLocal INSTANCE;
};
}  // namespace Dynamic
