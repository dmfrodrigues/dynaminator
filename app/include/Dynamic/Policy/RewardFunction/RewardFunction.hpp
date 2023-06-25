#pragma once

#include "Dynamic/Dynamic.hpp"

namespace Dynamic {
namespace Env {
class Env;
class Vehicle;
}  // namespace Env

class RewardFunction {
   public:
    virtual Reward operator()(const Env::Env &env, const Env::Vehicle &vehicle) = 0;

    virtual ~RewardFunction() = default;
};
}  // namespace Dynamic
