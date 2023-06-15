#pragma once

#include <random>

#include "Dynamic/Demand/Demand.hpp"
#include "Dynamic/Env/Env.hpp"

namespace Dynamic {
// clang-format off
class UniformDemandLoader: public Demand::Loader<
    const Static::Demand &,
    const Env::Env &,
    const Dynamic::SUMOAdapter &
> {
    // clang-format on

    double scale;
    Time   beginTime, endTime;

    Vehicle::Policy::Factory &policyFactory;

    std::mt19937 gen;

   public:
    UniformDemandLoader(
        double                          scale,
        Time                            beginTime,
        Time                            endTime,
        Vehicle::Policy::Factory       &policyFactory,
        std::random_device::result_type seed = std::random_device()()
    );

    Demand load(
        const Static::Demand       &staticDemand,
        const Env::Env             &env,
        const Dynamic::SUMOAdapter &sumoAdapter
    );
};
}  // namespace Dynamic
