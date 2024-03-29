#pragma once

#include <random>

#include "Dynamic/Demand/Demand.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Policy/Policy.hpp"

namespace Dynamic {
// clang-format off
class UniformDemandLoader: public Demand::Loader<
    const Static::Demand &,
    Env::Env &,
    const Dynamic::SUMOAdapter &,
    Vehicle::ID
> {
    // clang-format on

    double scale;
    Time   beginTime, endTime;

    Policy::Factory &policyFactory;

    std::mt19937 gen;

   public:
    UniformDemandLoader(
        double                          scale,
        Time                            beginTime,
        Time                            endTime,
        Policy::Factory                &policyFactory,
        std::random_device::result_type seed = 0
    );

    std::pair<Demand, Vehicle::ID> load(
        const Static::Demand       &staticDemand,
        Env::Env                   &env,
        const Dynamic::SUMOAdapter &sumoAdapter,
        Vehicle::ID                 nextID = 0
    );
};
}  // namespace Dynamic
