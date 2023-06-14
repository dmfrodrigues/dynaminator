#pragma once

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

   public:
    UniformDemandLoader(
        double                    scale,
        Time                      beginTime,
        Time                      endTime,
        Vehicle::Policy::Factory &policyFactory
    );

    Demand load(
        const Static::Demand       &staticDemand,
        const Env::Env             &env,
        const Dynamic::SUMOAdapter &sumoAdapter
    );
};
}  // namespace Dynamic
