#pragma once

#include "Dynamic/Demand/Demand.hpp"

namespace Dynamic {
// clang-format off
class PoissonDemandLoader: public Demand::Loader<
    Env::Env &,
    const Static::Demand &
> {
    // clang-format on
    double scale;
    Time   beginTime, endTime;

   public:
    PoissonDemandLoader(double scale, Time beginTime, Time endTime);

    Demand load(
        Env::Env &env,
        const Static::Demand &staticDemand
    );
};
}  // namespace Dynamic
