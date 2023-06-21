#include "Dynamic/Demand/PoissonDemandLoader.hpp"

#include <random>

#include "Dynamic/Env/Env.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

PoissonDemandLoader::PoissonDemandLoader(
    double scale_,
    Time   beginTime_,
    Time   endTime_
):
    scale(scale_),
    beginTime(beginTime_),
    endTime(endTime_) {}

Demand PoissonDemandLoader::load(
    Env::Env             &env,
    const Static::Demand &staticDemand
) {
    Demand demand;

    std::mt19937 gen(0);

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow f      = staticDemand.getDemand(u, v);
            double       lambda = f * scale;

            std::exponential_distribution<> dist(lambda);

            Env::Edge &from = env.getEdge(u), &to = env.getEdge(v);

            for(Time t = beginTime + dist(gen); t < endTime; t += dist(gen)) {
                demand.addVehicle(t, from, to, shared_ptr<Env::Vehicle::Policy>());
            }
        }
    }

    return demand;
}
