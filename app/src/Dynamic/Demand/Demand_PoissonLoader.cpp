#include <random>
#include "Dynamic/Demand.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

Demand::PoissonLoader::PoissonLoader(
    double scale_,
    Time startTime_,
    Time endTime_
) :
    scale(scale_),
    startTime(startTime_),
    endTime(endTime_) {}

Demand Demand::PoissonLoader::load(const Static::Demand &staticDemand) {
    Demand demand;

    std::random_device rd;
    std::mt19937 gen(rd());
    
    for(const Static::Network::Node &u: staticDemand.getStartNodes()){
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)){
            Static::Network::Flow f = staticDemand.getDemand(u, v);
            double lambda = f*scale;
            std::exponential_distribution<> dist(lambda);
            for(Time t = startTime + dist(gen); t < endTime; t += dist(gen)){
                demand.addVehicle(t, u, v);
            }
        }
    }

    return demand;
}
