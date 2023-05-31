#include <random>
#include "Dynamic/Demand.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

Demand::UniformLoader::UniformLoader(
    double scale_,
    Time startTime_,
    Time endTime_
) :
    scale(scale_),
    startTime(startTime_),
    endTime(endTime_) {}

Demand Demand::UniformLoader::load(const Static::Demand &staticDemand) {
    Demand demand;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 1);
    
    for(const Static::Network::Node &u: staticDemand.getStartNodes()){
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)){
            Static::Network::Flow f = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1/(f * scale);
            for(Time t = startTime + Dt * dist(gen); t < endTime; t += Dt){
                demand.addVehicle(t, u, v);
            }
        }
    }

    return demand;
}
