#include "Dynamic/Demand/UniformDemandLoader.hpp"

#include <iostream>
#include <random>

#include "Alg/ShortestPath/Dijkstra.hpp"
#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Env/Edge.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Policy/RandomPolicy.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

UniformDemandLoader::UniformDemandLoader(
    double                     scale_,
    Time                       beginTime_,
    Time                       endTime_,
    Policy::Factory           &policyFactory_,
    random_device::result_type seed
):
    scale(scale_),
    beginTime(beginTime_),
    endTime(endTime_),
    policyFactory(policyFactory_),
    gen(seed) {}

pair<Demand, Vehicle::ID> UniformDemandLoader::load(
    const Static::Demand       &staticDemand,
    Env::Env                   &env,
    const Dynamic::SUMOAdapter &sumoAdapter,
    Vehicle::ID                 nextID
) {
    Demand demand;

    uniform_real_distribution<> dist(0, 1);

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow  f  = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1 / (f * scale);

            SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
            SUMO::TAZ::ID toTAZ   = sumoAdapter.toSumoTAZ(v);

            Env::TAZ &envFromTAZ = env.getTAZ(sumoAdapter.toTAZ(fromTAZ));
            Env::TAZ &envToTAZ   = env.getTAZ(sumoAdapter.toTAZ(toTAZ));

            for(Time t = beginTime + Dt * dist(gen); t < endTime; t += Dt) {
                Vehicle::ID id = nextID++;

                shared_ptr<Policy> policy = policyFactory.create(
                    id,
                    t,
                    envFromTAZ,
                    envToTAZ
                );

                demand.addVehicle(
                    id,
                    t,
                    envFromTAZ,
                    envToTAZ,
                    policy
                );
            }
        }
    }

    return {demand, nextID};
}
