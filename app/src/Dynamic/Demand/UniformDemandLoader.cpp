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
    Vehicle::Policy::Factory  &policyFactory_,
    random_device::result_type seed
):
    scale(scale_),
    beginTime(beginTime_),
    endTime(endTime_),
    policyFactory(policyFactory_),
    gen(seed) {}

pair<Env::Edge &, Env::Edge &> pickSourceSink(
    Env::Env                                      &env,
    const vector<Env::Edge::ID>                   &sources,
    const vector<Env::Edge::ID>                   &sinks,
    mt19937                                       &gen,
    const Alg::ShortestPath::ShortestPathManyMany &sp
) {
    uniform_int_distribution<> distSource(0, sources.size() - 1);
    uniform_int_distribution<> distSink(0, sinks.size() - 1);

    // TODO: this is very inefficient. It is better to do DijkstraManyMany
    // to speed up. Since the number of origins/destinations is only ~100,
    // it is faster than finding a path for all 100k vehicles.

    while(true) {
        Env::Edge::ID aID = sources.at(distSource(gen));
        Env::Edge::ID bID = sinks.at(distSink(gen));

        Env::Edge &a = env.getEdge(aID), &b = env.getEdge(bID);

        if(sp.hasVisited(a.u, b.v)) {
            return {a, b};
        }
    }
}

Demand UniformDemandLoader::load(
    const Static::Demand       &staticDemand,
    Env::Env                   &env,
    const Dynamic::SUMOAdapter &sumoAdapter
) {
    Demand demand;

    std::uniform_real_distribution<> dist(0, 1);

    vector<Env::Node> startNodes;
    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        SUMO::TAZ::ID           fromTAZ     = sumoAdapter.toSumoTAZ(u);
        list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
        for(const SUMO::TAZ::Source &source: sourcesList)
            if(source.weight > 0.0) {
                Env::Edge::ID edgeID = sumoAdapter.toEdge(source.id);
                Env::Node     nodeID = env.getEdge(edgeID).u;
                startNodes.push_back(nodeID);
            }
    }

    Alg::Graph                      G = env.toGraph();
    Alg::ShortestPath::DijkstraMany shortestPathManyMany;

    shortestPathManyMany.solve(G, startNodes);

    Vehicle::ID nextID = 0;

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow  f  = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1 / (f * scale);

            SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
            SUMO::TAZ::ID toTAZ   = sumoAdapter.toSumoTAZ(v);

            list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
            list<SUMO::TAZ::Sink>   sinksList   = sumoAdapter.toTAZEdges(toTAZ).second;

            vector<Env::Edge::ID> sources;
            for(const SUMO::TAZ::Source &source: sourcesList)
                if(source.weight > 0.0)
                    sources.push_back(sumoAdapter.toEdge(source.id));

            vector<Env::Edge::ID> sinks;
            for(const SUMO::TAZ::Sink &sink: sinksList)
                if(sink.weight > 0.0)
                    sinks.push_back(sumoAdapter.toEdge(sink.id));

            for(Time t = beginTime + Dt * dist(gen); t < endTime; t += Dt) {
                auto [from, to] = pickSourceSink(env, sources, sinks, gen, shortestPathManyMany);

                Vehicle::ID id = nextID++;

                shared_ptr<Env::Vehicle::Policy> policy = policyFactory.create(
                    id,
                    t,
                    from,
                    to
                );

                demand.addVehicle(
                    id,
                    t,
                    from,
                    to,
                    policy
                );

                // cerr << "UniformDemandLoader: added vehicle " << id
                // << " at time " << t
                // << " from " << a
                // << " to " << b
                // << "\n";
            }
        }
    }

    return demand;
}
