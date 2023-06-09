#include <iostream>
#include <random>

#include "Alg/ShortestPath/Dijkstra.hpp"
#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Demand.hpp"
#include "Dynamic/Environment.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

Demand::UniformLoader::UniformLoader(
    double scale_,
    Time   beginTime_,
    Time   endTime_
):
    scale(scale_),
    beginTime(beginTime_),
    endTime(endTime_) {}

pair<Environment::Edge::ID, Environment::Edge::ID> pickSourceSink(
    const Environment &env,
    const vector<Environment::Edge::ID> &sources,
    const vector<Environment::Edge::ID> &sinks,
    mt19937 &gen,
    const Alg::ShortestPath::ShortestPathManyMany &sp
) {
    Environment::Edge::ID a, b;
    Environment::Node aDest, bOrig;

    uniform_int_distribution<> distSource(0, sources.size() - 1);
    uniform_int_distribution<> distSink(0, sinks.size() - 1);

    // TODO: this is very inefficient. It is better to do DijkstraManyMany
    // to speed up. Since the number of origins/destinations is only ~100,
    // it is faster than finding a path for all 100k vehicles.

    while(true) {
        a = sources.at(distSource(gen));
        b = sinks.at(distSink(gen));

        aDest = env.getEdges().at(a).v;
        bOrig = env.getEdges().at(b).u;

        if(sp.hasVisited(aDest, bOrig))
            break;
    }

    return {a, b};
}

Demand Demand::UniformLoader::load(
    const Static::Demand       &staticDemand,
    const Environment          &env,
    const Dynamic::SUMOAdapter &sumoAdapter
) {
    Demand demand;

    std::random_device rd;
    std::mt19937       gen(rd());

    std::uniform_real_distribution<> dist(0, 1);

    vector<Environment::Node> startNodes;
    for(const Static::Network::Node &u: staticDemand.getStartNodes()){
        SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
        list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
        for(const SUMO::TAZ::Source &source: sourcesList)
            if(source.weight > 0.0) {
                Environment::Edge::ID edgeID = sumoAdapter.toEdge(source.id);
                Environment::Node nodeID = env.getEdges().at(edgeID).v;
                startNodes.push_back(nodeID);
            }
    }

    Alg::Graph G = env.toGraph();
    Alg::ShortestPath::DijkstraMany shortestPathManyMany;

    shortestPathManyMany.solve(G, startNodes);

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow  f  = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1 / (f * scale);

            SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
            SUMO::TAZ::ID toTAZ = sumoAdapter.toSumoTAZ(v);

            list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
            list<SUMO::TAZ::Sink> sinksList = sumoAdapter.toTAZEdges(toTAZ).second;

            vector<Environment::Edge::ID> sources;
            for(const SUMO::TAZ::Source &source: sourcesList)
                if(source.weight > 0.0)
                    sources.push_back(sumoAdapter.toEdge(source.id));

            vector<Environment::Edge::ID> sinks;
            for(const SUMO::TAZ::Sink &sink: sinksList)
                if(sink.weight > 0.0)
                    sinks.push_back(sumoAdapter.toEdge(sink.id));

            for(Time t = beginTime + Dt * dist(gen); t < endTime; t += Dt) {

                auto [sourceID, sinkID] = pickSourceSink(env, sources, sinks, gen, shortestPathManyMany);

                /**
                 * TODO: check if `b` is reachable from `a`.
                 *
                 * This is generally not an issue in residential streets, but
                 * in highway on/off-ramps outside the area of interest (which
                 * are not connected to other road elements) e.g. on-ramps are
                 * defined both as source and sink, it is possible that `a` is
                 * assigned to an off-ramp and as such is an invalid spawn edge,
                 * since it is a one-way road that leads into nothing.
                 */

                Demand::Vehicle::ID id = demand.addVehicle(t, sourceID, sinkID);

                // cerr << "UniformLoader: added vehicle " << id
                // << " at time " << t
                // << " from " << a
                // << " to " << b
                // << "\n";
            }
        }
    }

    return demand;
}
