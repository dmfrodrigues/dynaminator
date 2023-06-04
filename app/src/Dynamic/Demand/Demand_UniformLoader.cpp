#include <iostream>
#include <random>

#include "Dynamic/Demand.hpp"
#include "Dynamic/SUMOAdapter.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Dynamic;

Demand::UniformLoader::UniformLoader(
    double scale_,
    Time   startTime_,
    Time   endTime_
):
    scale(scale_),
    startTime(startTime_),
    endTime(endTime_) {}

Demand Demand::UniformLoader::load(
    const Static::Demand       &staticDemand,
    const Dynamic::SUMOAdapter &sumoAdapter
) {
    Demand demand;

    std::random_device rd;
    std::mt19937       gen(rd());

    std::uniform_real_distribution<> dist(0, 1);

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow  f  = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1 / (f * scale);

            SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
            SUMO::TAZ::ID toTAZ = sumoAdapter.toSumoTAZ(v);

            list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
            list<SUMO::TAZ::Sink> sinksList = sumoAdapter.toTAZEdges(toTAZ).second;

            vector<SUMO::TAZ::Source> sources(sourcesList.begin(), sourcesList.end());
            vector<SUMO::TAZ::Sink> sinks(sinksList.begin(), sinksList.end());

            std::uniform_int_distribution<> distSource(0, sources.size() - 1);
            std::uniform_int_distribution<> distSink(0, sinks.size() - 1);

            for(Time t = startTime + Dt * dist(gen); t < endTime; t += Dt) {
                SUMO::Network::Edge::ID sourceID = sources.at(distSource(gen)).id;
                SUMO::Network::Edge::ID sinkID = sinks.at(distSink(gen)).id;

                Environment::Edge::ID a = sumoAdapter.toEdge(sourceID);
                Environment::Edge::ID b = sumoAdapter.toEdge(sinkID);

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

                Demand::Vehicle::ID id = demand.addVehicle(t, a, b);

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
