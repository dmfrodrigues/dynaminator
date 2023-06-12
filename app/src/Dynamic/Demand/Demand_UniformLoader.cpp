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

class RandomPolicy: public Environment::Vehicle::Policy {
    Demand::Vehicle::ID id;

   public:
    RandomPolicy(Demand::Vehicle::ID id_):
        id(id_) {}

    virtual const Environment::Connection &pickConnection(
        const Environment &env
    ) override {
        const Environment::Vehicle &vehicle = env.getVehicles().at(id);

        const Environment::Edge &edge = env.getEdges().at(vehicle.position.edge);

        list<Environment::Connection::ID> connections = edge.getOutgoingConnections();

        if(connections.size() == 0) {
            return Environment::Connection::LEAVE;
        }

        int n = rand() % connections.size();
        
        auto it = connections.begin();
        advance(it, n);

        const Environment::Connection &connection = env.getConnections().at(*it);

        return connection;
    }
};

Demand::UniformLoader::UniformLoader(
    double scale_,
    Time   beginTime_,
    Time   endTime_
):
    scale(scale_),
    beginTime(beginTime_),
    endTime(endTime_) {}

pair<Environment::Edge::ID, Environment::Edge::ID> pickSourceSink(
    const Environment                             &env,
    const vector<Environment::Edge::ID>           &sources,
    const vector<Environment::Edge::ID>           &sinks,
    mt19937                                       &gen,
    const Alg::ShortestPath::ShortestPathManyMany &sp
) {
    Environment::Edge::ID a, b;
    Environment::Node     aDest, bOrig;

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
    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        SUMO::TAZ::ID           fromTAZ     = sumoAdapter.toSumoTAZ(u);
        list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
        for(const SUMO::TAZ::Source &source: sourcesList)
            if(source.weight > 0.0) {
                Environment::Edge::ID edgeID = sumoAdapter.toEdge(source.id);
                Environment::Node     nodeID = env.getEdges().at(edgeID).v;
                startNodes.push_back(nodeID);
            }
    }

    Alg::Graph                      G = env.toGraph();
    Alg::ShortestPath::DijkstraMany shortestPathManyMany;

    shortestPathManyMany.solve(G, startNodes);

    Demand::Vehicle::ID nextID = 0;

    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        for(const Static::Network::Node &v: staticDemand.getDestinations(u)) {
            Static::Flow  f  = staticDemand.getDemand(u, v);
            Dynamic::Time Dt = 1 / (f * scale);

            SUMO::TAZ::ID fromTAZ = sumoAdapter.toSumoTAZ(u);
            SUMO::TAZ::ID toTAZ   = sumoAdapter.toSumoTAZ(v);

            list<SUMO::TAZ::Source> sourcesList = sumoAdapter.toTAZEdges(fromTAZ).first;
            list<SUMO::TAZ::Sink>   sinksList   = sumoAdapter.toTAZEdges(toTAZ).second;

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

                Demand::Vehicle::ID id = nextID++;

                shared_ptr<Environment::Vehicle::Policy> policy =
                    make_shared<RandomPolicy>(id);

                demand.addVehicle(
                    id,
                    t,
                    sourceID,
                    sinkID,
                    policy
                );

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
