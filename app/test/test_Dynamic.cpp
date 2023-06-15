#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <thread>

#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Demand/UniformDemandLoader.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Loader.hpp"
#include "Dynamic/Policy/PathPolicy.hpp"
#include "Dynamic/Policy/RandomPolicy.hpp"
#include "Log/ProgressLogger.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Static/Demand.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/VISUM/OFormatDemand.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

extern string baseDir;
extern string benchmarkDir;

typedef chrono::steady_clock clk;

const size_t MATRIX_9_10_TOTAL_DEMAND_HOUR = 102731;

TEST_CASE("Dynamic environment", "[dynamic][!benchmark]") {
    utils::stringify::stringify<float>::PRECISION  = 3;
    utils::stringify::stringify<double>::PRECISION = 3;

    Log::ProgressLoggerTableOStream logger;

    logger << std::fixed << std::setprecision(6);

    Dynamic::Env::Loader<const SUMO::NetworkTAZs &> loader;

    // Environment
    SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.net.xml");
    SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.taz.xml");
    SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

    Dynamic::Env::Env env = loader.load(sumo);

    // loader.adapter.dump();

    // Demand
    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(benchmarkDir + "data/dynaminator-data/matrix.9.0.10.0.2.fma");
    // clang-format off
    Static::Demand::Loader<
        const VISUM::OFormatDemand &,
        const Static::SUMOAdapter &
    > staticDemandLoader;
    // clang-format on
    Static::Demand staticDemand = staticDemandLoader.load(
        oDemand,
        (Static::SUMOAdapter &)loader.adapter
    );

    Alg::Graph G = env.toGraph();

    vector<Dynamic::Env::Node> startNodes;
    for(const Static::Network::Node &u: staticDemand.getStartNodes()) {
        SUMO::TAZ::ID           fromTAZ     = loader.adapter.toSumoTAZ(u);
        list<SUMO::TAZ::Source> sourcesList = loader.adapter.toTAZEdges(fromTAZ).first;
        for(const SUMO::TAZ::Source &source: sourcesList)
            if(source.weight > 0.0) {
                Dynamic::Env::Edge::ID edgeID = loader.adapter.toEdge(source.id);
                Dynamic::Env::Node     nodeID = env.getEdge(edgeID).u;
                startNodes.push_back(nodeID);
            }
    }

    Alg::ShortestPath::DijkstraMany sp;
    sp.solve(G, startNodes);

    Dynamic::PathPolicy::ShortestPathFactory policyFactory(sp, Catch::getSeed());

    Dynamic::UniformDemandLoader demandLoader(1.0, 0.0, 3600.0, policyFactory, Catch::getSeed());
    Dynamic::Demand              demand = demandLoader.load(staticDemand, env, loader.adapter);

    REQUIRE(MATRIX_9_10_TOTAL_DEMAND_HOUR == demand.getVehicles().size());

    // Load demand into environment
    env.addDemand(demand);

    logger << Log::ProgressLogger::Elapsed(0)
           << Log::ProgressLogger::Progress(0)
           << Log::ProgressLogger::ETA(1)
           << Log::ProgressLogger::StartText()
           << "t"
           << "\t"
           << "#vehicles"
           << "\t"
           << "queueSize"
           << Log::ProgressLogger::EndMessage();

    env.log(logger, 0, 3600, 30);

    SUMO::NetState netState(baseDir + "data/out/netstate.xml");

    list<thread> threads;
    const size_t MAX_NUMBER_THREADS = 64;

    // Run simulation
    // for(Dynamic::Time t = 0.0; t <= 3600.0; t += 1.0) {
    //     env.runUntil(t);

    //     threads.emplace_back([&loader, &netState](Dynamic::Env::Env env, Dynamic::Time t) -> void {
    //         // clang-format off
    //         SUMO::NetState::Timestep::Loader<
    //             Dynamic::Env::Env &,
    //             const Dynamic::SUMOAdapter &,
    //             Dynamic::Time
    //         > timestepLoader;
    //         // clang-format on

    //         SUMO::NetState::Timestep timestep = timestepLoader.load(env, loader.adapter, t);

    //         netState << timestep;
    //     },
    //                          env, t);

    //     while(threads.size() > MAX_NUMBER_THREADS) {
    //         threads.front().join();
    //         threads.pop_front();
    //     }
    // }

    // while(!threads.empty()) {
    //     threads.front().join();
    //     threads.pop_front();
    // }

    env.runUntil(3600.0);
}
