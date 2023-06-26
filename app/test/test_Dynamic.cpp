#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <thread>

#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Demand/UniformDemandLoader.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Loader.hpp"
#include "Dynamic/Policy/DoubleQLearner.hpp"
#include "Dynamic/Policy/PathPolicy.hpp"
#include "Dynamic/Policy/Policy.hpp"
#include "Dynamic/Policy/QLearner.hpp"
#include "Dynamic/Policy/RandomPolicy.hpp"
#include "Log/ProgressLogger.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/Routes.hpp"
#include "data/VISUM/OFormatDemand.hpp"

using namespace std;
using Catch::Matchers::WithinRel;

extern string baseDir;
extern string benchmarkDir;

typedef chrono::steady_clock clk;

const size_t MATRIX_9_10_TOTAL_DEMAND_HOUR = 102731;

TEST_CASE("Dynamic environment", "[dynamic][!benchmark]") {
    utils::stringify::stringify<float>::PRECISION  = 3;
    utils::stringify::stringify<double>::PRECISION = 3;

    Log::ProgressLoggerTableOStream logger;

    // logger << std::fixed << std::setprecision(6);

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

    SECTION("Shortest path") {
        // loader.adapter.dump();

        // Policy
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

        Dynamic::PathPolicy::ShortestPathFactory policyFactory(env, 0);

        // Demand
        const double SCALE = 1.0;

        Dynamic::UniformDemandLoader demandLoader(SCALE, 0.0, 3600.0, policyFactory, 0);
        Dynamic::Demand              demand = demandLoader.load(staticDemand, env, loader.adapter).first;

        REQUIRE_THAT(demand.getVehicles().size(), WithinRel(MATRIX_9_10_TOTAL_DEMAND_HOUR * SCALE, 1e-2));

        // Load demand into environment
        env.addDemand(demand);

        env.initializeTrafficLights(0);

        env.log(logger, 0, 3600, 30);

        SUMO::NetState netState(baseDir + "data/out/netstate.xml");

        list<thread> threads;
        const size_t MAX_NUMBER_THREADS = 64;

        // Run simulation
        // clang-format off
        SUMO::NetState::Timestep::Loader<
            Dynamic::Env::Env &,
            const Dynamic::SUMOAdapter &,
            Dynamic::Time
        > timestepLoader;
        // clang-format on

        for(Dynamic::Time t = 0.0; t <= 3600.0; t += 10.0) {
            env.runUntil(t);

            // SUMO::NetState::Timestep timestep = timestepLoader.load(env, loader.adapter, t);

            // // clang-format off
            // threads.emplace_back(
            //     [&netState](SUMO::NetState::Timestep timestep) -> void {
            //         netState << timestep;
            //     },
            //     timestep
            // );
            // // clang-format on

            while(threads.size() > MAX_NUMBER_THREADS) {
                threads.front().join();
                threads.pop_front();
            }
        }

        while(!threads.empty()) {
            threads.front().join();
            threads.pop_front();
        }

        env.runUntil(3600.0);

        // clang-format off
        SUMO::Routes::Loader<
            const std::list<std::reference_wrapper<const Dynamic::Env::Vehicle>> &,
            const SUMO::TAZs &,
            const Dynamic::SUMOAdapter &
        > routesLoader;
        // clang-format on

        SUMO::Routes routes = routesLoader.load(env.getVehicles(), sumo.tazs, loader.adapter);

        routes.saveToFile(baseDir + "data/out/sp.rou.xml");
    }

    SECTION("Q-learners") {
        // ofstream adapterDumpFile(baseDir + "data/out/adapter-dynamic.dump.txt");
        // loader.adapter.dump(adapterDumpFile);

        const double HOUR2SEC = 3600.0;

        double END_SIMULATION = 50.0 * HOUR2SEC;

        // Policy
        Dynamic::QLearner::Logger policyLogger(0.5);

        std::shared_ptr<Dynamic::Policy::Factory> policyFactory;

        SECTION("Simple Q-learner") {
            policyFactory = make_shared<Dynamic::QLearner::Policy::Factory>(
                env,
                sumo,
                loader.adapter,
                0,
                policyLogger
            );
        }

        SECTION("Double Q-learner") {
            policyFactory = make_shared<Dynamic::DoubleQLearner::PolicyFactory>(
                env,
                sumo,
                loader.adapter,
                0,
                policyLogger
            );
        }

        // Demand
        vector<pair<Dynamic::Demand, Dynamic::Vehicle::ID>> demands;
        {
            Dynamic::UniformDemandLoader demandLoader(0.1, 0, HOUR2SEC * 1, *policyFactory, 0);
            demands.push_back(demandLoader.load(staticDemand, env, loader.adapter));
        }
        {
            Dynamic::UniformDemandLoader demandLoader(0.2, HOUR2SEC * 1, HOUR2SEC * 2, *policyFactory, 0);
            demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        }
        {
            Dynamic::UniformDemandLoader demandLoader(0.3, HOUR2SEC * 2, HOUR2SEC * 3, *policyFactory, 0);
            demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        }
        {
            Dynamic::UniformDemandLoader demandLoader(0.4, HOUR2SEC * 3, HOUR2SEC * 4, *policyFactory, 0);
            demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        }
        {
            Dynamic::UniformDemandLoader demandLoader(0.5, HOUR2SEC * 4, END_SIMULATION, *policyFactory, 0);
            demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.6, 35000, 50000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.7, 18000, 50000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.75, 15000, 30000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.8, 30000, 50000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.85, 30000, 50000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }
        // {
        //     Dynamic::UniformDemandLoader demandLoader(0.9, 45000, 50000, policyFactory, 0);
        //     demands.push_back(demandLoader.load(staticDemand, env, loader.adapter, demands.back().second));
        // }

        for(auto &[demand, _]: demands)
            env.addDemand(demand);

        // Load demand into environment

        env.initializeTrafficLights(0);

        env.log(logger, 0, END_SIMULATION, 900, policyLogger);

        env.setDiscardVehicles(true);

        // SUMO::NetState netState(baseDir + "data/out/netstate.xml");

        // list<thread> threads;
        // const size_t MAX_NUMBER_THREADS = 64;

        // Run simulation
        // // clang-format off
        // SUMO::NetState::Timestep::Loader<
        //     Dynamic::Env::Env &,
        //     const Dynamic::SUMOAdapter &,
        //     Dynamic::Time
        // > timestepLoader;
        // // clang-format on

        // for(Dynamic::Time t = 0.0; t <= END_SIMULATION; t += 1.0) {
        //     env.runUntil(t);

        //     SUMO::NetState::Timestep timestep = timestepLoader.load(env, loader.adapter, t);

        //     // clang-format off
        //     threads.emplace_back(
        //         [&netState](SUMO::NetState::Timestep timestep) -> void {
        //             netState << timestep;
        //         },
        //         timestep
        //     );
        //     // clang-format on

        //     while(threads.size() > MAX_NUMBER_THREADS) {
        //         threads.front().join();
        //         threads.pop_front();
        //     }
        // }

        // while(!threads.empty()) {
        //     threads.front().join();
        //     threads.pop_front();
        // }

        env.runUntil(END_SIMULATION);

        // policyFactory.dump();

        // // clang-format off
        // SUMO::Routes::Loader<
        //     const std::list<std::reference_wrapper<const Dynamic::Env::Vehicle>> &,
        //     const SUMO::TAZs &,
        //     const Dynamic::SUMOAdapter &
        // > routesLoader;
        // // clang-format on

        // SUMO::Routes routes = routesLoader.load(env.getVehicles(), sumo.tazs, loader.adapter);

        // routes.saveToFile(baseDir + "data/out/routes-ql.xml");
    }
}
