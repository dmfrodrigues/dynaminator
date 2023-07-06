#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <fstream>
#include <ios>
#include <iostream>
#include <thread>

#include "Alg/ShortestPath/DijkstraMany.hpp"
#include "Dynamic/Demand/UniformDemandLoader.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Loader.hpp"
#include "Dynamic/Env/Vehicle.hpp"
#include "Dynamic/Policy/DoubleQLearner.hpp"
#include "Dynamic/Policy/PathPolicy.hpp"
#include "Dynamic/Policy/QLearner.hpp"
#include "Dynamic/Policy/RandomPolicy.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunctionDifference.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunctionGreedy.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunctionLocal.hpp"
#include "Log/ProgressLogger.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/Routes.hpp"
#include "data/VISUM/OFormatDemand.hpp"

using namespace std;
using Catch::Matchers::WithinRel;

extern string baseDir;
extern string benchmarkDir;

typedef chrono::steady_clock clk;

const size_t MATRIX_9_10_TOTAL_DEMAND_HOUR = 102731;

TEST_CASE("Dynamic - shortest path", "[dynamic][dynamic-sp][!benchmark]") {
    utils::stringify::stringify<float>::PRECISION  = 3;
    utils::stringify::stringify<double>::PRECISION = 3;

    Log::ProgressLoggerTableOStream logger;

    // logger << std::fixed << std::setprecision(6);

    // clang-format off
    Dynamic::Env::Loader<
        const SUMO::NetworkTAZs &,
        Dynamic::RewardFunction &
    > loader;
    // clang-format on

    // Environment
    SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.net.xml");
    SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.taz.xml");
    SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

    Dynamic::RewardFunction &rewardFunction = Dynamic::RewardFunctionGreedy::INSTANCE;

    Dynamic::Env::Env env = loader.load(sumo, rewardFunction);

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
    const double SCALE = 0.1;

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

    for(Dynamic::Time t = 0.0; t <= 3600.0; t += 1.0) {
        env.runUntil(t);

        SUMO::NetState::Timestep timestep = timestepLoader.load(env, loader.adapter, t);

        // clang-format off
        threads.emplace_back(
            [&netState](SUMO::NetState::Timestep ts) -> void {
                netState << ts;
            },
            timestep
        );
        // clang-format on

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

    const Dynamic::Env::Env &envConst = env;

    SUMO::Routes routes = routesLoader.load(envConst.getVehicles(), sumo.tazs, loader.adapter);

    routes.saveToFile(baseDir + "data/out/sp.rou.xml");
}

void dynamic(Dynamic::RewardFunction &rewardFunction) {
    utils::stringify::stringify<float>::PRECISION  = 3;
    utils::stringify::stringify<double>::PRECISION = 3;

    Log::ProgressLoggerTableOStream logger;

    // logger << std::fixed << std::setprecision(6);

    // clang-format off
    Dynamic::Env::Loader<
        const SUMO::NetworkTAZs &,
        Dynamic::RewardFunction &
    > loader;
    // clang-format on

    // Environment
    SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.net.xml");
    SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.taz.xml");
    SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

    Dynamic::Env::Env env = loader.load(sumo, rewardFunction);

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

    ofstream adapterDumpFile(baseDir + "data/out/adapter-dynamic.dump.txt");
    loader.adapter.dump(adapterDumpFile);

    const double HOUR2SEC = 3600.0;

    // Policy
    Dynamic::QLearner::Logger policyLogger(0.5);

    std::shared_ptr<Dynamic::Policy::Factory> policyFactory = make_shared<Dynamic::QLearner::Policy::Factory>(
        env,
        sumo,
        loader.adapter,
        0,
        policyLogger
    );

    // Demand
    // clang-format off
    std::vector<std::tuple<float, Dynamic::Time, Dynamic::Time>> demandSpecs = {
        {0.010,  0,  1},
        {0.100,  1, 10},
        {0.200, 10, 30},
        {0.300, 30, 43},
        {0.350, 43, 55},
        {0.400, 55, 70},
        {0.450, 70, 80},
        {0.500, 80, 90},
        {0.550, 90, 100},
        // {0.600, 100, 110},
    };
    // clang-format off

    // double END_SIMULATION = get<2>(demandSpecs.back()) * HOUR2SEC;
    double END_SIMULATION = 200 * HOUR2SEC;

    Dynamic::Vehicle::ID nextID = 0;
    
    for(auto &[scale, begin, end]: demandSpecs) {
        Dynamic::UniformDemandLoader demandLoader(scale, begin * HOUR2SEC, end * HOUR2SEC, *policyFactory, 0);
        auto p = demandLoader.load(staticDemand, env, loader.adapter, nextID);
        auto [demand, id] = p;
        nextID = id;
        env.addDemand(demand);
    }

    // Load demand into environment

    env.initializeTrafficLights(5);

    env.log(logger, 0, END_SIMULATION, 600, policyLogger);

    env.setDiscardVehicles(true);

    env.setDespawnTime(5*60);

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

    list<reference_wrapper<Dynamic::Env::Vehicle>> vehiclesList = env.getVehicles();

    // vector<reference_wrapper<Dynamic::Env::Vehicle>> vehicles(vehiclesList.begin(), vehiclesList.end());

    // sort(vehicles.begin(), vehicles.end(), [](const Dynamic::Env::Vehicle &a, const Dynamic::Env::Vehicle &b) -> bool {
    //     if(a.position.lane != b.position.lane) return a.position.lane < b.position.lane;
    //     return a.position.offset > b.position.offset;
    // });

    // for(Dynamic::Env::Vehicle &vehicle: vehicles) {
    //     if(vehicle.state != Dynamic::Env::Vehicle::State::STOPPED) continue;

    //     size_t indexInQueue = vehicle.position.lane.stopped.order_of({ref(vehicle), nullptr});

    //     const auto &[_, a] = vehicle.position.lane.stopped.at(indexInQueue);

    //     cerr << vehicle.id << ": ";
    //     cerr
    //         << "state: " << static_cast<int>(vehicle.state) << ", "
    //         << "pos: {"
    //         << vehicle.position.lane.idAsString() << ", "
    //         << vehicle.position.offset
    //         << "}, "
    //         << "speed: " << vehicle.speed << ", "
    //         << "queue: " << indexInQueue << "/" << vehicle.position.lane.stopped.size() << ", "
    //         << "nextLane: " << a->connection.toLane.idAsString() << ", "
    //         << endl;
    // }

    // Create edgedata file
    SUMO::EdgeData edgeData;

    SUMO::EdgeData::Interval &interval = edgeData.createInterval(0.0, END_SIMULATION);

    for(const Dynamic::Env::Vehicle &vehicle: vehiclesList) {
        if(vehicle.state != Dynamic::Env::Vehicle::State::STOPPED) continue;

        SUMO::Network::Edge::ID       edgeID = loader.adapter.toSumoEdge(vehicle.position.lane.edge.id);
        SUMO::Network::Edge::Lane::ID laneID = edgeID + "_" + to_string(vehicle.position.lane.index);

        if(!interval.hasEdge(edgeID)) {
            SUMO::EdgeData::Interval::Edge &edge = interval.createEdge(edgeID);
            edge.attributes.setAttribute("hasQueue", true);
        }

        SUMO::EdgeData::Interval::Edge &edge = interval.getEdge(edgeID);

        if(!edge.hasLane(laneID)) {
            SUMO::EdgeData::Interval::Edge::Lane &lane = edge.createLane(laneID);

            lane.attributes.setAttribute("queueSize", vehicle.position.lane.stopped.size());
        }
    }

    edgeData.saveToFile(benchmarkDir + "data/out/edgedata-ql-queues.xml");

    // policyFactory.dump();

    // // clang-format off
    // SUMO::Routes::Loader<
    //     const std::list<std::reference_wrapper<const Dynamic::Env::Vehicle>> &,
    //     const SUMO::TAZs &,
    //     const Dynamic::SUMOAdapter &
    // > routesLoader;
    // // clang-format on

    // const Dynamic::Env::Env &envConst = env;

    // SUMO::Routes routes = routesLoader.load(envConst.getVehicles(), sumo.tazs, loader.adapter);

    // routes.saveToFile(baseDir + "data/out/routes-ql.xml");
}

TEST_CASE("Dynamic - Q-learning", "[dynamic][q-learn][!benchmark]") {
    SECTION("w=0.0"){
        Dynamic::RewardFunction &rewardFunction = Dynamic::RewardFunctionGreedy::INSTANCE;
        dynamic(rewardFunction);
    }
    SECTION("w=0.1"){
        Dynamic::RewardFunctionDifference rewardFunction(0.1);
        dynamic(rewardFunction);
    }
    SECTION("w=0.5"){
        Dynamic::RewardFunctionDifference rewardFunction(0.5);
        dynamic(rewardFunction);
    }
    SECTION("w=0.75"){
        Dynamic::RewardFunctionDifference rewardFunction(0.75);
        dynamic(rewardFunction);
    }
    SECTION("w=0.9"){
        Dynamic::RewardFunctionDifference rewardFunction(0.9);
        dynamic(rewardFunction);
    }
    SECTION("w=1.0"){
        Dynamic::RewardFunctionDifference rewardFunction(1.0);
        dynamic(rewardFunction);
    }
    SECTION("w=1.1"){
        Dynamic::RewardFunctionDifference rewardFunction(1.1);
        dynamic(rewardFunction);
    }
}

TEST_CASE("Dynamic - Q-learners - small", "[dynamic][q-learn-small]"){
    utils::stringify::stringify<float>::PRECISION  = 3;
    utils::stringify::stringify<double>::PRECISION = 3;

    Log::ProgressLoggerTableOStream logger;

    // logger << std::fixed << std::setprecision(6);

    // clang-format off
    Dynamic::Env::Loader<
        const SUMO::NetworkTAZs &,
        Dynamic::RewardFunction &
    > loader;
    // clang-format on

    // Environment
    SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.net.xml");
    SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(benchmarkDir + "data/dynaminator-data/porto.taz.xml");
    SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

    Dynamic::Env::Env env = loader.load(sumo, Dynamic::RewardFunctionGreedy::INSTANCE);

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

    ofstream adapterDumpFile(baseDir + "data/out/adapter-dynamic.dump.txt");
    loader.adapter.dump(adapterDumpFile);

    const double HOUR2SEC = 3600.0;

    // Policy
    Dynamic::QLearner::Logger policyLogger(0.5);

    std::shared_ptr<Dynamic::Policy::Factory> policyFactory = make_shared<Dynamic::QLearner::Policy::Factory>(
        env,
        sumo,
        loader.adapter,
        0,
        policyLogger
    );

    // Demand
    // clang-format off
    std::vector<std::tuple<float, Dynamic::Time, Dynamic::Time>> demandSpecs = {
        {0.2,  0, 1},
    };
    // clang-format off

    double END_SIMULATION = get<2>(demandSpecs.back()) * HOUR2SEC;
    // double END_SIMULATION = 130 * HOUR2SEC;

    Dynamic::Vehicle::ID nextID = 0;
    
    for(auto &[scale, begin, end]: demandSpecs) {
        Dynamic::UniformDemandLoader demandLoader(scale, begin * HOUR2SEC, end * HOUR2SEC, *policyFactory, 0);
        auto p = demandLoader.load(staticDemand, env, loader.adapter, nextID);
        auto [demand, id] = p;
        nextID = id;
        env.addDemand(demand);
    }

    // Load demand into environment

    env.initializeTrafficLights(0);

    env.log(logger, 0, END_SIMULATION, 600, policyLogger);

    // env.setDespawnTime(5*60);

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

    list<reference_wrapper<Dynamic::Env::Vehicle>> vehiclesList = env.getVehicles();

    // vector<reference_wrapper<Dynamic::Env::Vehicle>> vehicles(vehiclesList.begin(), vehiclesList.end());

    // sort(vehicles.begin(), vehicles.end(), [](const Dynamic::Env::Vehicle &a, const Dynamic::Env::Vehicle &b) -> bool {
    //     if(a.position.lane != b.position.lane) return a.position.lane < b.position.lane;
    //     return a.position.offset > b.position.offset;
    // });

    // for(Dynamic::Env::Vehicle &vehicle: vehicles) {
    //     if(vehicle.state != Dynamic::Env::Vehicle::State::STOPPED) continue;

    //     size_t indexInQueue = vehicle.position.lane.stopped.order_of({ref(vehicle), nullptr});

    //     const auto &[_, a] = vehicle.position.lane.stopped.at(indexInQueue);

    //     cerr << vehicle.id << ": ";
    //     cerr
    //         << "state: " << static_cast<int>(vehicle.state) << ", "
    //         << "pos: {"
    //         << vehicle.position.lane.idAsString() << ", "
    //         << vehicle.position.offset
    //         << "}, "
    //         << "speed: " << vehicle.speed << ", "
    //         << "queue: " << indexInQueue << "/" << vehicle.position.lane.stopped.size() << ", "
    //         << "nextLane: " << a->connection.toLane.idAsString() << ", "
    //         << endl;
    // }

    // Create edgedata file
    SUMO::EdgeData edgeData;

    SUMO::EdgeData::Interval &interval = edgeData.createInterval(0.0, END_SIMULATION);

    for(const Dynamic::Env::Vehicle &vehicle: vehiclesList) {
        if(vehicle.state != Dynamic::Env::Vehicle::State::STOPPED) continue;

        SUMO::Network::Edge::ID       edgeID = loader.adapter.toSumoEdge(vehicle.position.lane.edge.id);
        SUMO::Network::Edge::Lane::ID laneID = edgeID + "_" + to_string(vehicle.position.lane.index);

        if(!interval.hasEdge(edgeID)) {
            SUMO::EdgeData::Interval::Edge &edge = interval.createEdge(edgeID);
            edge.attributes.setAttribute("hasQueue", true);
        }

        SUMO::EdgeData::Interval::Edge &edge = interval.getEdge(edgeID);

        if(!edge.hasLane(laneID)) {
            SUMO::EdgeData::Interval::Edge::Lane &lane = edge.createLane(laneID);

            lane.attributes.setAttribute("queueSize", vehicle.position.lane.stopped.size());
        }
    }

    edgeData.saveToFile(benchmarkDir + "data/out/edgedata-ql-queues.xml");

    // policyFactory.dump();

    // clang-format off
    SUMO::Routes::Loader<
        const std::list<std::reference_wrapper<const Dynamic::Env::Vehicle>> &,
        const SUMO::TAZs &,
        const Dynamic::SUMOAdapter &
    > routesLoader;
    // clang-format on

    const Dynamic::Env::Env &envConst = env;

    SUMO::Routes routes = routesLoader.load(envConst.getVehicles(), sumo.tazs, loader.adapter);

    routes.saveToFile(baseDir + "data/out/routes-ql.xml");
}
