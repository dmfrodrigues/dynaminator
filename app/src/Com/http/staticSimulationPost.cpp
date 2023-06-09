#include <cstdlib>
#include <nlohmann/json.hpp>

#include "Com/HTTPServer.hpp"
#include "GlobalState.hpp"
#include "Log/ProgressLoggerJsonOStream.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/algos/ConjugateFrankWolfe.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/Routes.hpp"
#include "utils/require_env.hpp"

using namespace std;
using namespace Com;

using json = nlohmann::json;

/**yaml POST /static/simulation/{id}
 * summary: Run static simulation.
 * tags:
 *   - Static
 * consumes:
 *   - application/json
 * parameters:
 *   - name: id
 *     in: path
 *     required: true
 *     description: ID of simulation
 *     schema:
 *       type: string
 *       pattern: '^[\w\-]+$'
 * requestBody:
 *   description: Configuration of simulation to run.
 *   content:
 *     application/json:
 *       schema:
 *         $ref: '#/components/schemas/StaticSimulation'
 * responses:
 *   '200':
 *     description: Simulation executed successfully
 */
void HTTPServer::staticSimulationPost(const httplib::Request &req, httplib::Response &res) {
    const string WS_HOST = utils::require_env("WS_HOST");

    json data = json::parse(req.body);

    const string &resourceID = req.matches[0];

    string netPath, tazPath, demandPath, outEdgesPath, outRoutesPath;
    try {
        netPath       = data.at("netPath");
        tazPath       = data.at("tazPath");
        demandPath    = data.at("demandPath");
        outEdgesPath  = data.at("outEdgesPath");
        outRoutesPath = data.at("outRoutesPath");
    } catch(const json::out_of_range &e) {
        res.status = 400;
        return;
    }

    try {
        GlobalState::ResourceID taskID = "task://"s + resourceID;

        // Create stringstream resource
        GlobalState::ResourceID streamID = "stream://"s + resourceID;
        utils::pipestream &ios = GlobalState::streams.create(streamID);

        shared_future<GlobalState::TaskReturn> *future;

        lock_guard<mutex> lock(GlobalState::tasks);

        auto [it, success] = GlobalState::tasks->emplace(taskID, make_shared<shared_future<GlobalState::TaskReturn>>());

        if(!success) {
            res.status = 400;
            res.set_content("Resource " + taskID + " already exists", "text/plain");
            return;
        }

        future = it->second.get();

        // clang-format off
        *future = shared_future<GlobalState::TaskReturn>(async(launch::async, [
            netPath,
            tazPath,
            demandPath,
            outEdgesPath,
            outRoutesPath,
            taskID,
            streamID,
            &ios
        ]() -> GlobalState::TaskReturn {
            try {
                Log::ProgressLoggerJsonOStream logger(ios.o());

                // Supply
                SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(netPath);
                SUMO::TAZs    sumoTAZs    = SUMO::TAZ::loadFromFile(tazPath);
                SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

                Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs> loader;
                Static::BPRNetwork *network = loader.load(sumo);

                // Demand
                VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(demandPath);
                Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;
                Static::Demand       demand  = demandLoader.load(oDemand, loader.adapter);

                // Solve

                // All or Nothing
                Static::DijkstraAoN aon;
                Static::Solution    x0 = aon.solve(*network, demand);

                // Solver
                Opt::QuadraticSolver      innerSolver;
                Opt::QuadraticGuessSolver solver(
                    innerSolver,
                    0.5,
                    0.2,
                    0.845,
                    0.365
                );
                solver.setStopCriteria(0.01);

                // Frank-Wolfe
                Static::ConjugateFrankWolfe fw(aon, solver, logger);
                fw.setStopCriteria(1.0);
                Static::Solution x = fw.solve(*network, demand, x0);

                // Save edgeData
                // clang-format off
                SUMO::EdgeData::Loader<
                    const SUMO::NetworkTAZs &,
                    const Static::BPRNetwork &,
                    const Static::Solution &,
                    const Static::SUMOAdapter &
                > edgeDataLoader;
                // clang-format on
                SUMO::EdgeData edgeData = edgeDataLoader.load(sumo, *network, x, loader.adapter);
                edgeData.saveToFile(outEdgesPath);

                // Save routes
                // clang-format off
                SUMO::Routes::Loader<
                    const Static::Network &,
                    const Static::Solution &,
                    const Static::SUMOAdapter &
                > routesLoader;
                // clang-format on
                SUMO::Routes routes = routesLoader.load(*network, x, loader.adapter);
                routes.saveToFile(outRoutesPath);

                ios.closeWrite();
                GlobalState::streams.erase(streamID);

                thread([taskID]() {
                    lock_guard<mutex> lock(GlobalState::tasks);
                    GlobalState::tasks->erase(taskID);
                }).detach();

            } catch(const GlobalState::ResourceException &e) {
                cerr << "Task " << taskID << " aborted, what(): " << e.what() << endl;
                return {400, "what(): "s + e.what()};
            } catch(const ios_base::failure &e) {
                cerr << "Task " << taskID << " aborted, what(): " << e.what() << endl;
                return {400, "what(): "s + e.what()};
            } catch(const exception &e) {
                cerr << "Task " << taskID << " aborted, what(): " << e.what() << endl;
                return {500, "what(): "s + e.what()};
            }

            cerr << "Task " << taskID << " finished" << endl;
            return {200, ""};
        }));
        // clang-format on

        cerr << "Task " << taskID << " created" << endl;

        // clang-format off
        json resData = {
            {"log", {
                {"resourceID", resourceID},
                {"url", "ws://" + WS_HOST + resourceID + "/log"}
            }}
        };
        // clang-format on
        res.set_content(resData.dump(), "application/json");

    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
    }
}
