/**yaml
 * openapi: 3.0.0
 * info:
 *   title: DynamiNATOR API
 *   description: Web REST API to interact with the DynamiNATOR simulator.
 *   version: 0.0.4
 * servers:
 *   - url: http://localhost:9000
 *
 * components:
 *   schemas:
 *     StaticSimulation:
 *       type: object
 *       required:
 *         - netPath
 *         - tazPath
 *         - demandPath
 *         - dstPath
 *       properties:
 *         netPath:
 *           description: Path of SUMO network file to use.
 *           type: string
 *           example: network/net.net.xml
 *         tazPath:
 *           description: Path of SUMO TAZ file to use.
 *           type: string
 *           example: network/taz.xml
 *         demandPath:
 *           description: Path of O-formatted OD matrix to use.
 *           type: string
 *           example: od/matrix.9.0.10.0.2.fma
 *         outEdgesPath:
 *           description: Path to which edges output will be printed.
 *           type: string
 *           example: out/edgedata-static.xml
 *         outRoutesPath:
 *           description: Path to which routes output will be printed.
 *           type: string
 *           example: out/routes-static.xml
 */

#include "Com/HTTPServer.hpp"

#include <exception>
#include <functional>
#include <future>
#include <ios>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "GlobalState.hpp"
#include "Log/ProgressLoggerJsonOStream.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/algos/ConjugateFrankWolfe.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/supply/BPRNetwork.hpp"

using namespace std;
using namespace std::placeholders;
using namespace Com;
using namespace httplib;

using json = nlohmann::json;

HTTPServer::HTTPServer(int port_):
    port(port_) {
    cerr << "Starting HTTP server" << endl;

    if(!svr.set_mount_point("/", "/var/www/html")) {
        throw runtime_error("Failed to setup HTTP server mount point");
    }

    svr.Get("/hello", Server::Handler(bind(&HTTPServer::helloGet, this, _1, _2)));

    svr.Post(R"(/static/simulation/([\w\-]+))", Server::Handler(bind(&HTTPServer::staticSimulationPost, this, _1, _2)));

    svr.Get(R"((/static/simulation/([\w\-]+))/join)", Server::Handler(bind(&HTTPServer::staticSimulationJoinGet, this, _1, _2)));

    cerr << "Started HTTP server" << endl;
}

void HTTPServer::loop() {
    if(!svr.listen("0.0.0.0", 80)) {
        throw runtime_error("HTTP server listen() failed");
    }

    cerr << "Closing HTTPserver" << endl;
}

/**yaml GET /hello
 * summary: Ping server.
 * tags:
 *   - Global
 */
void HTTPServer::helloGet(const httplib::Request &, httplib::Response &res) {
    res.set_content("Hello world!\n", "text/plain");
}

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
 *   - name: body
 *     in: body
 *     required: true
 *     description: Configuration of simulation to run.
 *     schema:
 *       $ref: '#/components/schemas/StaticSimulation'
 * responses:
 *   '200':
 *     description: Simulation executed successfully
 */
void HTTPServer::staticSimulationPost(const httplib::Request &req, httplib::Response &res) {
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
        utils::pipestream      *iosPtr   = nullptr;
        {
            lock_guard<mutex> lock(GlobalState::streams);
            auto [it, success] = GlobalState::streams->emplace(streamID, make_shared<utils::pipestream>());
            if(!success) {
                res.status = 400;
                res.set_content("Resource " + streamID + " already exists", "text/plain");
                return;
            }
            iosPtr = it->second.get();
        }
        utils::pipestream &ios = *iosPtr;

        shared_future<GlobalState::TaskReturn> *future;
        {
            lock_guard<mutex> lock(GlobalState::tasks);

            auto [it, success] = GlobalState::tasks->emplace(taskID, make_shared<shared_future<GlobalState::TaskReturn>>());

            if(!success) {
                res.status = 400;
                res.set_content("Resource " + taskID + " already exists", "text/plain");
                return;
            }

            future = it->second.get();
        }

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
                Log::ProgressLoggerJsonOStream loggerOStream(ios.o());
                Log::ProgressLogger           &logger = loggerOStream;

                // Supply
                SUMO::Network            sumoNetwork = SUMO::Network::loadFromFile(netPath);
                SUMO::TAZs               sumoTAZs    = SUMO::TAZ::loadFromFile(tazPath);
                auto                     t           = Static::BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
                Static::BPRNetwork      *network     = get<0>(t);
                const SumoAdapterStatic &adapter     = get<1>(t);

                // Demand
                VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(demandPath);
                Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, adapter);

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

                network->saveResultsToFile(x, adapter, outEdgesPath, outRoutesPath);

                ios.closeWrite();
                {
                    lock_guard<mutex> lock(GlobalState::streams);
                    GlobalState::streams->erase(streamID);
                }

                thread([taskID](){
                    lock_guard<mutex> lock(GlobalState::tasks);
                    GlobalState::tasks->erase(taskID);
                }).detach();

            } catch(const ios_base::failure &e){
                cerr << "Task " << taskID << " aborted" << endl;
                return { 400, "what(): "s + e.what() };
            } catch(const exception &e){
                cerr << "Task " << taskID << " aborted" << endl;
                return { 500, "what(): "s + e.what() };
            }

            cerr << "Task " << taskID << " finished" << endl;
            return { 200, "" };
        }));
        // clang-format on

        cerr << "Task " << taskID << " created" << endl;

        json resData = {
            {"log", streamID}};
        res.set_content(resData.dump(), "application/json");

    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
    }
}

/**yaml GET /static/simulation/{id}/join
 * summary: Wait for simulation to finish.
 * tags:
 *   - Static
 * parameters:
 *   - name: id
 *     in: path
 *     required: true
 *     description: ID of simulation
 *     schema:
 *       type: string
 *       pattern: '^[\w\-]+$'
 * responses:
 *   '200':
 *     description: Successfully waited for simulation
 */
void HTTPServer::staticSimulationJoinGet(const httplib::Request &req, httplib::Response &res) {
    // TODO: implement this.
    /*
    To implement this, I need:
    - To store the thread where the simulation is running
    - Protect the map of threads with a mutex
    - TO either join the thread, or have a condition variable to notify that operation is over.
    */
    const string &resourceID = req.matches[1];
    const string &taskID     = "task://" + resourceID;

    try {
        shared_ptr<shared_future<GlobalState::TaskReturn>> future;
        try {
            lock_guard<mutex> lock(GlobalState::tasks);
            future = GlobalState::tasks->at(taskID);
        } catch(const std::out_of_range &e) {
            res.status = 404;
            res.set_content("No such task " + taskID, "text/plain");
            return;
        }

        GlobalState::TaskReturn ret = future->get();

        res.status = ret.status;
        res.set_content(ret.content, ret.content_type);

    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
        cerr << "what(): " << e.what() << endl;
    }
}