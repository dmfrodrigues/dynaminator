#include <cstdlib>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "Com/HTTPServer.hpp"
#include "GlobalState.hpp"
#include "Log/ProgressLoggerJsonOStream.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/ConjugateFrankWolfe.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/algos/IterativeEquilibration.hpp"
#include "Static/supply/BPRConvexNetwork.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "Static/supply/BPRNotConvexNetwork.hpp"
#include "data/SUMO/EdgeData.hpp"
#include "data/SUMO/NetworkTAZ.hpp"
#include "data/SUMO/Routes.hpp"
#include "utils/invertMap.hpp"
#include "utils/require_env.hpp"

using namespace std;
using namespace Com;
using namespace utils::stringify;

using json = nlohmann::json;

enum class StaticSimulationType : int {
    CONVEX,
    NONCONVEX
};

namespace utils::stringify {
template<>
class stringify<StaticSimulationType> {
   public:
    static StaticSimulationType fromString(const string &s);

    static string toString(const StaticSimulationType &t);
};
}  // namespace utils::stringify

// clang-format off
const unordered_map<string, StaticSimulationType> str2staticSimulationType = {
    {"convex"   , StaticSimulationType::CONVEX      },
    {"nonconvex", StaticSimulationType::NONCONVEX   }
};
const unordered_map<StaticSimulationType, string> staticSimulationType2str = utils::invertMap(str2staticSimulationType);
// clang-format on

StaticSimulationType stringify<StaticSimulationType>::fromString(const string &s) {
    return str2staticSimulationType.at(s);
}

string stringify<StaticSimulationType>::toString(const StaticSimulationType &t) {
    return staticSimulationType2str.at(t);
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

    StaticSimulationType type = StaticSimulationType::NONCONVEX;
    if(data.contains("type")) {
        type = stringify<StaticSimulationType>::fromString(data.at("type"));
    }

    try {
        GlobalState::ResourceID taskID = "task://"s + resourceID;

        // Create stringstream resource
        GlobalState::ResourceID streamID = "stream://"s + resourceID;
        utils::pipestream      &ios      = GlobalState::streams.create(streamID);

        // Create task
        // clang-format off
        shared_future<GlobalState::TaskReturn> &future = GlobalState::tasks.create(
            taskID,
            [
                netPath,
                tazPath,
                demandPath,
                type,
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

                    // Demand
                    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(demandPath);
                    Static::Demand::Loader<const VISUM::OFormatDemand &, const Static::SUMOAdapter &> demandLoader;

                    // Model
                    Static::BPRNetwork::Loader<SUMO::NetworkTAZs> *loader = nullptr;
                    switch(type) {
                        case StaticSimulationType::CONVEX:
                            loader = new Static::BPRNetwork::Loader<SUMO::NetworkTAZs>();
                            break;
                        case StaticSimulationType::NONCONVEX:
                            loader = new Static::BPRNotConvexNetwork::Loader<SUMO::NetworkTAZs>();
                            break;
                        default:
                            throw out_of_range("Invalid static simulation type");
                    }

                    Static::BPRNetwork *network = loader->load(sumo);

                    Static::Demand       demand  = demandLoader.load(oDemand, loader->adapter);

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

                    Static::Solution x;
                    switch(type){
                        case StaticSimulationType::CONVEX: {
                            // Frank-Wolfe
                            Static::ConjugateFrankWolfe fw(aon, solver, logger);
                            fw.setStopCriteria(1.0);
                            x = fw.solve(*network, demand, x0);

                            break;
                        }
                        case StaticSimulationType::NONCONVEX: {
                            ofstream                        ofsNull("/dev/null");
                            Log::ProgressLoggerTableOStream loggerNull(ofsNull);

                            // Iterative Equilibration
                            Static::ConjugateFrankWolfe fw(aon, solver, loggerNull);

                            fw.setStopCriteria(0.0);
                            fw.setIterations(5);

                            Static::IterativeEquilibration<Static::BPRNotConvexNetwork, Static::ConjugateFrankWolfe> ie(fw, logger);
                            ie.setIterations(10);

                            Static::BPRNotConvexNetwork *networkNotConvex = dynamic_cast<Static::BPRNotConvexNetwork *>(network);
                            if(networkNotConvex == nullptr)
                                throw logic_error("Network is not BPRNotConvexNetwork");

                            x = ie.solve(*networkNotConvex, demand, x0);

                            break;
                        }
                    }

                    // Save edgeData
                    // clang-format off
                    SUMO::EdgeData::Loader<
                        const SUMO::NetworkTAZs &,
                        const Static::BPRNetwork &,
                        const Static::Solution &,
                        const Static::SUMOAdapter &
                    > edgeDataLoader;
                    // clang-format on
                    SUMO::EdgeData edgeData = edgeDataLoader.load(sumo, *network, x, loader->adapter);
                    edgeData.saveToFile(outEdgesPath);

                    // Save routes
                    // clang-format off
                    SUMO::Routes::Loader<
                        const Static::Network &,
                        const Static::Solution &,
                        const Static::SUMOAdapter &
                    > routesLoader;
                    // clang-format on
                    SUMO::Routes routes = routesLoader.load(*network, x, loader->adapter);
                    routes.saveToFile(outRoutesPath);

                    delete network;
                    delete loader;

                    ios.closeWrite();
                    GlobalState::streams.erase(streamID);

                } catch(const GlobalState::ResourceException &e) {
                    cerr << "Task " << taskID << " aborted, what(): " << e.what() << endl;
                    return {400, "what(): "s + e.what()};
                } catch(const ios_base::failure &e) {
                    cerr << "Task " << taskID << " aborted, what(): " << e.what() << endl;
                    return {400, "what(): "s + e.what()};
                }

                return {200, ""};
            }
        );
        // clang-format on

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
