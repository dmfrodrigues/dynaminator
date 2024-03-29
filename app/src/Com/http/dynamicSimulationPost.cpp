#include <spdlog/spdlog.h>

#include <future>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "Com/HTTPServer.hpp"
#include "Dynamic/Demand/Demand.hpp"
#include "Dynamic/Demand/UniformDemandLoader.hpp"
#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Loader.hpp"
#include "Dynamic/Policy/RandomPolicy.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunction.hpp"
#include "Dynamic/Policy/RewardFunction/RewardFunctionGreedy.hpp"
#include "GlobalState.hpp"
#include "Log/ProgressLoggerJsonOStream.hpp"
#include "data/SUMO/NetState.hpp"
#include "utils/require_env.hpp"

using namespace std;
using namespace Com;

using namespace utils::stringify;

using json = nlohmann::json;

/**yaml POST /dynamic/simulation/{id}
 * summary: Run dynamic simulation.
 * tags:
 *   - Dynamic
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
 *         $ref: '#/components/schemas/DynamicSimulation'
 * responses:
 *   '200':
 *     description: Simulation executed successfully
 */
void HTTPServer::dynamicSimulationPost(const httplib::Request &req, httplib::Response &res) {
    const string WS_HOST = utils::require_env("WS_HOST");

    json data = json::parse(req.body);

    const string &resourceID = req.matches[0];

    string        netPath, tazPath, demandPath;
    Dynamic::Time beginTime, endTime;
    try {
        netPath    = data.at("netPath");
        tazPath    = data.at("tazPath");
        demandPath = data.at("demandPath");
        beginTime  = data.at("begin");
        endTime    = data.at("end");

        if(endTime < beginTime)
            throw logic_error("endTime < beginTime");
    } catch(const json::out_of_range &e) {
        res.status = 400;
        return;
    } catch(const logic_error &e) {
        res.status = 400;
        res.set_content("what(): "s + e.what(), "text/plain");
        return;
    }

    Dynamic::Time stepTime = 1.0;
    if(data.contains("step")) {
        stepTime = data.at("step");
    }

    optional<string> netstatePath;
    if(data.contains("netstatePath")) {
        netstatePath = data.at("netstatePath");
    }

    try {
        GlobalState::ResourceID taskID = "task://"s + resourceID;

        GlobalState::ResourceID streamID = "stream://"s + resourceID;
        utils::pipestream      &ios      = GlobalState::streams.create(streamID);

        GlobalState::tasks.create(
            taskID,
            [netPath,
             tazPath,
             demandPath,
             beginTime,
             endTime,
             taskID,
             streamID,
             &ios,
             netstatePath,
             stepTime]() -> GlobalState::TaskReturn {
                try {
                    Log::ProgressLoggerJsonOStream logger(ios.o());

                    // Supply
                    shared_ptr<SUMO::Network> sumoNetwork = SUMO::Network::loadFromFile(netPath);
                    SUMO::TAZs                sumoTAZs    = SUMO::TAZ::loadFromFile(tazPath);
                    SUMO::NetworkTAZs         sumo{*sumoNetwork, sumoTAZs};

                    // clang-format off
                    Dynamic::Env::Loader<
                        const SUMO::NetworkTAZs &,
                        Dynamic::RewardFunction &
                    > loader;
                    // clang-format on

                    shared_ptr<Dynamic::Env::Env> env = loader.load(sumo, Dynamic::RewardFunctionGreedy::INSTANCE);

                    // Demand
                    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(demandPath);
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
                    Dynamic::RandomPolicy::Factory policyFactory;
                    Dynamic::UniformDemandLoader   demandLoader(1.0, beginTime, endTime, policyFactory);
                    Dynamic::Demand                demand = demandLoader.load(staticDemand, *env, loader.adapter).first;

                    // Simulation
                    env->addDemand(demand);

                    Dynamic::Time delta = (endTime - beginTime) / 100;
                    env->log(logger, beginTime, endTime, delta);

                    optional<SUMO::NetState> netstate;
                    if(netstatePath.has_value()) {
                        size_t numberDumps = (size_t)((endTime - beginTime) / stepTime);

                        netstate.emplace(netstatePath.value(), ios_base::out);

                        env->dump(netstate.value(), loader.adapter, beginTime, endTime, numberDumps);
                    }

                    // TODO: run simulation

                    ios.closeWrite();
                    GlobalState::streams.erase(streamID);

                } catch(const exception &e) {
                    spdlog::error(
                        "Task {} aborted, what(): {}",
                        taskID,
                        e.what()
                    );
                    return {500, "what(): "s + e.what()};
                }

                spdlog::info("Task {} finished", taskID);
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
