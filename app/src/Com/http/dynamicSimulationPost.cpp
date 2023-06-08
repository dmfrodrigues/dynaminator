#include <nlohmann/json.hpp>

#include "Com/HTTPServer.hpp"
#include "Dynamic/Environment.hpp"
#include "Dynamic/Environment_Loader.hpp"
#include "GlobalState.hpp"

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
    json data = json::parse(req.body);

    const string &resourceID = req.matches[0];

    string        netPath, tazPath, demandPath;
    Dynamic::Time beginTime, endTime;
    try {
        netPath    = data.at("netPath");
        tazPath    = data.at("tazPath");
        demandPath = data.at("demandPath");
        beginTime  = stringify<Dynamic::Time>::fromString(data.at("beginTime"));
        endTime    = stringify<Dynamic::Time>::fromString(data.at("endTime"));
    } catch(const json::out_of_range &e) {
        res.status = 400;
        return;
    }

    Dynamic::Time stepTime = 1.0;
    if(data.contains("step")) {
        stepTime = stringify<Dynamic::Time>::fromString(data.at("step"));
    }

    optional<string> netstatePath;
    if(data.contains("netstatePath")) {
        netstatePath = data.at("netstatePath");
    }

    try {
        GlobalState::ResourceID taskID = "task://"s + resourceID;

        Dynamic::Environment::Loader<const SUMO::NetworkTAZs &> loader;

        // Supply
        SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(netPath);
        SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(tazPath);
        SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

        Dynamic::Environment *env = loader.load(sumo);

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
        Dynamic::Demand::UniformLoader demandLoader(1.0, 0.0, 3600.0);
        Dynamic::Demand                demand = demandLoader.load(staticDemand, *env, loader.adapter);

    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
    }
}
