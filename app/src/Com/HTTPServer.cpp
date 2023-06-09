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
 *         type:
 *           description: Type of static simulation to run.
 *           type: string
 *           enum:
 *             - convex
 *             - nonconvex
 *           example: nonconvex
 *           default: nonconvex
 *         outEdgesPath:
 *           description: Path to which edges output will be printed.
 *           type: string
 *           example: out/edgedata-static.xml
 *         outRoutesPath:
 *           description: Path to which routes output will be printed.
 *           type: string
 *           example: out/routes-static.xml
 *     DynamicSimulation:
 *       type: object
 *       required:
 *         - netPath
 *         - tazPath
 *         - demandPath
 *         - begin
 *         - end
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
 *         begin:
 *           description: Beginning time of simulation (in seconds).
 *           type: float
 *           example: 0
 *         end:
 *           description: End time of simulation (in seconds).
 *           type: float
 *           example: 3600
 *         step:
 *           description: Time step of simulation (in seconds). Only relevant if creating netstate output.
 *           type: float
 *           example: 1
 *           default: 1
 *         netstatePath:
 *           description: Path to which netstate output will be printed.
 *           type: string
 *           example: out/netstate.xml
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

    // Ping
    svr.Get(
        "/hello",
        Server::Handler(bind(&HTTPServer::helloGet, this, _1, _2))
    );

    // /static/simulation
    svr.Post(
        R"(/static/simulation/([\w\-]+))",
        Server::Handler(bind(&HTTPServer::staticSimulationPost, this, _1, _2))
    );

    svr.Get(
        R"((/static/simulation/([\w\-]+))/join)",
        Server::Handler(bind(&HTTPServer::staticSimulationJoinGet, this, _1, _2))
    );

    // /dynamic/simulation
    svr.Post(
        R"(/dynamic/simulation/([\w\-]+))",
        Server::Handler(bind(&HTTPServer::dynamicSimulationPost, this, _1, _2))
    );

    cerr << "Started HTTP server" << endl;
}

void HTTPServer::loop() {
    if(!svr.listen("0.0.0.0", 80)) {
        throw runtime_error("HTTP server listen() failed");
    }

    cerr << "Closing HTTPserver" << endl;
}
