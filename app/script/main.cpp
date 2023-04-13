/**yaml
 * openapi: 3.0.0
 * info:
 *   title: DynamiNATOR API
 *   description: Web REST API to interact with the DynamiNATOR simulator.
 *   version: 0.0.4
 * servers:
 *   - url: http://localhost
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

#include <cstring>
#include <iostream>

#include "Com/RunFWSimulation.hpp"
#include "Com/Socket.hpp"
#include "REST/Server.hpp"
#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

REST::Server server;

void forwardToSimulator(Com::MessageRequest *m) {
    if(m == nullptr) {
        cout << "Content-type: text/html\n\n";
        cout << "Status: 400 Bad Request\n";
        return;
    }

    Com::Socket socket;
    socket.connect("127.0.0.1", 8001);
    socket.send(m);
    Com::Message         *res_m = socket.receive();
    Com::MessageResponse *res   = static_cast<Com::MessageResponse *>(res_m);
    res->handle(cout);
}

int main() {
    MESSAGE_REGISTER_MAIN(Com::RunFWSimulation::Response);

    /**yaml GET /hello
     * summary: Ping server.
     * tags:
     *   - Global
     */
    server.enroll("GET", "/hello", [](const REST::Server::Request &) {
        cout << "Content-type: text/html\n\n";
        cout << "Hello world!\n";
    });

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
     *       pattern: '^[a-zA-Z0-9\-]*$'
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
    server.enroll("POST", "/static/simulation/{id}", [](const REST::Server::Request &req) {
        const string &resourceID = req.pathVariables.at("id");
        string        netPath, tazPath, demandPath, outEdgesPath, outRoutesPath;
        try {
            netPath       = req.data.at("netPath");
            tazPath       = req.data.at("tazPath");
            demandPath    = req.data.at("demandPath");
            outEdgesPath  = req.data.at("outEdgesPath");
            outRoutesPath = req.data.at("outRoutesPath");
        } catch(const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        Com::MessageRequest *m = new Com::RunFWSimulation(resourceID, netPath, tazPath, demandPath, outEdgesPath, outRoutesPath);

        forwardToSimulator(m);
    });

    string method = getenv("REQUEST_METHOD");
    string url    = getenv("REQUEST_URI");

    server.route(method, url);

    return 0;
}
