/**yaml
 * openapi: 3.0.0
 * info:
 *   title: DynamiNATOR API
 *   description: Web REST API to interact with the DynamiNATOR simulator.
 *   version: 0.0.3
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
 *           example: od/matrix.8.0.9.0.1.fma
 *         dstPath:
 *           description: Path to which output will be printed.
 *           type: string
 *           example: out/static-bpr1-demand1.xml
 */

#include <cstring>
#include <iostream>

#include "network/RunFWSimulation.hpp"
#include "network/Socket.hpp"
#include "script/Server.hpp"
#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

Server server;

void forwardToSimulator(MessageRequest *m){
    if(m == nullptr){
        cout << "Content-type: text/html\n\n";
        cout << "Status: 400 Bad Request\n";
        return;
    }

    Socket socket;
    socket.connect("127.0.0.1", 8001);
    socket.send(m);
    Message *res_m = socket.receive();
    MessageResponse *res = static_cast<MessageResponse*>(res_m);
    res->handle(cout);
}

int main() {
    MESSAGE_REGISTER_MAIN(RunFWSimulation::Response);

    /**yaml GET /hello
     * summary: Ping server.
     * tags:
     *   - Global
     */
    server.enroll("GET", "/hello", [](const Server::Request &) {
        cout << "Content-type: text/html\n\n";
        cout << "Hello world!\n";
    });

    /**yaml POST /static/simulation
     * summary: Run static simulation.
     * tags:
     *   - Static
     * consumes:
     *   - application/json
     * parameters:
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
    server.enroll("POST", "/static/simulation", [](const Server::Request &req) {
        string netPath, tazPath, demandPath, dstPath;
        try {
            netPath = req.data.at("netPath");
            tazPath = req.data.at("tazPath");
            demandPath = req.data.at("demandPath");
            dstPath = req.data.at("dstPath");
        } catch (const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = new RunFWSimulation(netPath, tazPath, demandPath, dstPath);

        forwardToSimulator(m);
    });

    string method = getenv("REQUEST_METHOD");
    string url = getenv("REQUEST_URI");

    server.route(method, url);

    return 0;
}
