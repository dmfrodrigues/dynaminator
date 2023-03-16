/**yaml
 * openapi: 3.0.0
 * info:
 *   title: DynamiNATOR API
 *   description: Web REST API to interact with the DynamiNATOR simulator.
 *   version: 0.0.1
 * servers:
 *   - url: http://localhost
 * 
 * components:
 *   schemas:
 *     StaticNetwork:
 *       type: object
 *       required:
 *         - netPath
 *         - tazPath
 *         - model
 *       properties:
 *         netPath:
 *           description: Path of SUMO network file to use.
 *           type: string
 *           example: network/net.net.xml
 *         tazPath:
 *           description: Path of SUMO TAZ file to use.
 *           type: string
 *           example: network/taz.xml
 *         model:
 *           description: Type of model to build.
 *           type: string
 *           enum: [BPR]
 * 
 *     StaticDemand:
 *       type: object
 *       required:
 *         - networkId
 *         - srcPath
 *       properties:
 *         networkId:
 *           description: Resource ID of static network to use as helper to build this demand object.
 *           type: string
 *           example: bpr1
 *         srcPath:
 *           description: Path of O-formatted OD matrix to use.
 *           type: string
 *           example: od/matrix.8.0.9.0.1.fma
 * 
 *     StaticSimulation:
 *       type: object
 *       required:
 *         - networkId
 *         - demandId
 *         - dstPath
 *       properties:
 *         networkId:
 *           description: Resource ID of network to use.
 *           type: string
 *           example: bpr1
 *         demandId:
 *           description: Resource ID of demand to use.
 *           type: string
 *           example: demand1
 *         dstPath:
 *           description: Path to which output will be printed.
 *           type: string
 *           example: out/static-bpr1-demand1.xml
 */

#include <cstring>
#include <iostream>

#include "network/CreateBPRNetwork.hpp"
#include "network/CreateStaticDemand.hpp"
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
    MESSAGE_REGISTER_MAIN(CreateBPRNetwork::Response);
    MESSAGE_REGISTER_MAIN(CreateStaticDemand::Response);
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

    /**yaml PUT /static/network/{id}
     * summary: Create static network resource.
     * description: Create static network resource.
     * tags:
     *   - Static
     * consumes:
     *   - application/json
     * parameters:
     *   - name: id
     *     in: path
     *     required: true
     *     description: ID of new static network resource
     *     schema:
     *       type: string
     *   - name: body
     *     in: body
     *     required: true
     *     description: Description of static network to create.
     *     schema:
     *       $ref: '#/components/schemas/StaticNetwork'
     * responses:
     *   '200':
     *     description: Network created successfully
     */
    server.enroll("PUT", "/static/network/{id}", [](const Server::Request &req) {
        GlobalState::ResourceId resourceId = req.pathVariables.at("id");

        string netPath, tazPath, model;
        try {
            netPath = req.data.at("netPath");
            tazPath = req.data.at("tazPath");
            model = req.data.at("model");
        } catch (const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = nullptr;
        if (model == "BPR") m = new CreateBPRNetwork(resourceId, netPath, tazPath);

        forwardToSimulator(m);
    });

    /**yaml PUT /static/demand/{id}
     * summary: Create static demand resource.
     * description: Create static demand resource.
     * tags:
     *   - Static
     * parameters:
     *   - name: id
     *     in: path
     *     required: true
     *     description: ID of new demand resource
     *     schema:
     *       type: string
     *   - name: body
     *     in: body
     *     required: true
     *     description: Description of static demand to create.
     *     schema:
     *       $ref: '#/components/schemas/StaticDemand'
     * responses:
     *   '200':
     *     description: Demand resource created successfully
     */
    server.enroll("PUT", "/static/demand/{id}", [](const Server::Request &req) {
        GlobalState::ResourceId resourceId = req.pathVariables.at("id");

        string networkId, srcPath;
        try {
            networkId = req.data.at("networkId");
            srcPath = req.data.at("srcPath");
        } catch (const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = new CreateStaticDemand(resourceId, networkId, srcPath);

        forwardToSimulator(m);
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
        string networkId, demandId, dstPath;
        try {
            networkId = req.data.at("networkId");
            demandId = req.data.at("demandId");
            dstPath = req.data.at("dstPath");
        } catch (const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = new RunFWSimulation(networkId, demandId, dstPath);

        forwardToSimulator(m);
    });

    string method = getenv("REQUEST_METHOD");
    string url = getenv("REQUEST_URI");

    server.route(method, url);

    return 0;
}
