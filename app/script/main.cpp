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
 *         - source
 *         - model
 *       properties:
 *         source:
 *           description: Path of SUMO network file to use
 *           type: string
 *           example: net.net.xml
 *         model:
 *           description: Type of model to build.
 *           type: string
 *           enum: [BPR]
 */

#include <cstring>
#include <iostream>

#include "network/CreateBPRNetwork.hpp"
#include "network/CreateStaticDemand.hpp"
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
    socket.connect("localhost", 8001);
    socket.send(m);
    Message *res_m = socket.receive();
    MessageResponse *res = static_cast<MessageResponse*>(res_m);
    res->handle(cout);
}

int main() {
    /**yaml GET /hello
     * summary: Ping server.
     */
    server.enroll("GET", "/hello", [](const Server::Request &) {
        cout << "Content-type: text/html\n\n";
        cout << "Hello world!\n";
    });

    /**yaml PUT /static/network/{id}
     * summary: Create static network resource.
     * description: Create static network resource.
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
     * parameters:
     *   - name: id
     *     in: path
     *     required: true
     *     description: ID of new demand resource
     *     schema:
     *       type: string
     *   - name: source
     *     in: query
     *     required: true
     *     description: Path of OD matrix file to use.
     *     schema:
     *       type: string
     * responses:
     *   '200':
     *     description: Demand resource created successfully
     */
    server.enroll("PUT", "/static/demand/{id}", [](const Server::Request &req) {
        GlobalState::ResourceId resourceId = req.pathVariables.at("id");

        string networkId, path;
        try {
            networkId = req.data.at("networkId");
            path = req.data.at("path");
        } catch (const json::out_of_range &e) {
            cout << "Content-type: text/html\n\n";
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = new CreateStaticDemand(resourceId, networkId, path);

        forwardToSimulator(m);
    });

    string method = getenv("REQUEST_METHOD");
    string url = getenv("REQUEST_URI");

    server.route(method, url);

    return 0;
}
