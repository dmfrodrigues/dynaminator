/**yaml
 * openapi: 3.0.0
 * info:
 *   title: DynamiNATOR API
 *   description: Web REST API to interact with the DynamiNATOR simulator.
 *   version: 0.0.1
 * servers:
 *   - url: http://localhost
 */

#include <cstring>
#include <iostream>

#include "network/CreateBPRNetwork.hpp"
#include "network/Socket.hpp"
#include "script/Server.hpp"
#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

Server server;

int main() {
    /**yaml PUT /network
     * summary: Create network resource.
     * description: Optional extended description in CommonMark or HTML.
     * responses:
     *   '200':    # status code
     *     description: A JSON array of user names
     *     content:
     *       application/json:
     *         schema:
     *           type: array
     *           items:
     *             type: string
     */
    server.enroll("PUT", "/network", [](const Server::Request &req) {
        cout << "Content-type: application/json\n\n";
        string source, model;
        GlobalState::ResourceId resourceId;
        try {
            source = req.data.at("source");
            model = req.data.at("model");
            resourceId = req.data.at("resourceId");
        } catch (const json::out_of_range &e) {
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *m = nullptr;
        if (model == "BPR") m = new CreateBPRNetwork(resourceId, source);

        if(m == nullptr){
            cout << "Status: 400 Bad Request\n";
            return;
        }

        Socket socket;
        socket.connect("localhost", 8001);
        socket.send(m);
        Message *res_m = socket.receive();
        MessageResponse *res = static_cast<MessageResponse*>(res_m);
        res->handle(cin);
    });

    server.enroll("GET", "/network/edges", [](const Server::Request &) {
        cout << "Content-type: text/html\n\n";
        cout << "Getting edges..." << endl;
    });

    string method = getenv("REQUEST_METHOD");
    string url = getenv("REDIRECT_URL");

    server.route(method, url);

    return 0;
}
