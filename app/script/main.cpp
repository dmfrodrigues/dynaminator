#include <cstring>
#include <iostream>

#include "network/CreateBPRNetwork.hpp"
#include "network/Socket.hpp"
#include "script/Server.hpp"
#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

int main() {
    Server server;

    server.enroll("PUT", "/network", [](Server::GetParams, nlohmann::json data) {
        cout << "Content-type: application/json\n\n";
        string source, model;
        GlobalState::ResourceId resourceId;
        try {
            source = data.at("source");
            model = data.at("model");
            resourceId = data.at("resourceId");
        } catch (const json::out_of_range &e) {
            cout << "Status: 400 Bad Request\n";
            return;
        }
        MessageRequest *req = nullptr;
        if (model == "BPR") req = new CreateBPRNetwork(resourceId, source);
        
        if(req == nullptr){
            cout << "Status: 400 Bad Request\n";
            return;
        }

        Socket socket;
        socket.connect("localhost", 8001);
        socket.send(req);
        Message *m = socket.receive();
        MessageResponse *res = static_cast<MessageResponse*>(m);
        res->handle(cin);
    });

    server.enroll("GET", "/network/edges", [](Server::GetParams getParams, nlohmann::json) {
        cout << "Content-type: text/html\n\n";
        cout << "Getting edges..." << endl;
    });

    string method = getenv("REQUEST_METHOD");
    string url = getenv("REDIRECT_URL");

    server.route(method, url);

    return 0;
}
