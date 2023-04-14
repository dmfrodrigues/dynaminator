#include <thread>

#include "Com/WebSocketServer.hpp"
#include "Com/HTTPServer.hpp"

using namespace std;

int main() {
    Com::WebSocketServer wsServer(9001);
    thread websocketServerThread([&wsServer](){
        wsServer.loop();
    }); 

    Com::HTTPServer httpServer(80);
    httpServer.loop();

    return 0;
}
