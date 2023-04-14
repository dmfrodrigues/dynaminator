#include <thread>

#include "Com/WebSocketServer.hpp"
#include "Com/HTTPServer.hpp"

using namespace std;

int main() {
    thread websocketServerThread([](){
        Com::WebSocketServer wsServer(9001);
        wsServer.loop();
    });

    Com::HTTPServer httpServer(80);
    httpServer.loop();

    return 0;
}

