#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#pragma GCC diagnostic ignored "-Wswitch-enum"
#pragma GCC diagnostic ignored "-Wswitch-default"
#include <httplib.h>
#pragma GCC diagnostic pop

namespace Com {
class HTTPServer {
    int port;
    httplib::Server svr;

    void helloGet(const httplib::Request &req, httplib::Response &res);
    void staticSimulationPost(const httplib::Request &req, httplib::Response &res);

   public:
    HTTPServer(int port);
    void loop();
};
}  // namespace Com
