#pragma once

#include <httplib.h>

namespace Com {
class HTTPServer {
    int port;

    httplib::Server svr;

    void helloGet(const httplib::Request &req, httplib::Response &res);
    void staticSimulationPost(const httplib::Request &req, httplib::Response &res);
    void staticSimulationJoinGet(const httplib::Request &req, httplib::Response &res);

    void dynamicSimulationPost(const httplib::Request &req, httplib::Response &res);

   public:
    HTTPServer(int port);
    void loop();
};
}  // namespace Com
