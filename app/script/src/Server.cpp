#include "script/Server.hpp"

#include <iostream>
#include <stdexcept>

#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

typedef Server::Method Method;
typedef Server::URL URL;
typedef Server::Function Function;

Server::Request::Request()
:requestParams(obtainGetParams()), data(json::parse(cin))
{}

void Server::enroll(const Method &method, const URL &url, Function f) {
    Server::routes[url][method] = f;
}

void Server::route(const Method &method, const URL &url) const {
    try {
        routes.at(url).at(method)(req);
    } catch (out_of_range &e) {
        cout << "Status: 404 Not Found\n";
    }
}
