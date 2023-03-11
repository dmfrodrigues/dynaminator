#include "script/Server.hpp"

#include <iostream>
#include <stdexcept>

#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

void Server::enroll(const Method &method, const URL &url, Function f) {
    routes[url][method] = f;
}

void Server::route(const Method &method, const URL &url) const {
    GetParams getParams = obtainGetParams();

    json data = json::parse(cin);

        try {
        routes.at(url).at(method)(getParams, data);
    } catch (out_of_range &e) {
        cout << "Status: 404 Not Found\n";
    }
}
