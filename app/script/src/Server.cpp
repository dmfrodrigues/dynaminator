#include "script/Server.hpp"

#include <iostream>
#include <regex>
#include <stdexcept>

#include "script/utils.hpp"

using namespace std;
using json = nlohmann::json;

typedef Server::Method Method;
typedef Server::URL URL;
typedef Server::Function Function;

const string uriElementRegex = "[a-zA-Z0-9]*";

json obtainData() {
    // cout << cin.peek() << " " << char_traits<char>::eof() << endl;
    if (cin.peek() == char_traits<char>::eof()) {
        // cout << "Returning empty" << endl;
        return json({});
    }
    return json::parse(cin);
}

Server::Request::Request(
    const PathVariables &pathVariables_,
    const RequestParams &requestParams_,
    const Data &data_
):
    pathVariables(pathVariables_),
    requestParams(requestParams_),
    data(data_)
{}

void Server::enroll(const Method &method, const URL &url, Function f) {
    Server::routes[method][url] = f;
}

void Server::route(const Method &method, const URL &url) const {
    const unordered_map<string, Function> &routesMethod = routes.at(method);
    for (const auto &p : routesMethod) {
        string s = p.first;
        vector<string> ids;
        while (true) {
            size_t l = s.find('{'), r = s.find('}');
            if (l == string::npos) break;
            if (r == string::npos) throw logic_error("Opening braces '{' does not have corresponding closing braces '}'");
            ids.push_back(s.substr(l + 1, r - l - 1));
            s.replace(l, r - l + 1, "(" + uriElementRegex + ")");
        }

        regex rgx(s);
        smatch matches;
        if(regex_search(url, matches, rgx)){
            PathVariables pathVariables;

            for(size_t i = 1; i < matches.size(); ++i){
                pathVariables[ids.at(i-1)] = matches[i].str();
            }

            Request req(pathVariables, obtainGetParams(), obtainData());

            try {
                p.second(req);
            } catch(const exception &e){
                cout << "Status: 500 Internal Server Error\n";
                cout << "Content-type: text/html\n\n";
                cout << "what(): " << e.what() << "\n";
            }

            return;
        }
    }

    cout << "Status: 404 Not Found\n";
}
