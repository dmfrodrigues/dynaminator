#pragma once

#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

class Server {
   public:
    typedef std::string Method;
    typedef std::string URL;
    typedef std::map<std::string, std::string> PathVariables;
    typedef std::map<std::string, std::string> RequestParams;
    typedef nlohmann::json Data;

    class Request {
        friend Server;

       public:
        const PathVariables pathVariables;
        const RequestParams requestParams;
        const Data data;

       private:
        Request(
            const PathVariables &pathVariables,
            const RequestParams &requestParams,
            const Data &data);
    };

    typedef std::function<void(const Request &)> Function;

   private:
    std::unordered_map<URL, std::unordered_map<Method, Function>> routes;

   public:
    void enroll(const Method &method, const URL &url, Function f);

    void route(const Method &method, const URL &url) const;
};
