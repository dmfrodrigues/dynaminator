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
    typedef std::map<std::string, std::string> RequestParams;

    class Request {
        friend Server;

       public:
        const RequestParams requestParams;
        const nlohmann::json data;

       private:
        Request();
    };

    typedef std::function<void(const Request &)> Function;

   private:
    std::unordered_map<URL, std::unordered_map<Method, Function>> routes;
    Request req;

   public:
    void enroll(const Method &method, const URL &url, Function f);

    void route(const Method &method, const URL &url) const;
};
