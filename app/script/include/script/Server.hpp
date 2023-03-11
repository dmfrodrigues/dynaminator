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
    typedef std::map<std::string, std::string> GetParams;
    typedef std::function<
        void(
            const GetParams &,
            const nlohmann::json &)>
        Function;

   private:
    std::unordered_map<URL, std::unordered_map<Method, Function>> routes;

   public:
    void enroll(const Method &method, const URL &url, Function f);

    void route(const Method &method, const URL &url) const;
};
