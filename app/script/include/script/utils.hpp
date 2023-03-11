#pragma once

#include <map>
#include <string>

template<class Container=std::map<std::string, std::string>>
Container decodeGetParams(const std::string &params){
    Container ret;
    size_t begin = 0, end = params.find("&");
    while(end != std::string::npos){
        std::string param = params.substr(begin, end-begin);

        size_t eq = param.find("=");
        std::string
            key = param.substr(0, eq),
            value = param.substr(eq+1);
        ret[key] = value;

        begin = end + 1;
        end = params.find("&", begin);
    }
    std::string param = params.substr(begin, std::string::npos);
    size_t eq = param.find("=");
    std::string
        key = param.substr(0, eq),
        value = param.substr(eq+1);
    ret[key] = value;
    
    return ret;
}

template<class Container=std::map<std::string, std::string>>
Container obtainGetParams(){
    return decodeGetParams<Container>(getenv("QUERY_STRING"));
}
