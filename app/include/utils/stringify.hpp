#pragma once

#include <string>
#include <sstream>
#include <vector>

namespace utils {
template <class T>
class stringify {
   public:
    static std::string toString(const T &t);
    static T fromString(const std::string &s);
};

template <>
class stringify<int> {
   public:
    static std::string toString(const int &t);
    static int fromString(const std::string &s);
};

template <>
class stringify<unsigned long> {
   public:
    static std::string toString(const unsigned long &t);
    static unsigned long fromString(const std::string &s);
};

template <>
class stringify<float> {
   public:
    static std::string toString(const float &t);
    static float fromString(const std::string &s);
};

template <>
class stringify<double> {
   public:
    static std::string toString(const double &t);
    static double fromString(const std::string &s);
};

template <>
class stringify<std::string> {
   public:
    static std::string toString(const std::string &t);
    static std::string fromString(const std::string &s);
};

template <class T>
class stringify<std::vector<T>> {
   public:
    static std::string toString(const std::vector<T> &t){
        std::stringstream ss;

        bool first = true;
        for(const T &el: t){
            if(first) first = false;
            else ss << " ";
            
            ss << stringify<T>::toString(el);
        }

        return ss.str();
    }
    static std::vector<T> fromString(const std::string &s){
        std::vector<T> ret;

        std::stringstream ss(s);
        std::string elStr;
        while (ss >> elStr) {
            ret.emplace_back(stringify<T>::fromString(elStr));
        }

        return ret;
    }
};

}  // namespace utils
