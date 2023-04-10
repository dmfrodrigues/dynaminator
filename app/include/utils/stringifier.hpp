#pragma once

#include <string>
#include <sstream>
#include <vector>

namespace utils {
template <class T>
class stringifier {
   public:
    static std::string toString(const T &t);
    static T fromString(const std::string &s);
};

template <>
class stringifier<int> {
   public:
    static std::string toString(const int &t);
    static int fromString(const std::string &s);
};

template <>
class stringifier<unsigned long> {
   public:
    static std::string toString(const unsigned long &t);
    static unsigned long fromString(const std::string &s);
};

template <>
class stringifier<float> {
   public:
    static std::string toString(const float &t);
    static float fromString(const std::string &s);
};

template <>
class stringifier<double> {
   public:
    static std::string toString(const double &t);
    static double fromString(const std::string &s);
};

template <>
class stringifier<std::string> {
   public:
    static std::string toString(const std::string &t);
    static std::string fromString(const std::string &s);
};

template <class T>
class stringifier<std::vector<T>> {
   public:
    static std::string toString(const std::vector<T> &t){
        std::stringstream ss;

        bool first = true;
        for(const T &el: t){
            if(first) first = false;
            else ss << " ";
            
            ss << stringifier<T>::toString(el);
        }

        return ss.str();
    }
    static std::vector<T> fromString(const std::string &s){
        std::vector<T> ret;

        std::stringstream ss(s);
        std::string elStr;
        while (ss >> elStr) {
            ret.emplace_back(stringifier<T>::fromString(elStr));
        }

        return ret;
    }
};

}  // namespace utils
