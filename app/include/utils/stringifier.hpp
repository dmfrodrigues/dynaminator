#pragma once

#include <string>

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
class stringifier<double> {
   public:
    static std::string toString(const double &t);
    static double fromString(const std::string &s);
};

}  // namespace utils
