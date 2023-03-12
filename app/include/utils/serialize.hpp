#pragma once

#include <iostream>
#include <string>

namespace utils {
template <class T>
class serialize;

template <class T>
class deserialize;
}  // namespace utils

namespace std {
ostream &operator<<(ostream &os, const utils::serialize<string> &s);
istream &operator>>(istream &is, utils::deserialize<string> s);

ostream &operator<<(ostream &os, const utils::serialize<bool> &s);
istream &operator>>(istream &is, utils::deserialize<bool> s);
}

namespace utils {
template <>
class serialize<std::string> {
    const std::string &t;

   public:
    serialize(const std::string &obj);
    friend std::ostream &std::operator<<(
        std::ostream &os,
        const serialize<std::string> &s);
};

template <>
class deserialize<std::string> {
    std::string &t;

   public:
    deserialize(std::string &obj);
    friend std::istream &std::operator>>(
        std::istream &is,
        deserialize<std::string> s);
};

template <>
class serialize<bool> {
    bool t;

   public:
    serialize(bool obj);
    friend std::ostream &std::operator<<(
        std::ostream &os,
        const serialize<bool> &s);
};

template <>
class deserialize<bool> {
    bool &t;

   public:
    deserialize(bool &obj);
    friend std::istream &std::operator>>(
        std::istream &is,
        deserialize<bool> s);
};

}  // namespace utils
