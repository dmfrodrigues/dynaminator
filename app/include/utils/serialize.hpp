#pragma once

#include <iostream>
#include <string>

namespace utils::serialize {
template<class T>
class serialize;

template<class T>
class deserialize;
}  // namespace utils::serialize

namespace std {
ostream &operator<<(ostream &os, const utils::serialize::serialize<string> &s);
istream &operator>>(istream &is, utils::serialize::deserialize<string> s);

ostream &operator<<(ostream &os, const utils::serialize::serialize<bool> &s);
istream &operator>>(istream &is, utils::serialize::deserialize<bool> s);

ostream &operator<<(ostream &os, const utils::serialize::serialize<int> &s);
istream &operator>>(istream &is, utils::serialize::deserialize<int> s);
}  // namespace std

namespace utils::serialize {
template<>
class serialize<std::string> {
    const std::string &t;

   public:
    serialize(const std::string &obj);
    friend std::ostream &std::operator<<(
        std::ostream                 &os,
        const serialize<std::string> &s
    );
};

template<>
class deserialize<std::string> {
    std::string &t;

   public:
    deserialize(std::string &obj);
    friend std::istream &std::operator>>(
        std::istream            &is,
        deserialize<std::string> s
    );
};

template<>
class serialize<bool> {
    bool t;

   public:
    serialize(bool obj);
    friend std::ostream &std::operator<<(
        std::ostream          &os,
        const serialize<bool> &s
    );
};

template<>
class deserialize<bool> {
    bool &t;

   public:
    deserialize(bool &obj);
    friend std::istream &std::operator>>(
        std::istream     &is,
        deserialize<bool> s
    );
};

template<>
class serialize<int> {
    int t;

   public:
    serialize(int obj);
    friend std::ostream &std::operator<<(
        std::ostream         &os,
        const serialize<int> &s
    );
};

template<>
class deserialize<int> {
    int &t;

   public:
    deserialize(int &obj);
    friend std::istream &std::operator>>(
        std::istream    &is,
        deserialize<int> s
    );
};

}  // namespace utils::serialize
