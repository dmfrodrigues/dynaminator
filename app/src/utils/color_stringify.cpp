#include "utils/color_stringify.hpp"

#include <spdlog/spdlog.h>

#include <iostream>

using namespace std;
using namespace utils::stringify;

string stringify<color::rgb<float>>::toString(const color::rgb<float> &t) {
    stringstream ss;
    ss
        << color::get::red(t) << ","
        << color::get::green(t) << ","
        << color::get::blue(t);
    return ss.str();
}

const unordered_map<string, color::rgb<float>> string2Color = {
    {"white", color::rgb<float>({1.0, 1.0, 1.0})},
    {"grey", color::rgb<float>({0.5, 0.5, 0.5})},
    {"black", color::rgb<float>({0.0, 0.0, 0.0})},
};

color::rgb<float> stringify<color::rgb<float>>::fromString(const string &s) {
    color::rgb<float> c;

    if(string2Color.find(s) != string2Color.end()) return string2Color.at(s);

    size_t l, r;
    l = 0;
    r = s.find(',', l);
    color::set::red(c, stringify<float>::fromString(s.substr(l, r - l)) / 255.0f);
    l = r + 1;
    r = s.find(',', l);
    color::set::green(c, stringify<float>::fromString(s.substr(l, r - l)) / 255.0f);
    l = r + 1;
    r = s.find(',', l);
    color::set::blue(c, stringify<float>::fromString(s.substr(l, r - l)) / 255.0f);

    return c;
}

string stringify<color::rgba<float>>::toString(const color::rgba<float> &t) {
    stringstream ss;
    ss
        << color::get::red(t) << ","
        << color::get::green(t) << ","
        << color::get::blue(t) << ","
        << color::get::alpha(t);
    return ss.str();
}

color::rgba<float> stringify<color::rgba<float>>::fromString(const string &s) {
    color::rgba<float> c;

    size_t l, r;
    l = 0;
    r = s.find(',', l) - 1;
    color::set::red(c, stringify<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::green(c, stringify<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::blue(c, stringify<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::alpha(c, stringify<float>::fromString(s.substr(l, r)));

    return c;
}
