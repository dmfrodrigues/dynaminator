#include "utils/color_stringify.hpp"

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

color::rgb<float> stringify<color::rgb<float>>::fromString(const string &s) {
    color::rgb<float> c;

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
