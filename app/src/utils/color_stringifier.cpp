#include "utils/color_stringifier.hpp"

using namespace std;

string utils::stringifier<color::rgb<float>>::toString(const color::rgb<float> &t) {
    stringstream ss;
    ss
        << color::get::red(t) << ","
        << color::get::green(t) << ","
        << color::get::blue(t);
    return ss.str();
}

color::rgb<float> utils::stringifier<color::rgb<float>>::fromString(const string &s) {
    color::rgb<float> c;

    size_t l, r;
    l = 0;
    r = s.find(',', l) - 1;
    color::set::red(c, stringifier<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::green(c, stringifier<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::blue(c, stringifier<float>::fromString(s.substr(l, r)));

    return c;
}


string utils::stringifier<color::rgba<float>>::toString(const color::rgba<float> &t) {
    stringstream ss;
    ss
        << color::get::red(t) << ","
        << color::get::green(t) << ","
        << color::get::blue(t) << ","
        << color::get::alpha(t);
    return ss.str();
}

color::rgba<float> utils::stringifier<color::rgba<float>>::fromString(const string &s) {
    color::rgba<float> c;

    size_t l, r;
    l = 0;
    r = s.find(',', l) - 1;
    color::set::red(c, stringifier<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::green(c, stringifier<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::blue(c, stringifier<float>::fromString(s.substr(l, r)));
    l = r;
    r = s.find(',', l) - 1;
    color::set::alpha(c, stringifier<float>::fromString(s.substr(l, r)));

    return c;
}
