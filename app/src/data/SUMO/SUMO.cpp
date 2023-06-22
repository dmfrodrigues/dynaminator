#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace SUMO;
using namespace utils::stringify;

size_t std::hash<Vector2>::operator()(const Coord &v) const {
    return hash<double>()(v.X) ^ (hash<double>()(v.Y) << 1);
}

Coord stringify<Coord>::fromString(const string &s) {
    size_t idx = s.find(',');

    Coord c(
        stringify<double>::fromString(s.substr(0, idx).c_str()),
        stringify<double>::fromString(s.substr(idx + 1).c_str())
    );

    return c;
}

string stringify<Coord>::toString(const Coord &t) {
    char s[256];
    sprintf(s, "%lf,%lf", t.X, t.Y);
    return string(s);
}
