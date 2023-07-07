#include "data/SUMO/SUMO.hpp"

#include <cassert>

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

pair<Coord, Coord> stringify<pair<Coord, Coord>>::fromString(const string &s) {
    string         sCopy = s;
    vector<double> numbers;
    while(true) {
        size_t idx = sCopy.find(',');
        if(idx == string::npos) {
            numbers.push_back(stringify<double>::fromString(sCopy));
            break;
        }

        string sNumber = sCopy.substr(0, idx - 1);
        sCopy          = sCopy.substr(idx + 1);

        numbers.push_back(stringify<double>::fromString(sNumber));
    }

    assert(numbers.size() == 4);

    return pair<Coord, Coord>(
        Coord(numbers.at(0), numbers.at(1)),
        Coord(numbers.at(2), numbers.at(3))
    );
}
