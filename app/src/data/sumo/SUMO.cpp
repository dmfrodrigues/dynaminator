#include "data/sumo/SUMO.hpp"

#include <sstream>

using namespace std;

typedef SUMO::Shape Shape;

Shape SUMO::stringToShape(const string &s) {
    Shape shape;

    stringstream ss(s);
    string coordStr;
    while (ss >> coordStr) {
        size_t idx = coordStr.find(',');
        Coord c(
            atof(coordStr.substr(0, idx).c_str()),
            atof(coordStr.substr(idx + 1).c_str()));
    }

    return shape;
}
