#include "data/sumo/SUMO.hpp"

#include <sstream>

using namespace std;

typedef SUMO::Shape Shape;

Shape utils::stringifier<Shape>::fromString(const std::string &s) {
    Shape shape;

    stringstream ss(s);
    string coordStr;
    while (ss >> coordStr) {
        size_t idx = coordStr.find(',');
        Coord c(
            stringifier<double>::fromString(coordStr.substr(0, idx).c_str()),
            stringifier<double>::fromString(coordStr.substr(idx + 1).c_str()));
    }

    return shape;
}
