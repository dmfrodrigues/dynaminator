#include "data/SUMO/SUMO.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

#include "Vector2.hpp"
#include "utils/stringify.hpp"

using namespace std;
using namespace SUMO;
using namespace utils::stringify;

Coord::Coord(double x, double y, double z):
    Vector2(x, y),
    Z(z) {}

Coord Coord::operator+(const Coord &rhs) const {
    return Coord(
        X + rhs.X,
        Y + rhs.Y,
        Z + rhs.Z
    );
}
Coord Coord::operator+(const Vector2 &rhs) const {
    return Coord(
        X + rhs.X,
        Y + rhs.Y,
        Z
    );
}

Coord Coord::operator-(const Coord &rhs) const {
    return Coord(
        X - rhs.X,
        Y - rhs.Y,
        Z - rhs.Z
    );
}
Coord Coord::operator-(const Vector2 &rhs) const {
    return Coord(
        X - rhs.X,
        Y - rhs.Y,
        Z
    );
}

Coord Coord::operator*(double rhs) const {
    return Coord(
        X * rhs,
        Y * rhs,
        Z * rhs
    );
}

Coord Coord::operator/(double rhs) const {
    return Coord(
        X / rhs,
        Y / rhs,
        Z / rhs
    );
}

size_t std::hash<Coord>::operator()(const Coord &v) const {
    return hash<double>()(v.X) ^ (hash<double>()(v.Y) << 1);
}

Shape::Shape(initializer_list<Coord> il):
    v(il) {}

Coord       Shape::front() { return v.front(); }
const Coord Shape::front() const { return v.front(); }

Coord       Shape::back() { return v.back(); }
const Coord Shape::back() const { return v.back(); }

Shape::iterator Shape::begin() { return v.begin(); }
Shape::iterator Shape::end() { return v.end(); }

Shape::const_iterator Shape::begin() const { return v.begin(); }
Shape::const_iterator Shape::end() const { return v.end(); }

size_t Shape::size() const { return v.size(); }

bool Shape::empty() const { return v.empty(); }

Coord       &Shape::at(size_t i) { return v.at(i); }
const Coord &Shape::at(size_t i) const { return v.at(i); }

Coord &Shape::operator[](size_t i) { return v[i]; }
const Coord &Shape::operator[](size_t i) const { return v[i]; }

void Shape::computeLengths() const {
    lengths.resize(v.size());
    Length l = 0;
    for(size_t i = 0; i < v.size() - 1; ++i) {
        lengths[i] = l;
        l += Vector2::Magnitude(v[i + 1] - v[i]);
    }
    lengths.at(v.size() - 1) = l;
    len                      = l;
}

Coord Shape::locationAtProgress(double progress) const {
    if(lengths.empty()) computeLengths();

    Length l = len.value() * progress;

    l = max(0.0, min(len.value(), l));

    ssize_t pos = upper_bound(lengths.begin(), lengths.end(), l) - lengths.begin() - 1;
    assert(pos >= 0);

    size_t i = (size_t)pos;
    assert(i < v.size());

    Length lInSegment = l - lengths[i];

    assert(lInSegment >= 0);

    if(progress >= 1.0)
        return v.back();

    assert(i < v.size() - 1);

    double progressInSegment = lInSegment / Vector2::Magnitude(v.at(i + 1) - v.at(i));

    return v.at(i) * (1.0 - progressInSegment) + v.at(i + 1) * progressInSegment;
}

Vector2 Shape::directionAtProgress(double progress) const {
    if(lengths.empty()) computeLengths();

    Length l = len.value() * progress;

    l = max(0.0, min(len.value(), l));

    ssize_t pos = upper_bound(lengths.begin(), lengths.end(), l) - lengths.begin() - 1;

    pos = min(pos, (ssize_t)v.size() - 2);

    assert(pos >= 0);

    size_t i = (size_t)pos;

    assert(i < v.size() - 1);

    Vector2 dir = v.at(i + 1) - v.at(i);
    dir /= Vector2::Magnitude(dir);

    return dir;
}

Length Shape::length() const {
    if(lengths.empty()) computeLengths();

    return *len;
}

double Shape::getProgress(SUMO::Length l) const {
    return l / length();
}

Coord stringify<Coord>::fromString(const string &s) {
    string str = s;

    size_t idx;

    idx      = str.find(',');
    double x = stringify<double>::fromString(str.substr(0, idx).c_str());
    str      = str.substr(idx + 1);

    idx      = str.find(',');
    double y = stringify<double>::fromString(str.substr(0, idx).c_str());
    str      = (idx == string::npos ? "" : str.substr(idx + 1));

    double z = 0.0;
    if(!str.empty()) {
        z = stringify<double>::fromString(str.c_str());
    }

    return Coord(x, y, z);
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

Shape stringify<Shape>::fromString(const std::string &s) {
    vector<Coord> v = stringify<vector<Coord>>::fromString(s);
    return Shape(v.begin(), v.end());
}
