#include "data/SUMO/SUMO.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

#include "utils/stringify.hpp"

using namespace std;
using namespace SUMO;
using namespace utils::stringify;

size_t std::hash<Vector2>::operator()(const Coord &v) const {
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

    ssize_t i = upper_bound(lengths.begin(), lengths.end(), l) - lengths.begin() - 1;
    assert(i >= 0);
    assert(i < v.size());

    Length lInSegment = l - lengths[i];

    assert(lInSegment >= 0);

    Vector2 dir = directionAtProgress(progress);

    return v.at(i) + dir * lInSegment;
}

Vector2 Shape::directionAtProgress(double progress) const {
    if(lengths.empty()) computeLengths();

    Length l = len.value() * progress;

    l = max(0.0, min(len.value(), l));

    ssize_t i = upper_bound(lengths.begin(), lengths.end(), l) - lengths.begin() - 1;

    i = min(i, (ssize_t)v.size() - 2);

    assert(i >= 0);
    assert(i < v.size() - 1);

    Vector2 dir = v.at(i + 1) - v.at(i);
    dir /= Vector2::Magnitude(dir);

    return dir;
}

Length Shape::length() const {
    if(lengths.empty()) computeLengths();

    return *len;
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

Shape stringify<Shape>::fromString(const std::string &s) {
    vector<Coord> v = stringify<vector<Coord>>::fromString(s);
    return Shape(v.begin(), v.end());
}
