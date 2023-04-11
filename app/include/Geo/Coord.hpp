#pragma once

#include <functional>

#include "utils/Vector2.hpp"
#include "utils/stringify.hpp"

namespace Geo {
class Coord : public Vector2 {
// private:
//     double getMetersPerLatDeg() const;
//     double getMetersPerLonDeg() const;
public:
    explicit Coord();
    explicit Coord(double x, double y);
//     explicit Coord(const Vector2 &v);

//     /**
//      * @brief Get distance between two positions in SI units (meters).
//      * 
//      * @param p1        First position
//      * @param p2        Second position
//      * @return double   Distance between them in meters
//      */
//     static double getDistanceArc(const Coord   &p1, const Coord   &p2);
//     static double getDistanceArcSimple(const Coord &p1, const Coord &p2);
    
//     double &lat();
//     double &lon();

//     const double &lat() const;
//     const double &lon() const;

//     Coord& operator+=(const Coord& other);
//     Coord& operator-=(const Coord& other);
//     Coord& operator*=(double t);
//     Coord& operator/=(double t);

//     Coord operator+(const Coord &rhs) const;
//     Coord operator-(const Coord &rhs) const;
//     Coord operator*(double t) const;
//     Coord operator/(double t) const;

//     static bool compXY(const Coord &c1, const Coord &c2);

//     static double MetersToLatDegrees();
//     static double MetersToLonDegrees(double lat);
//     static double LatDegreesToMeters();
//     static double LonDegreesToMeters(double lat);
};
}  // namespace Geo

namespace std {
template<>
struct hash<Geo::Coord> {
    size_t operator()(const Geo::Coord &v) const;
};
}  // namespace std

namespace utils::stringify {
template<>
class stringify<Geo::Coord> {
   public:
    static Geo::Coord  fromString(const std::string &s);
    static std::string toString(const Geo::Coord &t);
};
}  // namespace utils::stringify
