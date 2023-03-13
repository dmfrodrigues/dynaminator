#pragma once

#include <functional>
#include <limits>

class Vector2 {
public:
    double x;
    double y;

    explicit Vector2(double x = std::numeric_limits<double>::infinity(), double y = std::numeric_limits<double>::infinity());

    Vector2 operator+(const Vector2 &rhs) const;
    Vector2 operator-(const Vector2 &rhs) const;
    Vector2 operator*(double t) const;
    Vector2 operator/(double t) const;

    Vector2& operator+=(const Vector2& other);
    Vector2& operator-=(const Vector2& other);
    Vector2& operator*=(double t);
    Vector2& operator/=(double t);

    bool operator==(const Vector2 &rhs) const;

    Vector2 getOrthogonal() const;
    double getNorm() const;
    double dot(const Vector2 &other) const;
    double getDistance(const Vector2 &other) const;
    double getDet(Vector2 vector2);
    bool isOn(Vector2 a, Vector2 b) const;

    static double getDistance(const Vector2 &v1, const Vector2 &v2);

    static bool compX(const Vector2 &lhs, const Vector2 &rhs);
    static bool compY(const Vector2 &lhs, const Vector2 &rhs);
    static bool compXY(const Vector2 &lhs, const Vector2 &rhs);
};

namespace std {
    template <> struct hash<Vector2> {
        size_t operator()(const Vector2& v) const;
    };
}
