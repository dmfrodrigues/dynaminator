#include "utils/Vector2.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <cmath>

#define EPSILON 0.00000001

Vector2::Vector2(double x_, double y_) : x(x_), y(y_) {

}

Vector2 &Vector2::operator+=(const Vector2 &other) {
    x += other.x;
    y += other.y;

    return *this;
}

Vector2 &Vector2::operator-=(const Vector2 &other) {
    x -= other.x;
    y -= other.y;

    return *this;
}

Vector2 &Vector2::operator*=(double t) {
    x *= t;
    y *= t;

    return *this;
}

Vector2 &Vector2::operator/=(double t) {
    x /= t;
    y /= t;

    return *this;
}


Vector2 Vector2::operator+(const Vector2 &rhs) const {
    return Vector2(x + rhs.x, y + rhs.y);
}

Vector2 Vector2::operator-(const Vector2 &rhs) const {
    return Vector2(x - rhs.x, y - rhs.y);
}

Vector2 Vector2::operator*(double t) const {
    return Vector2(x*t, y*t);
}

Vector2 Vector2::operator/(double t) const {
    return Vector2(x/t, y/t);
}

bool Vector2::operator==(const Vector2 &rhs) const {
    return fabs(x - rhs.x) < EPSILON && fabs(y - rhs.y) < EPSILON;
}

Vector2 Vector2::getOrthogonal() const {
    return Vector2(-y, x);
}

double Vector2::dot(const Vector2 &other) const {
    return x * other.x + y * other.y;
}

double Vector2::getNorm() const {
    return std::sqrt(x * x + y * y);
}

double Vector2::getDistance(const Vector2 &other) const {
    return (*this - other).getNorm();
}

double Vector2::getDistance(const Vector2 &v1, const Vector2 &v2) {
    return (v1 - v2).getNorm();
}

double Vector2::getDet(Vector2 v) {
    return x * v.y - y * v.x;
}

bool Vector2::compX(const Vector2 &lhs, const Vector2 &rhs){
    return (lhs.x < rhs.x);
}

bool Vector2::compY(const Vector2 &lhs, const Vector2 &rhs){
    return (lhs.y < rhs.y);
}

bool Vector2::compXY(const Vector2 &lhs, const Vector2 &rhs){
    if(lhs.x != rhs.x) return lhs.x < rhs.x;
    else               return lhs.y < rhs.y;
}

bool collinear(Vector2 a, Vector2 b, Vector2 c) {
    if (a.x == b.x && b.x == c.x)
        return true;
        
    return fabs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) <= EPSILON;
}

bool within(double p, double q, double r) {
    return (p <= q && q <= r) || (r <= q && q <= p);
}

bool Vector2::isOn(Vector2 a, Vector2 b) const {
    Vector2 c = *this;

    return collinear(a, b, c) && (a.x != b.x ? within(a.x, c.x, b.x) : within(a.y, c.y, b.y));
}

size_t std::hash<Vector2>::operator()(const Vector2& v) const {
    return hash<double>()(v.x) ^ (hash<double>()(v.y) << 1);
}

#pragma GCC diagnostic pop
