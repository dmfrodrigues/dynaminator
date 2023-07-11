#pragma once

#include <initializer_list>
#include <list>
#include <optional>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "Vector2.hpp"
#pragma GCC diagnostic pop

#include "utils/stringify.hpp"

/**
 * @brief SUMO (Simulation of Urban MObility) is an open source, portable,
 * microscopic and continuous multi-modal traffic simulation package designed to
 * handle large networks.
 *
 * SUMO is developed by the German Aerospace Center and community users. It has
 * been freely available as open-source since 2001, and since 2017 it is an
 * Eclipse Foundation project. (Wikipedia)
 */
namespace SUMO {
typedef std::string ID;
typedef double      Time;
typedef double      Length;
typedef double      Speed;
typedef size_t      Index;
typedef Vector2     Coord;

class Shape {
   private:
    std::vector<Coord>            v;
    mutable std::vector<Length>   lengths;
    mutable std::optional<Length> len;

    void computeLengths() const;

   public:
    typedef std::vector<Coord>::iterator         iterator;
    typedef std::vector<Coord>::const_iterator   const_iterator;
    typedef std::vector<Coord>::reverse_iterator reverse_iterator;

    Shape(std::initializer_list<Coord> il);

    template<class Iterator>
    Shape(Iterator begin, Iterator end):
        v(begin, end) {}

    Coord       front();
    const Coord front() const;

    Coord       back();
    const Coord back() const;

    iterator begin();
    iterator end();

    const_iterator begin() const;
    const_iterator end() const;

    size_t size() const;
    bool   empty() const;

    Coord       &at(size_t i);
    const Coord &at(size_t i) const;

    Coord       &operator[](size_t i);
    const Coord &operator[](size_t i) const;

    Coord   locationAtProgress(double progress) const;
    Vector2 directionAtProgress(double progress) const;
    Length  length() const;
};
}  // namespace SUMO

namespace std {
template<>
struct hash<SUMO::Coord> {
    size_t operator()(const SUMO::Coord &v) const;
};
}  // namespace std

namespace utils::stringify {

template<>
class stringify<SUMO::Coord> {
   public:
    static SUMO::Coord fromString(const std::string &s);
    static std::string toString(const SUMO::Coord &t);
};

template<>
class stringify<std::pair<SUMO::Coord, SUMO::Coord>> {
   public:
    static std::pair<SUMO::Coord, SUMO::Coord> fromString(const std::string &s);
    static std::string                         toString(const std::pair<SUMO::Coord, SUMO::Coord> &t);
};

template<>
class stringify<SUMO::Shape> {
   public:
    static SUMO::Shape fromString(const std::string &s);
    static std::string toString(const SUMO::Shape &t);
};

}  // namespace utils::stringify
