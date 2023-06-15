#pragma once

#include <cstddef>
#include <deque>

namespace Dynamic::Env {

class Edge;

class Vehicle;

struct Lane {
    typedef size_t Index;

    const Edge &edge;
    Index       index;

    std::deque<std::reference_wrapper<Vehicle>> queue;

    Lane(const Edge &edge, Index index);

    static const Lane INVALID;
};
}  // namespace Dynamic::Env
