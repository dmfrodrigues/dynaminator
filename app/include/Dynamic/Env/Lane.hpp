#pragma once

#include <cstddef>

namespace Dynamic::Env {

class Edge;

struct Lane {
    typedef size_t Index;

    const Edge &edge;
    Index       index;

    Lane(const Edge &edge, Index index);

    static const Lane INVALID;
};
}  // namespace Dynamic::Env
