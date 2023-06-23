#pragma once

#include <functional>
#include <set>

#include "utils/reference_wrapper.hpp"
namespace Dynamic::Env {

class Edge;

class TAZ {
   public:
    typedef long ID;

    ID id;

    // clang-format off
    std::set<
        std::reference_wrapper<Edge>,
        std::less<Edge>
    > sources;
    std::set<
        std::reference_wrapper<Edge>,
        std::less<Edge>
    > sinks;
    // clang-format on

    TAZ(ID id);
};

}  // namespace Dynamic::Env
