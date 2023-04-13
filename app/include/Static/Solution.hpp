#pragma once

#include <memory>
#include <unordered_set>

#include "Static/supply/Network.hpp"

namespace Static {
class SolutionBase;

class Solution {
    friend SolutionBase;

    struct Internals {
        std::unordered_map<Network::Path, Network::Flow> paths;
        std::unordered_set<Network::Edge::ID>            edges;
        std::vector<Network::Flow>                       flows;

        std::shared_ptr<Internals>
            s1 = std::shared_ptr<Internals>(nullptr),
            s2 = std::shared_ptr<Internals>(nullptr);

        void addToRoutes(
            std::unordered_map<Network::Path, Network::Flow> &routes
        ) const;
    };

    std::shared_ptr<Internals>
        s = std::shared_ptr<Internals>(new Internals());

   public:
    Solution();
    Solution(const Solution &sol);

    virtual std::unordered_set<Network::Edge::ID> getEdges() const;
    virtual Network::Flow                         getFlowInEdge(Network::Edge::ID id) const;

    virtual std::unordered_map<Network::Path, Network::Flow> getRoutes() const;

    Solution &operator=(const Solution &sol);

    static Solution interpolate(
        const Solution &s1,
        const Solution &s2,
        Network::Flow   alpha
    );
};

class SolutionBase: public Solution {
   public:
    void addPath(const Network::Path &path, Network::Flow flow);
};

}  // namespace Static
