#pragma once

#include <memory>
#include <unordered_set>

#include "Static/supply/Network.hpp"

namespace Static {
class SolutionBase;

class Solution {
    friend SolutionBase;

   public:
    typedef std::unordered_map<Network::Path, Flow> Routes;

   private:
    struct Internals {
        Routes                                paths;
        std::unordered_set<Network::Edge::ID> edges;
        std::vector<Flow>            flows;

        std::shared_ptr<Internals>
            s1 = std::shared_ptr<Internals>(nullptr),
            s2 = std::shared_ptr<Internals>(nullptr);

        double alpha;

        void addToRoutes(Routes &routes) const;
        void addToRoutes(Routes &routes, double a) const;

        void materialize();
    };

    std::shared_ptr<Internals>
        s = std::shared_ptr<Internals>(new Internals());

   public:
    Solution();
    Solution(const Solution &sol);

    virtual std::unordered_set<Network::Edge::ID> getEdges() const;

    virtual Flow getFlowInEdge(Network::Edge::ID id) const;

    virtual Routes getRoutes() const;

    Solution &operator=(const Solution &sol);

    static Solution interpolate(
        const Solution &s1,
        const Solution &s2,
        Flow   alpha
    );

    void materialize();

    double getTotalFlow() const;
};

class SolutionBase: public Solution {
   public:
    void addPath(const Network::Path &path, Flow flow);
};

}  // namespace Static
