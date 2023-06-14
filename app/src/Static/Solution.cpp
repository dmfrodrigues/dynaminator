#include "Static/Solution.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>

using namespace std;
using namespace Static;

typedef Network::Edge    Edge;
typedef Network::Path    Path;
typedef Solution::Routes Routes;

Solution::Solution() {}

Solution::Solution(const Solution &sol):
    s(sol.s) {}

void Solution::Internals::addToRoutes(
    Routes &routes
) const {
    if(s1 == nullptr && s2 == nullptr) {
        routes = paths;
        return;
    }

    addToRoutes(routes, 1.0);
}

void Solution::Internals::addToRoutes(
    Routes &routes,
    double  a
) const {
    for(const auto &[path, flow]: paths) {
        routes[path] += a * flow;
    }
    if(s1 != nullptr) s1->addToRoutes(routes, a * (1.0 - alpha));
    if(s2 != nullptr) s2->addToRoutes(routes, a * alpha);
}

unordered_set<Edge::ID> Solution::getEdges() const {
    const auto &edges = s.get()->edges;
    return edges;
}

Flow Solution::getFlowInEdge(Edge::ID id) const {
    const auto &flows = s.get()->flows;
    if((Edge::ID)flows.size() <= id)
        return 0.0;
    else
        return flows[(size_t)id];
}

Routes Solution::getRoutes() const {
    Routes ret;
    s->addToRoutes(ret);
    return ret;
}

Solution &Solution::operator=(const Solution &sol) {
    s = sol.s;

    return *this;
}

Solution Solution::interpolate(
    const Solution &s1,
    const Solution &s2,
    Flow            alpha
) {
    if(alpha == 0.0)
        return s1;
    if(alpha == 1.0)
        return s2;

    Solution ret;

    ret.s->alpha = alpha;
    ret.s->s1    = s1.s;
    ret.s->s2    = s2.s;

    const unordered_set<Edge::ID> &edges1 = s1.getEdges();
    const unordered_set<Edge::ID> &edges2 = s2.getEdges();

    auto &edges = ret.s->edges;
    auto &flows = ret.s->flows;

    edges.insert(edges1.begin(), edges1.end());
    edges.insert(edges2.begin(), edges2.end());

    Edge::ID maxId = (edges.empty() ? -1 : *max_element(edges.begin(), edges.end()));

    flows.resize(size_t(maxId + 1), 0.0);
    for(size_t id = 0; id < flows.size(); ++id) {
        flows[id] =
            (1.0 - alpha) * s1.getFlowInEdge(id) + (alpha)*s2.getFlowInEdge(id);
    }

    return ret;
}

void Solution::materialize() {
    s->materialize();
}

void Solution::Internals::materialize() {
    if(s1) {
        s1->addToRoutes(paths, 1.0 - alpha);
        s1.reset();
    }
    if(s2) {
        s2->addToRoutes(paths, alpha);
        s2.reset();
    }
}

double Solution::getTotalFlow() const {
    double ret    = 0.0;
    Routes routes = getRoutes();
    for(const auto &[path, flow]: routes) {
        ret += flow;
    }
    return ret;
}

void SolutionBase::addPath(const Path &path, Flow newFlow) {
    auto &paths = s.get()->paths;
    auto &flows = s.get()->flows;
    auto &edges = s.get()->edges;

    Flow &f     = paths[path];
    Flow  delta = newFlow - f;

    f = newFlow;

    Edge::ID maxId = (path.empty() ? -1 : *max_element(path.begin(), path.end()));
    if(maxId >= (Edge::ID)flows.size()) flows.resize(maxId + 1, 0.0);

    for(const Edge::ID &id: path) {
        edges.insert(id);
        flows[id] += delta;
    }
}
