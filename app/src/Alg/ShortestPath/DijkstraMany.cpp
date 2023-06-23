#include "Alg/ShortestPath/DijkstraMany.hpp"

#include <stdexcept>

using namespace std;
using namespace Alg;
using namespace Alg::ShortestPath;

typedef Graph::Node         Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge         Edge;

DijkstraMany::DijkstraMany(int parallelism):
    pool(parallelism) {}

void DijkstraMany::solve(const Graph &G, const vector<Node> &s_) {
    // Initialize
    for(const Node &s: s_)
        dijkstras[s];

    // Run
    if(pool.size() <= 0) {
        for(auto &[s, dijkstra]: dijkstras) {
            dijkstra.solve(G, s);
        }
        return;
    }

    vector<future<void>> results;
    for(auto &p: dijkstras) {
        const Node &s        = p.first;
        Dijkstra   &dijkstra = p.second;
        results.emplace_back(pool.push([&dijkstra, &G, s](int) -> void {
            dijkstra.solve(G, s);
        }));
    }
    for(future<void> &r: results) r.get();
}

Edge DijkstraMany::getPrev(Node s, Node d) const {
    try {
        return dijkstras.at(s).getPrev(d);
    } catch(out_of_range &e) {
        throw out_of_range("DijkstraMany::getPrev: No path from " + to_string(s) + " to " + to_string(d));
    }
}

Weight DijkstraMany::getPathWeight(Node s, Node d) const {
    try {
        return dijkstras.at(s).getPathWeight(d);
    } catch(out_of_range &e) {
        throw out_of_range("DijkstraMany::getPathWeight: No path from " + to_string(s) + " to " + to_string(d));
    }
}

bool DijkstraMany::hasVisited(Node s, Node u) const {
    auto it = dijkstras.find(s);
    if(it == dijkstras.end()) return false;
    return it->second.hasVisited(u);
}
