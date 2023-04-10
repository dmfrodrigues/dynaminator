#include "shortest-path/DijkstraMany.hpp"

using namespace std;

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;

DijkstraMany::DijkstraMany(int parallelism):
    pool(parallelism)
{}

void DijkstraMany::solve(const Graph *G, const vector<Node> &s_) {
    // Initialize
    for(const Node &s: s_)
        dijkstras[s];

    // Run
    if(pool.size() <= 0){
        for (auto &[s, dijkstra]: dijkstras) {
            dijkstra.solve(G, s);
        }
        return;
    }

    vector<future<void>> results;
    for (auto &p : dijkstras) {
        const Node &s = p.first;
        Dijkstra &dijkstra = p.second;
        results.emplace_back(pool.push([&dijkstra, G, s](int) {
            dijkstra.solve(G, s);
        }));
    }
    for (future<void> &r : results) r.get();
}

Edge DijkstraMany::getPrev(Node s, Node d) const {
    return dijkstras.at(s).getPrev(d);
}

Weight DijkstraMany::getPathWeight(Node s, Node d) const {
    return dijkstras.at(s).getPathWeight(d);
}

bool DijkstraMany::hasVisited(Node s, Node u) const {
    return dijkstras.at(s).hasVisited(u);
}
