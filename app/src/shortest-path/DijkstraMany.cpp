#include "shortest-path/DijkstraMany.hpp"

using namespace std;

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;

DijkstraMany::DijkstraMany(int parallelism):
    pool(parallelism)
{}

void DijkstraMany::initialize(const Graph *G, const vector<Node> &s_) {
    for(const Node &s: s_){
        dijkstras[s].initialize(G, s);
    }
}

void DijkstraMany::run() {
    if(pool.size() <= 0){
        for (auto &p : dijkstras) {
            Dijkstra &dijkstra = p.second;
            dijkstra.run();
        }
        return;
    }

    vector<future<void>> results;
    for (auto &p : dijkstras) {
        const Node &u = p.first;
        Dijkstra &dijkstra = p.second;
        results.emplace_back(pool.push([&dijkstra, u](int) {
            dijkstra.run();
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
