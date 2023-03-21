#include "static/algos/AllOrNothing.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include "shortest-path/Dijkstra.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Flow Flow;

AllOrNothing::AllOrNothing(const StaticProblem &prob)
    : problem(prob) {}

StaticSolution AllOrNothing::solve() {
    StaticSolution xn;
    Graph G = problem.supply.toGraph(xn);

    const vector<Node> startNodes = problem.demand.getStartNodes();
    for(const Node &u: startNodes){
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());
        sp.get()->initialize(&G, u);
        sp.get()->run();

        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for(const Node &v: endNodes){
            Graph::Path path = sp.get()->getPath(v);
            if (path.front().u != u || path.back().v != v)
                throw logic_error("There is no path " + to_string(u) + "->" + to_string(v));

            StaticNetwork::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for(const Graph::Edge &e: path)
                pathNetwork.push_back(e.id);

            Flow f = problem.demand.getDemand(u, v);
            xn.addPath(pathNetwork, f);
        }
    }

    return xn;
}
