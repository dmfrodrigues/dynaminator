#include "static/algos/DijkstraAoN.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include "shortest-path/DijkstraMany.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Flow Flow;

StaticSolutionBase DijkstraAoN::solve(const StaticProblem &problem, const StaticSolution &x0) {
    Graph G = problem.supply.toGraph(x0);

    const vector<Node> startNodes = problem.demand.getStartNodes();

    DijkstraMany shortestPaths;
    shortestPaths.solve(&G, startNodes);

    StaticSolutionBase x;

    for (const Node &u : startNodes) {
        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for (const Node &v : endNodes) {
            Graph::Path path = shortestPaths.getPath(u, v);

            if (path.size() == 1 && path.front().id == Graph::EDGE_INVALID.id)
                throw logic_error("Could not find path " + to_string(u) + "->" + to_string(v));

            StaticNetwork::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for (const Graph::Edge &e : path)
                pathNetwork.push_back(e.id);

            Flow f = problem.demand.getDemand(u, v);
            x.addPath(pathNetwork, f);
        }
    }

    return x;
}
