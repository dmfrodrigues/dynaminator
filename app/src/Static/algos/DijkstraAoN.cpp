#include "Static/algos/DijkstraAoN.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include "Alg/ShortestPath/DijkstraMany.hpp"

using namespace std;
using namespace Static;
using namespace Alg;
using namespace Alg::ShortestPath;

typedef Network::Node Node;
typedef Network::Flow Flow;

SolutionBase DijkstraAoN::solve(
    const Network &supply,
    const Demand &demand,
    const Solution &x0
) {
    Graph G = supply.toGraph(x0);

    const vector<Node> startNodes = demand.getStartNodes();

    DijkstraMany shortestPaths;
    shortestPaths.solve(&G, startNodes);

    SolutionBase x;

    for (const Node &u : startNodes) {
        const vector<Node> endNodes = demand.getDestinations(u);
        for (const Node &v : endNodes) {
            Graph::Path path = shortestPaths.getPath(u, v);

            if (path.size() == 1 && path.front().id == Graph::EDGE_INVALID.id)
                throw logic_error("Could not find path " + to_string(u) + "->" + to_string(v));

            Network::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for (const Graph::Edge &e : path)
                pathNetwork.push_back(e.id);

            Flow f = demand.getDemand(u, v);
            x.addPath(pathNetwork, f);
        }
    }

    return x;
}
