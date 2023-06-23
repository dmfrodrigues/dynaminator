#include "Alg/ShortestPath/ShortestPathOneMany.hpp"

#include <list>

using namespace std;
using namespace Alg;
using namespace Alg::ShortestPath;

ShortestPathOneMany::~ShortestPathOneMany() {}

void ShortestPathOneMany::solve(const Graph &G, Graph::Node s) {
    list<Graph::Node> sList = {s};
    solveList(G, sList);
}

Graph::Path ShortestPathOneMany::getPath(Graph::Node d) const {
    if(isStart(d)) return Graph::Path();

    list<Graph::Edge> res;

    Graph::Edge e = getPrev(d);

    if(e.u == Graph::NODE_INVALID) return Graph::Path({Graph::EDGE_INVALID});

    while(!isStart(e.u)) {
        res.push_front(e);
        e = getPrev(e.u);
    }
    res.push_front(e);
    return Graph::Path(res.begin(), res.end());
}
