#include "shortest-path/ShortestPathManyMany.hpp"

#include <list>

using namespace std;

ShortestPathManyMany::~ShortestPathManyMany(){}

Graph::Path ShortestPathManyMany::getPath(Graph::Node s, Graph::Node d) const{
    if(d == s) return Graph::Path();
    list<Graph::Edge> res;
    Graph::Edge e = getPrev(s, d);
    if(e.u == Graph::NODE_INVALID) return Graph::Path({Graph::EDGE_INVALID});
    while(e.u != s){
        res.push_front(e);
        e = getPrev(s, e.u);
    }
    res.push_front(e);
    return Graph::Path(res.begin(), res.end());
}
