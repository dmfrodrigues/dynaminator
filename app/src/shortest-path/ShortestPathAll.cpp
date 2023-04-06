#include "shortest-path/ShortestPathAll.hpp"

#include <list>

using namespace std;

ShortestPathAll::~ShortestPathAll(){}

Graph::Path ShortestPathAll::getPath(Graph::Node s, Graph::Node d) const{
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
