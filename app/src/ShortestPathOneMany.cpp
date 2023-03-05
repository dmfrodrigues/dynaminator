#include "ShortestPathOneMany.hpp"

using namespace std;

ShortestPathOneMany::~ShortestPathOneMany(){}

list<Graph::Node> ShortestPathOneMany::getPath(Graph::Node d) const{
    list<Graph::Node> res;
    while(d != getStart()){
        res.push_front(d);
        d = getPrev(d);
        if(d == Graph::NODE_INVALID) return list<Graph::Node>();
    }
    res.push_front(d);
    return res;
}
