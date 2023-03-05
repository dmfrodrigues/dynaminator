#include "ShortestPathOneMany.hpp"

#include <list>

using namespace std;

ShortestPathOneMany::~ShortestPathOneMany(){}

vector<Graph::Node> ShortestPathOneMany::getPath(Graph::Node d) const{
    list<Graph::Node> res;
    while(d != getStart()){
        res.push_front(d);
        d = getPrev(d);
        if(d == Graph::NODE_INVALID) return vector<Graph::Node>();
    }
    res.push_front(d);
    return vector<Graph::Node>(res.begin(), res.end());
}
