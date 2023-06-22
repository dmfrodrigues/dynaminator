#include "data/VISUM/OFormatDemand.hpp"

#include <fstream>
#include <iostream>

using namespace std;
using namespace VISUM;

typedef OFormatDemand::Node Node;
typedef OFormatDemand::Flow Flow;
typedef OFormatDemand::Time Time;

const double HOUR_TO_SECOND   = 60 * 60;
const double MINUTE_TO_SECOND = 60;

Time OFormatDemand::getFrom() const { return from; }
Time OFormatDemand::getTo() const { return to; }

void OFormatDemand::addDemand(Node u, Node v, Flow f) {
    flows[u][v] += f;
}

vector<Node> OFormatDemand::getStartNodes() const {
    vector<Node> ret;
    ret.reserve(flows.size());

    for(const auto &[u, _]: flows)
        ret.push_back(u);

    return ret;
}

vector<Node> OFormatDemand::getDestinations(Node u) const {
    const auto &dest = flows.at(u);

    vector<Node> ret;
    ret.reserve(dest.size());

    for(const auto &[v, _]: dest)
        ret.push_back(v);

    return ret;
}

Flow OFormatDemand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}

void ignoreCommentsOFile(istream &is) {
    string line;
    while(is.peek() == '\r' || is.peek() == '\n' || is.peek() == '*')
        getline(is, line);
}

pair<int, int> parseTime(const string &t) {
    size_t i = t.find(".");
    return make_pair(
        stoi(t.substr(0, i)),
        stoi(t.substr(i + 1))
    );
}

OFormatDemand OFormatDemand::loadFromFile(const string &path) {
    OFormatDemand demand;

    ifstream is;
    is.exceptions(ios_base::failbit | ios_base::badbit);
    try {
        is.open(path);
    } catch(const ios_base::failure &e) {
        throw ios_base::failure("Could not open file " + path);
    }
    if(is.peek() != '$') throw ios_base::failure("File is not VISUM");
    is.ignore(1);
    if(is.peek() != 'O') throw ios_base::failure("File is not O-formatted");

    string line;
    getline(is, line);
    ignoreCommentsOFile(is);

    string fromStr, toStr;
    is >> fromStr >> toStr;
    pair<int, int>
        fromPair = parseTime(fromStr),
        toPair   = parseTime(toStr);
    demand.from =
        fromPair.first * HOUR_TO_SECOND + fromPair.second * MINUTE_TO_SECOND;
    demand.to =
        toPair.first * HOUR_TO_SECOND + toPair.second * MINUTE_TO_SECOND;

    ignoreCommentsOFile(is);

    is >> demand.factor;

    ignoreCommentsOFile(is);

    is.exceptions(ios_base::badbit);

    Node u, v;
    Flow f;
    while(is >> u >> v >> f) {
        ignoreCommentsOFile(is);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if(f == 0.0) continue;
#pragma GCC diagnostic pop

        demand.addDemand(u, v, f);
    }

    return demand;
}
