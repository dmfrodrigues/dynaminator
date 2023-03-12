#include "data/OFormatDemand.hpp"

#include <fstream>

using namespace std;

typedef OFormatDemand::Node Node;
typedef OFormatDemand::Flow Flow;

const double HOUR_TO_SECOND = 60 * 60;
const double MINUTE_TO_SECOND = 60;

void OFormatDemand::addDemand(Node u, Node v, Flow f) {
    flows[u][v] += f;
}

vector<Node> OFormatDemand::getStartNodes() const {
    vector<Node> ret;
    ret.reserve(flows.size());

    for (const auto &p : flows)
        ret.push_back(p.first);

    return ret;
}

vector<Node> OFormatDemand::getDestinations(Node u) const {
    const auto &dest = flows.at(u);

    vector<Node> ret;
    ret.reserve(dest.size());

    for (const auto &p : dest)
        ret.push_back(p.first);

    return ret;
}

Flow OFormatDemand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}


void ignoreCommentsOFile(istream &is) {
    string line;
    while (is.peek() == '*')
        getline(is, line);
}

pair<int, int> parseTime(const std::string &t) {
    size_t i = t.find(".");
    return make_pair(
        stoi(t.substr(0, i)),
        stoi(t.substr(i + 1)));
}

OFormatDemand OFormatDemand::loadFromFile(const string &path) {
    OFormatDemand demand;

    ifstream is(path);
    if (is.peek() != '$') throw ios_base::failure("File is not VISUM");
    is.ignore(1);
    if (is.peek() != 'O') throw ios_base::failure("File is not O-formatted");

    string line;

    ignoreCommentsOFile(is);

    getline(is, line);
    ignoreCommentsOFile(is);

    string fromStr, toStr;
    is >> fromStr >> toStr;
    pair<int, int>
        fromPair = parseTime(fromStr),
        toPair = parseTime(toStr);
    demand.from =
        fromPair.first * HOUR_TO_SECOND +
        fromPair.second * MINUTE_TO_SECOND;
    demand.to =
        toPair.first * HOUR_TO_SECOND +
        toPair.second * MINUTE_TO_SECOND;

    ignoreCommentsOFile(is);

    is >> demand.factor;

    ignoreCommentsOFile(is);

    Node u, v; Flow f;
    while (is >> u >> v >> f) {
        ignoreCommentsOFile(is);

        demand.addDemand(u, v, f);
    }

    return demand;
}
