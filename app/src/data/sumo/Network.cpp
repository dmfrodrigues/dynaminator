#include "data/sumo/Network.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>

#include "utils/io.hpp"
#include "utils/stringifier.hpp"
#include "utils/invertMap.hpp"

using namespace std;
using namespace rapidxml;

typedef SUMO::Shape Shape;
typedef SUMO::Network::Junction Junction;
typedef SUMO::Network::Edge Edge;
typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Network::TrafficLightLogic TrafficLightLogic;

using utils::stringifier;

const Junction::ID Junction::INVALID = "";

const unordered_map<string, Edge::Function> str2function = {
    {"internal"     , Edge::Function::INTERNAL      },
    {"connector"    , Edge::Function::CONNECTOR     },
    {"crossing"     , Edge::Function::CROSSING      },
    {"walkingarea"  , Edge::Function::WALKINGAREA   },
    {"normal"       , Edge::Function::NORMAL        }
};
const unordered_map<Edge::Function, string> function2str = utils::invertMap(str2function);

const unordered_map<string, TrafficLightLogic::Type> str2tlType = {
    {"static"       , TrafficLightLogic::Type::STATIC       },
    {"actuated"     , TrafficLightLogic::Type::ACTUATED     },
    {"delay_based"  , TrafficLightLogic::Type::DELAY_BASED  }
};
const unordered_map<TrafficLightLogic::Type, string> tlType2str = utils::invertMap(str2tlType);

const unordered_map<string, TrafficLightLogic::Phase::State> str2tlState = {
    {"r", TrafficLightLogic::Phase::State::RED              },
    {"y", TrafficLightLogic::Phase::State::YELLOW_STOP      },
    {"g", TrafficLightLogic::Phase::State::GREEN_NOPRIORITY },
    {"G", TrafficLightLogic::Phase::State::GREEN_PRIORITY   },
    {"s", TrafficLightLogic::Phase::State::GREEN_RIGHT      },
    {"u", TrafficLightLogic::Phase::State::YELLOW_START     },
    {"o", TrafficLightLogic::Phase::State::OFF_YIELD        },
    {"O", TrafficLightLogic::Phase::State::OFF              }
};
const unordered_map<TrafficLightLogic::Phase::State, string> tlState2str = utils::invertMap(str2tlState);

Edge::Function stringifier<Edge::Function>::fromString(const string &s) {
    auto it = str2function.find(s);
    if(it != str2function.end()) return it->second;
    else return Edge::Function::NORMAL;
}

string stringifier<Edge::Function>::toString(const Edge::Function &t) {
    return function2str.at(t);
}

TrafficLightLogic::Type stringifier<TrafficLightLogic::Type>::fromString(const string &s) {
    return str2tlType.at(s);
}

string stringifier<TrafficLightLogic::Type>::toString(const TrafficLightLogic::Type &t) {
    return tlType2str.at(t);
}

TrafficLightLogic::Phase::State stringifier<TrafficLightLogic::Phase::State>::fromString(const string &s) {
    return str2tlState.at(s);
}

string stringifier<TrafficLightLogic::Phase::State>::toString(const TrafficLightLogic::Phase::State &t) {
    return tlState2str.at(t);
}

vector<TrafficLightLogic::Phase::State>
stringifier<vector<TrafficLightLogic::Phase::State>>::fromString(const string &s) {
    vector<TrafficLightLogic::Phase::State> ret;
    ret.reserve(s.size());
    for(const char &c: s){
        ret.emplace_back(stringifier<TrafficLightLogic::Phase::State>::fromString(string(1, c)));
    }
    return ret;
}

string stringifier<vector<TrafficLightLogic::Phase::State>>::toString(const vector<TrafficLightLogic::Phase::State> &t) {
    char *arr = new char[t.size()+1];
    for(size_t i = 0; i < t.size(); ++i){
        string s = stringifier<TrafficLightLogic::Phase::State>::toString(t[i]);
        if(s.size() != 1)
            throw logic_error("Stringification of tlLogic::Phase::State should always have only 1 char");
        arr[i] = s[0];
    }
    arr[t.size()] = '\0';
    return string(arr);
}

Junction SUMO::Network::loadJunction(const xml_node<> *it) const {
    Junction junction;
    junction.id = it->first_attribute("id")->value();
    junction.pos = Coord(
        stringifier<double>::fromString(it->first_attribute("x")->value()),
        stringifier<double>::fromString(it->first_attribute("y")->value()));
    return junction;
}

Edge SUMO::Network::loadEdge(const xml_node<> *it) const {
    Edge edge;

    edge.id = it->first_attribute("id")->value();
    {
        auto *fromAttr = it->first_attribute("from");
        if (fromAttr) edge.from = junctions.at(fromAttr->value()).id;
    }
    {
        auto *toAttr = it->first_attribute("to");
        if (toAttr) edge.to = junctions.at(toAttr->value()).id;
    }
    {
        auto *priorityAttr = it->first_attribute("priority");
        if (priorityAttr) edge.priority = stringifier<Edge::Priority>::fromString(priorityAttr->value());
    }
    {
        auto *functionAttr = it->first_attribute("function");
        if (functionAttr) edge.function = stringifier<Edge::Function>::fromString(functionAttr->value());
    }
    {
        auto *shapeAttr = it->first_attribute("shape");
        if (shapeAttr) edge.shape = stringifier<Shape>::fromString(shapeAttr->value());
    }

    for (auto it2 = it->first_node("lane"); it2; it2 = it2->next_sibling("lane")) {
        Lane lane;

        lane.id = it2->first_attribute("id")->value();
        lane.index = stringifier<Index>::fromString(it2->first_attribute("index")->value());
        lane.speed = stringifier<Lane::Speed>::fromString(it2->first_attribute("speed")->value());
        lane.length = stringifier<Lane::Length>::fromString(it2->first_attribute("length")->value());
        lane.shape = stringifier<Shape>::fromString(it2->first_attribute("shape")->value());

        if (edge.lanes.count(lane.index)) {
            cerr << "Lane " << lane.id << ", repeated index " << lane.index << endl;
            continue;
        }
        edge.lanes[lane.index] = lane;
    }

    return edge;
}

TrafficLightLogic SUMO::Network::loadTrafficLightLogic(const xml_node<> *it) const {
    TrafficLightLogic tlLogic;

    tlLogic.id = it->first_attribute("id")->value();
    tlLogic.type = stringifier<TrafficLightLogic::Type>::fromString(it->first_attribute("type")->value());
    tlLogic.programId = it->first_attribute("programID")->value();
    tlLogic.offset = stringifier<Time>::fromString(it->first_attribute("offset")->value());

    for (auto it2 = it->first_node("phase"); it2; it2 = it2->next_sibling("phase")) {
        TrafficLightLogic::Phase phase;

        phase.duration = stringifier<Time>::fromString(it2->first_attribute("duration")->value());
        phase.state = stringifier<vector<TrafficLightLogic::Phase::State>>::fromString(it2->first_attribute("state")->value());

        Time tPrev;
        if(tlLogic.phases.empty()) tPrev = 0;
        else tPrev = tlLogic.phases.rbegin()->first;

        tlLogic.phases[tPrev + phase.duration] = phase;
    }

    return tlLogic;
}

SUMO::Network SUMO::Network::loadFromFile(const string &path) {
    SUMO::Network network;

    // Parse XML
    string textStr = utils::readWholeFile(path);
    unique_ptr<char[]> text(new char[textStr.size() + 1]);
    strcpy(text.get(), textStr.c_str());
    xml_document<> doc;
    doc.parse<0>(text.get());

    // Get data from XML parser
    const auto &net = *doc.first_node();

    // Junctions
    for (auto it = net.first_node("junction"); it; it = it->next_sibling("junction")) {
        Junction junction = network.loadJunction(it);
        network.junctions[junction.id] = junction;
    }

    // Edges
    for (auto it = net.first_node("edge"); it; it = it->next_sibling("edge")) {
        Edge edge = network.loadEdge(it);
        network.edges[edge.id] = edge;
    }

    // Traffic lights
    for (auto it = net.first_node("tlLogic"); it; it = it->next_sibling("tlLogic")) {
        TrafficLightLogic tlLogic = network.loadTrafficLightLogic(it);
        network.trafficLights[tlLogic.id] = tlLogic;
    }

    return network;
}

vector<Junction> SUMO::Network::getJunctions() const {
    vector<Junction> ret;
    ret.reserve(junctions.size());
    for (const auto &p : junctions)
        ret.push_back(p.second);
    return ret;
}

vector<Edge> SUMO::Network::getEdges() const {
    vector<Edge> ret;
    ret.reserve(edges.size());
    for (const auto &p : edges)
        ret.push_back(p.second);
    return ret;
}

void SUMO::Network::saveStatsToFile(const string &path) const {
    xml_document<> doc;
    auto meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    list<string> strs;
    for (const auto &p : edges) {
        const SUMO::Network::Edge::ID &eid = p.first;
        const Edge &e = p.second;

        string &ps = (strs.emplace_back() = stringifier<Edge::Priority>::toString(e.priority));
        string &fs = (strs.emplace_back() = stringifier<Edge::Function>::toString(e.function));
        string &lns = (strs.emplace_back() = stringifier<size_t>::toString(e.lanes.size()));

        Lane::Length length = 0;
        Lane::Speed speed = 0;
        for (const auto &l : e.lanes) {
            length += l.second.length;
            speed += l.second.speed;
        }
        length /= (Lane::Length)e.lanes.size();
        speed /= (Lane::Speed)e.lanes.size();
        Lane::Speed speed_kmh = speed * 3.6;

        string &ls = (strs.emplace_back() = stringifier<Lane::Length>::toString(length));
        string &ss = (strs.emplace_back() = stringifier<Lane::Speed>::toString(speed));
        string &kmhs = (strs.emplace_back() = stringifier<Lane::Speed>::toString(speed_kmh));

        auto edge = doc.allocate_node(node_element, "edge");
        edge->append_attribute(doc.allocate_attribute("id", eid.c_str()));
        edge->append_attribute(doc.allocate_attribute("priority", ps.c_str()));
        edge->append_attribute(doc.allocate_attribute("function", fs.c_str()));
        edge->append_attribute(doc.allocate_attribute("lanes", lns.c_str()));
        edge->append_attribute(doc.allocate_attribute("length", ls.c_str()));
        edge->append_attribute(doc.allocate_attribute("speed", ss.c_str()));
        edge->append_attribute(doc.allocate_attribute("speed_kmh", kmhs.c_str()));
        interval->append_node(edge);
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    os.open(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
