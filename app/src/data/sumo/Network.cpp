#include "data/sumo/Network.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>

#include "utils/invertMap.hpp"
#include "utils/io.hpp"
#include "utils/stringifier.hpp"

using namespace std;
using namespace rapidxml;

typedef SUMO::Shape Shape;
typedef SUMO::Network::Junction Junction;
typedef SUMO::Network::Edge Edge;
typedef SUMO::Network::Edge::Lane Lane;
typedef SUMO::Network::TrafficLightLogic TrafficLightLogic;
typedef SUMO::Network::Connection Connection;

using utils::stringifier;

const Junction::ID Junction::INVALID = "";

const unordered_map<string, Edge::Function> str2function = {
    {"internal", Edge::Function::INTERNAL},
    {"connector", Edge::Function::CONNECTOR},
    {"crossing", Edge::Function::CROSSING},
    {"walkingarea", Edge::Function::WALKINGAREA},
    {"normal", Edge::Function::NORMAL}};
const unordered_map<Edge::Function, string> function2str = utils::invertMap(str2function);

const unordered_map<string, Junction::Type> str2junctionType = {
    {"priority"                     , Junction::Type::PRIORITY                   },
    {"traffic_light"                , Junction::Type::TRAFFIC_LIGHT              },
    {"right_before_left"            , Junction::Type::RIGHT_BEFORE_LEFT          },
    {"left_before_right"            , Junction::Type::LEFT_BEFORE_RIGHT          },
    {"unregulated"                  , Junction::Type::UNREGULATED                },
    {"traffic_light_unregulated"    , Junction::Type::TRAFFIC_LIGHT_UNREGULATED  },
    {"priority_stop"                , Junction::Type::PRIORITY_STOP              },
    {"allway_stop"                  , Junction::Type::ALLWAY_STOP                },
    {"rail_signal"                  , Junction::Type::RAIL_SIGNAL                },
    {"zipper"                       , Junction::Type::ZIPPER                     },
    {"rail_crossing"                , Junction::Type::RAIL_CROSSING              },
    {"traffic_light_right_on_red"   , Junction::Type::TRAFFIC_LIGHT_RIGHT_ON_RED },
    {"dead_end"                     , Junction::Type::DEAD_END                   },

    {"internal"                     , Junction::Type::INTERNAL                   },

    {"unknown"                      , Junction::Type::UNKNOWN                    },
    {"district"                     , Junction::Type::DISTRICT                   }
};
const unordered_map<Junction::Type, string> junctionType2str = utils::invertMap(str2junctionType);

const unordered_map<string, TrafficLightLogic::Type> str2tlType = {
    {"static", TrafficLightLogic::Type::STATIC},
    {"actuated", TrafficLightLogic::Type::ACTUATED},
    {"delay_based", TrafficLightLogic::Type::DELAY_BASED}};
const unordered_map<TrafficLightLogic::Type, string> tlType2str = utils::invertMap(str2tlType);

const unordered_map<string, TrafficLightLogic::Phase::State> str2tlState = {
    {"r", TrafficLightLogic::Phase::State::RED},
    {"y", TrafficLightLogic::Phase::State::YELLOW_STOP},
    {"g", TrafficLightLogic::Phase::State::GREEN_NOPRIORITY},
    {"G", TrafficLightLogic::Phase::State::GREEN_PRIORITY},
    {"s", TrafficLightLogic::Phase::State::GREEN_RIGHT},
    {"u", TrafficLightLogic::Phase::State::YELLOW_START},
    {"o", TrafficLightLogic::Phase::State::OFF_YIELD},
    {"O", TrafficLightLogic::Phase::State::OFF}};
const unordered_map<TrafficLightLogic::Phase::State, string> tlState2str = utils::invertMap(str2tlState);

const unordered_map<string, Connection::Direction> str2connDir = {
    {"invalid", Connection::Direction::INVALID},
    {"s", Connection::Direction::STRAIGHT},
    {"t", Connection::Direction::TURN},
    {"l", Connection::Direction::LEFT},
    {"r", Connection::Direction::RIGHT},
    {"L", Connection::Direction::PARTIALLY_LEFT},
    {"R", Connection::Direction::PARTIALLY_RIGHT}};
const unordered_map<Connection::Direction, string> connDir2str = utils::invertMap(str2connDir);

const unordered_map<string, Connection::State> str2connState = {
    {"-", Connection::State::DEAD_END},
    {"=", Connection::State::EQUAL},
    {"m", Connection::State::MINOR_LINK},
    {"M", Connection::State::MAJOR_LINK},
    {"O", Connection::State::CONTROLLER_OFF},
    {"o", Connection::State::YELLOW_FLASHING},
    {"y", Connection::State::YELLOW_MINOR_LINK},
    {"Y", Connection::State::YELLOW_MAJOR_LINK},
    {"r", Connection::State::RED},
    {"g", Connection::State::GREEN_MINOR},
    {"G", Connection::State::GREEN_MAJOR}};
const unordered_map<Connection::State, string> connState2str = utils::invertMap(str2connState);

Edge::Function stringifier<Edge::Function>::fromString(const string &s) {
    auto it = str2function.find(s);
    if (it != str2function.end())
        return it->second;
    else
        return Edge::Function::NORMAL;
}

string stringifier<Edge::Function>::toString(const Edge::Function &t) {
    return function2str.at(t);
}

Junction::Type stringifier<Junction::Type>::fromString(const string &s) {
    return str2junctionType.at(s);
}

string stringifier<Junction::Type>::toString(const Junction::Type &t) {
    return junctionType2str.at(t);
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
    for (const char &c : s) {
        ret.emplace_back(stringifier<TrafficLightLogic::Phase::State>::fromString(string(1, c)));
    }
    return ret;
}

string stringifier<vector<TrafficLightLogic::Phase::State>>::toString(const vector<TrafficLightLogic::Phase::State> &t) {
    char *arr = new char[t.size() + 1];
    for (size_t i = 0; i < t.size(); ++i) {
        string s = stringifier<TrafficLightLogic::Phase::State>::toString(t[i]);
        if (s.size() != 1)
            throw logic_error("Stringification of tlLogic::Phase::State should always have only 1 char");
        arr[i] = s[0];
    }
    arr[t.size()] = '\0';
    return string(arr);
}

Connection::Direction stringifier<Connection::Direction>::fromString(const string &s) {
    return str2connDir.at(s);
}

string stringifier<Connection::Direction>::toString(const Connection::Direction &t) {
    return connDir2str.at(t);
}

Connection::State stringifier<Connection::State>::fromString(const string &s) {
    return str2connState.at(s);
}

string stringifier<Connection::State>::toString(const Connection::State &t) {
    return connState2str.at(t);
}

Edge SUMO::Network::loadEdge(const xml_node<> *it) const {
    Edge edge;

    edge.id = it->first_attribute("id")->value();
    {
        auto *fromAttr = it->first_attribute("from");
        if (fromAttr) edge.from = fromAttr->value();
    }
    {
        auto *toAttr = it->first_attribute("to");
        if (toAttr) edge.to = toAttr->value();
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

Junction SUMO::Network::loadJunction(const xml_node<> *it) const {
    Junction junction;
    junction.id = it->first_attribute("id")->value();

    {
        auto *typeAttr = it->first_attribute("type");
        if (typeAttr) junction.type = stringifier<Junction::Type>::fromString(typeAttr->value());
    }

    junction.pos = Coord(
        stringifier<double>::fromString(it->first_attribute("x")->value()),
        stringifier<double>::fromString(it->first_attribute("y")->value()));
    
    junction.incLanes = stringifier<vector<Lane::ID>>::fromString(it->first_attribute("incLanes")->value());
    junction.intLanes = stringifier<vector<Lane::ID>>::fromString(it->first_attribute("intLanes")->value());

    // Check incLanes/intLanes are valid
    for(const Lane::ID &laneId: junction.incLanes)
        lanes.at(laneId);
    for(const Lane::ID &laneId: junction.intLanes)
        lanes.at(laneId);
    
    {
        auto *shapeAttr = it->first_attribute("shape");
        if (shapeAttr) junction.shape = stringifier<SUMO::Shape>::fromString(it->first_attribute("shape")->value());
    }

    return junction;
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
        if (tlLogic.phases.empty())
            tPrev = 0;
        else
            tPrev = tlLogic.phases.rbegin()->first;

        tlLogic.phases[tPrev + phase.duration] = phase;
    }

    return tlLogic;
}

Connection SUMO::Network::loadConnection(const xml_node<> *it) const {
    Connection connection;

    connection.from = it->first_attribute("from")->value();
    connection.to   = it->first_attribute("to"  )->value();

    connection.fromLane = stringifier<int>::fromString(it->first_attribute("fromLane")->value());
    connection.toLane   = stringifier<int>::fromString(it->first_attribute("toLane"  )->value());

    edges.at(connection.from).lanes.at(connection.fromLane);
    edges.at(connection.to).lanes.at(connection.toLane);

    connection.dir = stringifier<Connection::Direction>::fromString(it->first_attribute("dir")->value());
    connection.state = stringifier<Connection::State>::fromString(it->first_attribute("state")->value());

    {
        auto *viaAttr = it->first_attribute("via");
        if (viaAttr) {
            connection.via = it->first_attribute("via")->value();
            lanes.at(connection.via);
        }
    }
    {
        auto *tlAttr = it->first_attribute("tl");
        if (tlAttr) connection.tl = tlAttr->value();
    }
    {
        auto *linkIndexAttr = it->first_attribute("tl");
        if (linkIndexAttr) connection.linkIndex = stringifier<int>::fromString(linkIndexAttr->value());
    }

    return connection;
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

    // Edges
    for (auto it = net.first_node("edge"); it; it = it->next_sibling("edge")) {
        Edge edge = network.loadEdge(it);
        network.edges[edge.id] = edge;
        for (const auto &p : edge.lanes) {
            const Lane &lane = p.second;
            network.lanes[lane.id] = make_pair(edge.id, lane.index);
        }
    }

    // Junctions
    for (auto it = net.first_node("junction"); it; it = it->next_sibling("junction")) {
        Junction junction = network.loadJunction(it);
        network.junctions[junction.id] = junction;
    }

    // Check edge.from/to are valid junctions
    for(const auto &p: network.edges){
        const Edge &edge = p.second;
        if(edge.from != Junction::INVALID) network.junctions.at(edge.from);
        if(edge.to != Junction::INVALID) network.junctions.at(edge.to);
    }

    // Traffic lights
    for (auto it = net.first_node("tlLogic"); it; it = it->next_sibling("tlLogic")) {
        TrafficLightLogic tlLogic = network.loadTrafficLightLogic(it);
        network.trafficLights[tlLogic.id] = tlLogic;
    }

    // Connections
    for (auto it = net.first_node("connection"); it; it = it->next_sibling("connection")) {
        Connection connection = network.loadConnection(it);
        network.connections[connection.from][connection.to].push_back(connection);
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

const Junction &SUMO::Network::getJunction(const Junction::ID &id) const {
    return junctions.at(id);
}

vector<Edge> SUMO::Network::getEdges() const {
    vector<Edge> ret;
    ret.reserve(edges.size());
    for (const auto &p : edges)
        ret.push_back(p.second);
    return ret;
}

const Edge &SUMO::Network::getEdge(const Edge::ID &id) const {
    return edges.at(id);
}

const unordered_map<Edge::ID, unordered_map<Edge::ID, list<Connection>>> &SUMO::Network::getConnections() const {
    return connections;
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