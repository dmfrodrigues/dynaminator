#include "data/SUMO/Network.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <sstream>

#include "utils/io.hpp"
#include "utils/stringify.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO;
using namespace Geo;
using namespace utils::stringify;

typedef Network::Junction          Junction;
typedef Network::Edge              Edge;
typedef Network::Edge::Lane        Lane;
typedef Network::TrafficLightLogic TrafficLightLogic;
typedef Network::TrafficLights     TrafficLights;
typedef Network::Connection        Connection;
typedef Network::Connections       Connections;

const Junction::ID Junction::INVALID = "";

const Lane &Connection::fromLane() const {
    return from.lanes.at(fromLaneIndex);
}

const Lane &Connection::toLane() const {
    return from.lanes.at(toLaneIndex);
}

Length Edge::length() const {
    Length length = 0;
    for(const auto &[laneIndex, lane]: lanes) {
        length += lane.length;
    }
    length /= (Length)lanes.size();
    return length;
}

Speed Edge::speed() const {
    Speed speed = 0;
    for(const auto &[laneIndex, lane]: lanes) {
        speed += lane.speed;
    }
    speed /= (Speed)lanes.size();
    return speed;
}

Time TrafficLightLogic::getGreenTime(size_t linkIndex) const {
    Time t = 0.0;
    for(const auto &p: phases) {
        const TrafficLightLogic::Phase &phase = p.second;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
        switch(phase.state.at(linkIndex)) {
            case Phase::YELLOW_STOP:
            case Phase::GREEN_NOPRIORITY:
            case Phase::GREEN_PRIORITY:
            case Phase::GREEN_RIGHT:
            case Phase::OFF_YIELD:
            case Phase::OFF:
                t += phase.duration;
                break;
            default:
                break;
        }
#pragma GCC diagnostic pop
    }
    return t;
}
Time TrafficLightLogic::getCycleTime() const {
    Time t = 0.0;
    for(const auto &p: phases) {
        const TrafficLightLogic::Phase &phase = p.second;
        t += phase.duration;
    }
    return t;
}
size_t TrafficLightLogic::getNumberStops(size_t linkIndex) const {
    size_t n = 0;

    bool previousStateGo = (phases.rbegin()->second.state.at(linkIndex) != Phase::RED);

    for(const auto &[t, phase]: phases) {
        bool currentStateGo = (phase.state.at(linkIndex) != Phase::RED);
        if(!previousStateGo && currentStateGo)
            ++n;
        previousStateGo = currentStateGo;
    }
    return n;
}

Edge Network::loadEdge(const xml_node<> *it) const {
    Edge edge;

    edge.id = it->first_attribute("id")->value();
    {
        auto *fromAttr = it->first_attribute("from");
        if(fromAttr) edge.fromID = fromAttr->value();
    }
    {
        auto *toAttr = it->first_attribute("to");
        if(toAttr) edge.toID = toAttr->value();
    }
    {
        auto *priorityAttr = it->first_attribute("priority");
        if(priorityAttr) edge.priority = stringify<Edge::Priority>::fromString(priorityAttr->value());
    }
    {
        auto *functionAttr = it->first_attribute("function");
        if(functionAttr) edge.function = stringify<Edge::Function>::fromString(functionAttr->value());
    }
    {
        auto *shapeAttr = it->first_attribute("shape");
        if(shapeAttr) edge.shape = stringify<Shape>::fromString(shapeAttr->value());
    }

    for(auto it2 = it->first_node("lane"); it2; it2 = it2->next_sibling("lane")) {
        Lane lane;

        lane.id     = it2->first_attribute("id")->value();
        lane.index  = stringify<Index>::fromString(it2->first_attribute("index")->value());
        lane.speed  = stringify<Speed>::fromString(it2->first_attribute("speed")->value());
        lane.length = stringify<Length>::fromString(it2->first_attribute("length")->value());
        lane.shape  = stringify<Shape>::fromString(it2->first_attribute("shape")->value());

        if(edge.lanes.count(lane.index)) {
            cerr << "Lane " << lane.id << ", repeated index " << lane.index << endl;
            continue;
        }
        edge.lanes[lane.index] = lane;
    }

    return edge;
}

Junction Network::loadJunction(const xml_node<> *it) const {
    Junction junction;
    junction.id = it->first_attribute("id")->value();

    {
        auto *typeAttr = it->first_attribute("type");
        if(typeAttr) junction.type = stringify<Junction::Type>::fromString(typeAttr->value());
    }

    junction.pos = Coord(
        stringify<double>::fromString(it->first_attribute("x")->value()),
        stringify<double>::fromString(it->first_attribute("y")->value())
    );

    const vector<Lane::ID> incLanes = stringify<vector<Lane::ID>>::fromString(it->first_attribute("incLanes")->value());
    const vector<Lane::ID> intLanes = stringify<vector<Lane::ID>>::fromString(it->first_attribute("intLanes")->value());

    auto f = [this](const Lane::ID &id) -> const Lane * {
        const auto &[edgeID, laneIndex] = lanes.at(id);
        return &edges.at(edgeID).lanes.at(laneIndex);
    };

    junction.incLanes.clear();
    junction.incLanes.reserve(incLanes.size());
    transform(incLanes.begin(), incLanes.end(), back_inserter(junction.incLanes), f);

    junction.intLanes.clear();
    junction.intLanes.reserve(intLanes.size());
    transform(intLanes.begin(), intLanes.end(), back_inserter(junction.intLanes), f);

    {
        auto *shapeAttr = it->first_attribute("shape");
        if(shapeAttr) junction.shape = stringify<Shape>::fromString(it->first_attribute("shape")->value());
    }

    return junction;
}

TrafficLightLogic Network::loadTrafficLightLogic(const xml_node<> *it) const {
    TrafficLightLogic tlLogic;

    tlLogic.id        = it->first_attribute("id")->value();
    tlLogic.type      = stringify<TrafficLightLogic::Type>::fromString(it->first_attribute("type")->value());
    tlLogic.programId = it->first_attribute("programID")->value();
    tlLogic.offset    = stringify<Time>::fromString(it->first_attribute("offset")->value());

    for(auto it2 = it->first_node("phase"); it2; it2 = it2->next_sibling("phase")) {
        TrafficLightLogic::Phase phase;

        phase.duration = stringify<Time>::fromString(it2->first_attribute("duration")->value());
        phase.state    = stringify<vector<TrafficLightLogic::Phase::State>>::fromString(it2->first_attribute("state")->value());

        Time tPrev;
        if(tlLogic.phases.empty())
            tPrev = 0;
        else
            tPrev = tlLogic.phases.rbegin()->first;

        tlLogic.phases[tPrev + phase.duration] = phase;
    }

    return tlLogic;
}

Connection Network::loadConnection(const xml_node<> *it) const {
    // clang-format off
    Connection connection{
        edges.at(it->first_attribute("from")->value()),
        edges.at(it->first_attribute("to")->value()),
        stringify<Index>::fromString(it->first_attribute("fromLane")->value()),
        stringify<Index>::fromString(it->first_attribute("toLane")->value()),
        stringify<Connection::Direction>::fromString(it->first_attribute("dir")->value()),
        stringify<Connection::State>::fromString(it->first_attribute("state")->value())
    };
    // clang-format on

    connection.from.lanes.at(connection.fromLaneIndex);
    connection.to.lanes.at(connection.toLaneIndex);

    {
        auto *viaAttr = it->first_attribute("via");
        if(viaAttr) {
            const auto &[edgeID, laneIndex] = lanes.at(it->first_attribute("via")->value());
            connection.via                  = &edges.at(edgeID).lanes.at(laneIndex);
        }
    }
    {
        auto *tlAttr = it->first_attribute("tl");
        if(tlAttr) connection.tl = &trafficLights.at(tlAttr->value());
    }
    {
        auto *linkIndexAttr = it->first_attribute("linkIndex");
        if(linkIndexAttr) connection.linkIndex = stringify<int>::fromString(linkIndexAttr->value());
    }

    return connection;
}

Network Network::loadFromFile(const string &path) {
    Network network;

    // Parse XML
    string             textStr = utils::readWholeFile(path);
    unique_ptr<char[]> text(new char[textStr.size() + 1]);
    strcpy(text.get(), textStr.c_str());
    xml_document<> doc;
    doc.parse<0>(text.get());

    // Get data from XML parser
    const auto &net = *doc.first_node();

    // Edges
    for(auto it = net.first_node("edge"); it; it = it->next_sibling("edge")) {
        Edge edge              = network.loadEdge(it);
        network.edges[edge.id] = edge;
        for(const auto &p: edge.lanes) {
            const Lane &lane       = p.second;
            network.lanes[lane.id] = make_pair(edge.id, lane.index);
        }
    }

    // Junctions
    for(auto it = net.first_node("junction"); it; it = it->next_sibling("junction")) {
        Junction junction              = network.loadJunction(it);
        network.junctions[junction.id] = junction;
    }

    // Correct edge.from/to
    for(auto &[edgeID, edge]: network.edges) {
        if(edge.fromID != Junction::INVALID) edge.from = &network.junctions.at(edge.fromID);
        if(edge.toID != Junction::INVALID) edge.to = &network.junctions.at(edge.toID);
    }

    // Traffic lights
    for(auto it = net.first_node("tlLogic"); it; it = it->next_sibling("tlLogic")) {
        TrafficLightLogic tlLogic         = network.loadTrafficLightLogic(it);
        network.trafficLights[tlLogic.id] = tlLogic;
    }

    // Connections
    for(auto it = net.first_node("connection"); it; it = it->next_sibling("connection")) {
        Connection c = network.loadConnection(it);
        network.connections[c.from.id][c.to.id].push_back(c);
    }

    return network;
}

vector<Junction> Network::getJunctions() const {
    vector<Junction> ret;
    ret.reserve(junctions.size());
    for(const auto &p: junctions)
        ret.push_back(p.second);
    return ret;
}

const Junction &Network::getJunction(const Junction::ID &id) const {
    return junctions.at(id);
}

vector<Edge> Network::getEdges() const {
    vector<Edge> ret;
    ret.reserve(edges.size());
    for(const auto &p: edges)
        ret.push_back(p.second);
    return ret;
}

const Edge &Network::getEdge(const Edge::ID &id) const {
    return edges.at(id);
}

const Connections &Network::getConnections() const {
    return connections;
}

const TrafficLights &Network::getTrafficLights() const {
    return trafficLights;
}

void Network::saveStatsToFile(const string &path) const {
    xml_document<> doc;
    auto           meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    list<string> strs;
    for(const auto &[eid, e]: edges) {
        string &ps  = (strs.emplace_back() = stringify<Edge::Priority>::toString(e.priority));
        string &fs  = (strs.emplace_back() = stringify<Edge::Function>::toString(e.function));
        string &lns = (strs.emplace_back() = stringify<size_t>::toString(e.lanes.size()));

        Length length = 0;
        Speed  speed  = 0;
        for(const auto &[laneIndex, lane]: e.lanes) {
            length += lane.length;
            speed += lane.speed;
        }
        length /= (Length)e.lanes.size();
        speed /= (Speed)e.lanes.size();
        Speed speed_kmh = speed * 3.6;

        string &ls   = (strs.emplace_back() = stringify<Length>::toString(length));
        string &ss   = (strs.emplace_back() = stringify<Speed>::toString(speed));
        string &kmhs = (strs.emplace_back() = stringify<Speed>::toString(speed_kmh));

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
