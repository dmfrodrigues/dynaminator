#include "data/SUMO/Network.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <ios>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <rapidxml_utils.hpp>
#include <sstream>
#include <stdexcept>

#include "utils/stringify.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO;
using namespace utils::stringify;

typedef Network::Junction          Junction;
typedef Network::Junction::Request Request;
typedef Network::Edge              Edge;
typedef Network::Edge::Lane        Lane;
typedef Network::TrafficLightLogic TrafficLightLogic;
typedef Network::TrafficLights     TrafficLights;
typedef Network::Connection        Connection;

const Junction::ID Junction::INVALID = "";

const Edge &Lane::parent() const {
    return net.edges.at(edgeID);
}

Vector2 Lane::getIncomingDirection() const {
    const Coord &p1 = parent().from->pos;
    const Coord &p2 = shape.front();
    return (p2 - p1);
}

Vector2 Lane::getOutgoingDirection() const {
    const Coord &p1 = shape.back();
    const Coord &p2 = parent().to->pos;
    return (p2 - p1);
}

const Edge &Lane::edge() const {
    return net.getEdge(edgeID);
}

Shape Lane::getShape() const {
    if(!shape.empty())
        return shape;

    return edge().getShape();
}

double calculateAngle(const Vector2 &v1, const Vector2 &v2) {
    double theta1, r1;
    Vector2::ToPolar(v1, theta1, r1);
    double theta2, r2;
    Vector2::ToPolar(v2, theta2, r2);

    double theta = theta2 - theta1;
    while(theta > M_PI) theta -= 2 * M_PI;
    while(theta < -M_PI) theta += 2 * M_PI;
    return theta;
}

vector<const Connection *> Lane::getOutgoing() const {
    Vector2 inLaneDir = getOutgoingDirection();

    vector<pair<double, const Connection *>> outConnections;

    if(net.connections.count(parent().id)) {
        const auto &connectionsFromEdge = net.connections.at(parent().id);
        if(connectionsFromEdge.count(index)) {
            const auto &connectionsFrom = connectionsFromEdge.at(index);
            for(const auto &[toID, conns1]: connectionsFrom) {
                for(const auto &[toLaneIndex, conns2]: conns1) {
                    for(const Connection &conn: conns2) {
                        Vector2 outLaneDir = conn.toLane().getIncomingDirection();
                        double  angle      = calculateAngle(inLaneDir, outLaneDir);
                        outConnections.emplace_back(angle, &conn);
                    }
                }
            }
        }
    }

    sort(outConnections.begin(), outConnections.end());

    vector<const Connection *> ret;
    for(const auto &[_, conn]: outConnections)
        ret.push_back(conn);
    return ret;
}

const Lane &Connection::fromLane() const {
    return from.lanes.at(fromLaneIndex);
}

const Lane &Connection::toLane() const {
    return to.lanes.at(toLaneIndex);
}

const Junction &Connection::getJunction() const {
    return *from.to;
}

/*
 * This implementation is based on Node#getLinkIndex(conn):
 * https://github.com/eclipse/sumo/blob/main/tools/sumolib/net/node.py
 */
size_t Connection::getJunctionIndex() const {
    size_t ret = 0;

    const Junction &junction = *from.to;
    for(const Edge::Lane *lane: junction.incLanes) {
        const vector<const Connection *> &connections = lane->getOutgoing();
        if(lane->id != fromLane().id) {
            ret += connections.size();
        } else {
            for(const Connection *connection: connections) {
                if(connection == this) {
                    return ret;
                }
                ++ret;
            }
        }
    }

    throw logic_error(
        "This connection comes from lane " + fromLane().id + " but that lane is not in the incLanes of junction " + junction.id
    );
}

Time Connection::getGreenTime() const {
    return tl->getGreenTime(linkIndex);
}

Time Connection::getCycleTime() const {
    return tl->getCycleTime();
}

size_t Connection::getNumberStops() const {
    return tl->getNumberStops(linkIndex);
}

const Request &Connection::getRequest() const {
    return getJunction().requests.at(getJunctionIndex());
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

Shape Edge::getShape() const {
    if(!shape.empty()) {
        Shape ret = shape;

        SUMO::Coord first(0, 0), last(0, 0);
        for(const auto &[_, lane]: lanes) {
            first += lane.shape.front() / (double)lanes.size();
            last += lane.shape.back() / (double)lanes.size();
        }

        ret.push_front(first);
        ret.push_back(last);

        return ret;
    }

    if(lanes.size() == 1) {
        return lanes.at(0).shape;
    }

    SUMO::Coord p1 = from->pos;
    SUMO::Coord p2 = to->pos;

    return {p1, p2};
}

vector<const Edge *> Edge::getOutgoing() const {
    vector<const Edge *> ret;

    if(net.edgesByJunctions.count(to->id)) {
        for(const auto &[nextJunctionID, edges]: net.edgesByJunctions.at(to->id)) {
            for(const Edge *edge: edges) {
                if(edge->from->id == to->id) {
                    ret.push_back(edge);
                }
            }
        }
    }

    return ret;
}

vector<const Connection *> Edge::getOutgoingConnections() const {
    vector<const Connection *> ret;
    for(const auto &[laneIndex, lane]: lanes) {
        vector<const Connection *> conns = lane.getOutgoing();
        ret.insert(ret.end(), conns.begin(), conns.end());
    }
    return ret;
}

const Junction &Request::junction() const {
    return net.junctions.at(junctionID);
}

vector<const Connection *> Request::getResponse() const {
    vector<const Connection *> ret;

    vector<const Connection *> allConnections = junction().getConnections();

    assert(allConnections.size() == response.size());

    for(size_t i = 0; i < allConnections.size(); ++i) {
        if(response.at(i))
            ret.push_back(allConnections.at(i));
    }

    return ret;
}

vector<const Connection *> Junction::getConnections() const {
    vector<const Connection *> ret;
    for(const Lane *lanePtr: incLanes) {
        const Lane &lane = *lanePtr;

        vector<const Connection *> conns = lane.getOutgoing();
        ret.insert(ret.end(), conns.begin(), conns.end());
    }
    return ret;
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
    Edge edge{
        *this,
        it->first_attribute("id")->value()};

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
        // clang-format off
        Lane lane {
            *this,
            edge.id,
            it2->first_attribute("id")->value(),
            stringify<Lane::Index>::fromString(it2->first_attribute("index")->value()),
            stringify<Speed>::fromString(it2->first_attribute("speed")->value()),
            stringify<Length>::fromString(it2->first_attribute("length")->value()),
            stringify<Shape>::fromString(it2->first_attribute("shape")->value())
        };
        // clang-format on

        const auto &[_, success] = edge.lanes.emplace(lane.index, lane);

        if(!success) {
            throw logic_error("Lane " + lane.id + ", repeated index " + to_string(lane.index));
        }
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

    for(auto it2 = it->first_node("request"); it2; it2 = it2->next_sibling("request")) {
        // clang-format off
        Request request{
            *this,
            junction.id,
            stringify<Index>::fromString(it2->first_attribute("index")->value()),
            stringify<vector<bool>>::fromString(it2->first_attribute("response")->value()),
            stringify<vector<bool>>::fromString(it2->first_attribute("foes")->value()),
            stringify<bool>::fromString(it2->first_attribute("cont")->value())
        };
        // clang-format on

        reverse(request.response.begin(), request.response.end());
        reverse(request.foes.begin(), request.foes.end());

        junction.requests.emplace(request.index, request);
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
        else {
            tPrev = tlLogic.phases.rbegin()->first + tlLogic.phases.rbegin()->second.duration;
        }

        tlLogic.phases[tPrev] = phase;
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

    connection.fromLane();
    connection.toLane();

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
    shared_ptr<file<>> xmlFilePointer = nullptr;
    try {
        xmlFilePointer = make_shared<file<>>(path.c_str());
    } catch(const ios_base::failure &e) {
        throw ios_base::failure("Could not open file " + path);
    }
    file<> &xmlFile = *xmlFilePointer;

    xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // Get data from XML parser
    const auto &net = *doc.first_node();

    // Edges
    for(auto it = net.first_node("edge"); it; it = it->next_sibling("edge")) {
        Edge edge                = network.loadEdge(it);
        const auto &[_, success] = network.edges.emplace(edge.id, edge);
        for(const auto &p: edge.lanes) {
            const Lane &lane       = p.second;
            network.lanes[lane.id] = make_pair(edge.id, lane.index);
        }
        if(edge.fromID.empty() || edge.toID.empty()) continue;
        network.edgesByJunctions[edge.fromID][edge.toID].push_back(&network.edges.at(edge.id));
    }

    // Junctions
    for(auto it = net.first_node("junction"); it; it = it->next_sibling("junction")) {
        Junction junction = network.loadJunction(it);
        assert(
            junction.type == Junction::Type::INTERNAL || junction.requests.size() == junction.intLanes.size()
        );

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
        network.connections[c.from.id][c.fromLaneIndex][c.to.id][c.toLaneIndex].push_back(c);
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

vector<const Connection *> Network::getConnections(const Edge &e1, const Edge &e2) const {
    vector<const Connection *> ret;

    if(!connections.count(e1.id)) return ret;
    for(const auto &[laneIndex1, conns1]: connections.at(e1.id)) {
        if(!conns1.count(e2.id)) continue;
        for(const auto &[laneIndex2, conns2]: conns1.at(e2.id)) {
            for(const Connection &conn: conns2)
                ret.push_back(&conn);
        }
    }

    return ret;
}

unordered_map<SUMO::Network::Edge::ID, unordered_map<SUMO::Network::Edge::ID, list<const SUMO::Network::Connection *>>> Network::getConnections() const {
    unordered_map<SUMO::Network::Edge::ID, unordered_map<SUMO::Network::Edge::ID, list<const SUMO::Network::Connection *>>> ret;

    for(const auto &[fromID, conns1]: connections)
        for(const auto &[fromLaneIndex, conns2]: conns1)
            for(const auto &[toID, conns3]: conns2)
                for(const auto &[toLaneIndex, conns4]: conns3)
                    for(const Connection &conn: conns4)
                        ret[fromID][toID].push_back(&conn);

    return ret;
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
    try {
        os.open(path);
    } catch(const ios_base::failure &e) {
        throw ios_base::failure("Could not open file " + path);
    }
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
