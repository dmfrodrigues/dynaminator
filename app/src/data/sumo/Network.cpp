#include "data/sumo/Network.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>

#include "utils/io.hpp"
#include "utils/stringifier.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#pragma GCC diagnostic pop

using namespace std;
using namespace rapidxml;

typedef SUMO::Shape Shape;
typedef SUMO::Network::Junction Junction;
typedef SUMO::Network::Edge Edge;
typedef SUMO::Network::Edge::Lane Lane;

using utils::stringifier;

const Junction::ID Junction::INVALID = "";

Edge::Function utils::stringifier<SUMO::Network::Edge::Function>::fromString(const string &s) {
    if (s == "internal")
        return Edge::Function::INTERNAL;
    else if (s == "connector")
        return Edge::Function::CONNECTOR;
    else if (s == "crossing")
        return Edge::Function::CROSSING;
    else if (s == "walkingarea")
        return Edge::Function::WALKINGAREA;
    else
        return Edge::Function::NORMAL;
}

string utils::stringifier<SUMO::Network::Edge::Function>::toString(const Edge::Function &t) {
    switch (t) {
        case Edge::Function::INTERNAL:
            return "internal";
        case Edge::Function::CONNECTOR:
            return "connector";
        case Edge::Function::CROSSING:
            return "crossing";
        case Edge::Function::WALKINGAREA:
            return "walkingarea";
        case Edge::Function::NORMAL:
            return "normal";
        default:
            throw logic_error("Value of SUMO::Network::Edge::Function is invalid");
    }
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
        Junction junction;

        junction.id = it->first_attribute("id")->value();
        junction.pos = Coord(
            stringifier<double>::fromString(it->first_attribute("x")->value()),
            stringifier<double>::fromString(it->first_attribute("y")->value()));

        network.junctions[junction.id] = junction;
    }

    // Edges
    for (auto it = net.first_node("edge"); it; it = it->next_sibling("edge")) {
        Edge edge;

        edge.id = it->first_attribute("id")->value();
        try {
            {
                auto *fromAttr = it->first_attribute("from");
                if (fromAttr) edge.from = network.junctions.at(fromAttr->value()).id;
            }
            {
                auto *toAttr = it->first_attribute("to");
                if (toAttr) edge.to = network.junctions.at(toAttr->value()).id;
            }
        } catch (const out_of_range &e) {
            cerr << e.what() << endl;
            continue;
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

        network.edges[edge.id] = edge;
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

        string &ps  = (strs.emplace_back() = stringifier<Edge::Priority>::toString(e.priority));
        string &fs  = (strs.emplace_back() = stringifier<Edge::Function>::toString(e.function));
        string &lns = (strs.emplace_back() = stringifier<size_t        >::toString(e.lanes.size()));

        Lane::Length length = 0;
        Lane::Speed speed = 0;
        for (const auto &l : e.lanes) {
            length += l.second.length;
            speed += l.second.speed;
        }
        length /= (Lane::Length)e.lanes.size();
        speed /= (Lane::Speed)e.lanes.size();
        Lane::Speed speed_kmh = speed * 3.6;

        string &ls   = (strs.emplace_back() = stringifier<Lane::Length>::toString(length   ));
        string &ss   = (strs.emplace_back() = stringifier<Lane::Speed >::toString(speed    ));
        string &kmhs = (strs.emplace_back() = stringifier<Lane::Speed >::toString(speed_kmh));

        auto edge = doc.allocate_node(node_element, "edge");
        edge->append_attribute(doc.allocate_attribute("id"       , eid  .c_str()));
        edge->append_attribute(doc.allocate_attribute("priority" , ps   .c_str()));
        edge->append_attribute(doc.allocate_attribute("function" , fs   .c_str()));
        edge->append_attribute(doc.allocate_attribute("lanes"    , lns  .c_str()));
        edge->append_attribute(doc.allocate_attribute("length"   , ls   .c_str()));
        edge->append_attribute(doc.allocate_attribute("speed"    , ss   .c_str()));
        edge->append_attribute(doc.allocate_attribute("speed_kmh", kmhs .c_str()));
        interval->append_node(edge);
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    os.open(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
