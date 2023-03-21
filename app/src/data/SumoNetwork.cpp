#include "data/SumoNetwork.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include "utils/io.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#pragma GCC diagnostic pop

using namespace std;
using namespace rapidxml;

typedef SumoNetwork::Shape Shape;
typedef SumoNetwork::Junction Junction;
typedef SumoNetwork::Edge Edge;
typedef SumoNetwork::Edge::Lane Lane;

const Junction::Id Junction::INVALID = "";

SumoNetwork::Edge::Function SumoNetwork::Edge::stringToFunction(const string &s){
    if(     s == "internal") return INTERNAL;
    else if(s == "connector") return CONNECTOR;
    else if(s == "crossing") return CROSSING;
    else if(s == "walkingarea") return WALKINGAREA;
    else return NORMAL;
}

Shape SumoNetwork::stringToShape(const string &s) {
    Shape shape;

    stringstream ss(s);
    string coordStr;
    while (ss >> coordStr) {
        size_t idx = coordStr.find(',');
        Coord c(
            atof(coordStr.substr(0, idx).c_str()),
            atof(coordStr.substr(idx + 1).c_str()));
    }

    return shape;
}

SumoNetwork SumoNetwork::loadFromFile(const string &path) {
    SumoNetwork network;

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
            atof(it->first_attribute("x")->value()),
            atof(it->first_attribute("y")->value()));

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
            if (priorityAttr) edge.priority = atoi(priorityAttr->value());
        }
        {
            auto *functionAttr = it->first_attribute("function");
            if (functionAttr) edge.function = Edge::stringToFunction(functionAttr->value());
        }
        {
            auto *shapeAttr = it->first_attribute("shape");
            if (shapeAttr) edge.shape = SumoNetwork::stringToShape(shapeAttr->value());
        }

        for (auto it2 = it->first_node("lane"); it2; it2 = it2->next_sibling("lane")) {
            Lane lane;

            lane.id = it2->first_attribute("id")->value();
            lane.index = atoi(it2->first_attribute("index")->value());
            lane.speed = atof(it2->first_attribute("speed")->value());
            lane.length = atof(it2->first_attribute("length")->value());
            lane.shape = SumoNetwork::stringToShape(it2->first_attribute("shape")->value());

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

vector<Junction> SumoNetwork::getJunctions() const {
    vector<Junction> ret;
    ret.reserve(junctions.size());
    for (const auto &p : junctions)
        ret.push_back(p.second);
    return ret;
}

vector<Edge> SumoNetwork::getEdges() const {
    vector<Edge> ret;
    ret.reserve(edges.size());
    for (const auto &p : edges)
        ret.push_back(p.second);
    return ret;
}

void SumoNetwork::saveStatsToFile(const string &path) const {
    xml_document<> doc;
    auto meandata = doc.allocate_node(node_element, "meandata");
    doc.append_node(meandata);
    auto interval = doc.allocate_node(node_element, "interval");
    interval->append_attribute(doc.allocate_attribute("begin", "0.0"));
    interval->append_attribute(doc.allocate_attribute("end", "1.0"));
    meandata->append_node(interval);

    for (const auto &p : edges) {
        const SumoNetwork::Edge::Id &eid = p.first;
        const Edge &e = p.second;

        char *ps = new char[256];
        sprintf(ps, "%d", e.priority);
        char *fs = new char[256];
        sprintf(fs, "%d", e.function);
        char *lns = new char[256];
        sprintf(lns, "%lu", e.lanes.size());

        double length = 0;
        double speed = 0;
        for(const auto &l: e.lanes){
            length += l.second.length;
            speed += l.second.speed;
        }
        length /= (double)e.lanes.size();
        speed  /= (double)e.lanes.size();
        double speed_kmh = speed * 3.6;

        char *ls = new char[256];
        sprintf(ls, "%lf", length);
        char *ss = new char[256];
        sprintf(ss, "%lf", speed);
        char *kmhs = new char[256];
        sprintf(kmhs, "%lf", speed_kmh);

        auto edge = doc.allocate_node(node_element, "edge");
        edge->append_attribute(doc.allocate_attribute("id", eid.c_str()));
        edge->append_attribute(doc.allocate_attribute("priority", ps));
        edge->append_attribute(doc.allocate_attribute("function", fs));
        edge->append_attribute(doc.allocate_attribute("lanes", lns));
        edge->append_attribute(doc.allocate_attribute("length", ls));
        edge->append_attribute(doc.allocate_attribute("speed", ss));
        edge->append_attribute(doc.allocate_attribute("speed_kmh", kmhs));
        interval->append_node(edge);
    }

    ofstream os(path);
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
