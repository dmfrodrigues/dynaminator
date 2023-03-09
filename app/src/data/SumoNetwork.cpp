#include "data/SumoNetwork.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "rapidxml.hpp"
#pragma GCC diagnostic pop

using namespace std;
using namespace rapidxml;

typedef SumoNetwork::Shape Shape;
typedef SumoNetwork::Junction Junction;
typedef SumoNetwork::Edge Edge;
typedef SumoNetwork::Edge::Lane Lane;

SumoNetwork::Edge::Function SumoNetwork::Edge::stringToFunction(const string &s){
    if(     s == "internal") return INTERNAL;
    else if(s == "connector") return CONNECTOR;
    else if(s == "crossing") return CROSSING;
    else if(s == "walkingarea") return WALKINGAREA;
    else return NORMAL;
}

Shape SumoNetwork::stringToShape(const string &s){
    Shape shape;

    stringstream ss(s);
    string coordStr;
    while(ss >> coordStr){
        size_t idx = coordStr.find(',');
        Coord c(
            atof(coordStr.substr(0, idx).c_str()),
            atof(coordStr.substr(idx+1).c_str())
        );
    }

    return shape;
}


string readWholeFile(const string &path) {
    ifstream ifs(path);
    stringstream ss;
    ss << ifs.rdbuf();
    string all = ss.str();
    return all;
}

SumoNetwork SumoNetwork::loadFromFile(const string &path) {
    SumoNetwork network;

    // Parse XML
    string textStr = readWholeFile(path);
    unique_ptr<char[]> text(new char[textStr.size() + 1]);
    strcpy(text.get(), textStr.c_str());
    xml_document<> doc;
    doc.parse<0>(text.get());

    // Get data from XML parser
    const auto &net = *doc.first_node();

    // Junctions
    for (auto it = net.first_node("junction"); it && string(it->name()) == "junction"; it = it->next_sibling()) {
        Junction junction;

        junction.id = it->first_attribute("id")->value();
        junction.pos = Coord(
            atof(it->first_attribute("x")->value()),
            atof(it->first_attribute("y")->value())
        );

        network.junctions[junction.id] = junction;
    }

    // Edges
    for (auto it = net.first_node("edge"); it && string(it->name()) == "edge"; it = it->next_sibling()) {
        Edge edge;

        edge.id = it->first_attribute("id")->value();
        { auto *fromAttr = it->first_attribute("from"); if(fromAttr) edge.from = fromAttr->value(); }
        { auto *toAttr   = it->first_attribute("to"  ); if(toAttr  ) edge.to   = toAttr  ->value(); }
        { auto *priorityAttr = it->first_attribute("priority"); if(priorityAttr) edge.priority = atoi(priorityAttr->value()); }
        { auto *functionAttr = it->first_attribute("function"); if(functionAttr) edge.function = Edge::stringToFunction(functionAttr->value()); }
        { auto *shapeAttr = it->first_attribute("shape"); if(shapeAttr) edge.shape = SumoNetwork::stringToShape(shapeAttr->value()); }

        if(!edge.from.empty() && network.junctions.find(edge.from) == network.junctions.end()){ cerr << "Edge " << edge.id << " 'from' is invalid" << endl; }
        if(!edge.to  .empty() && network.junctions.find(edge.to  ) == network.junctions.end()){ cerr << "Edge " << edge.id << " 'to' is invalid"   << endl; }

        for(auto it2 = it->first_node("lane"); it2 && string(it2->name()) == "lane"; it2 = it2->next_sibling()){
            Lane lane;

            lane.id    = it2->first_attribute("id")->value();
            lane.index = atoi(it2->first_attribute("index")->value());
            lane.speed = atof(it2->first_attribute("speed")->value());
            lane.length = atof(it2->first_attribute("length")->value());
            lane.shape = SumoNetwork::stringToShape(it2->first_attribute("shape")->value());

            if(edge.lanes.count(lane.index)){
                cerr << "Lane " << lane.id << ", repeated index " << lane.index << endl;
                continue;
            }
            edge.lanes[lane.index] = lane;
        }

        network.edges[edge.id] = edge;
    }

    return network;
}
