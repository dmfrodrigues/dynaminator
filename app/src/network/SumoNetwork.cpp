#include "network/SumoNetwork.hpp"

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

typedef SumoNetwork::Junction Junction;
typedef SumoNetwork::Junction::Id JunctionId;
typedef SumoNetwork::Edge Edge;
typedef SumoNetwork::Edge::Id EdgeId;

SumoNetwork::Edge::Function SumoNetwork::Edge::charArrayToFunction(char *arr){
    string s(arr);
    if(     s == "internal") return INTERNAL;
    else if(s == "connector") return CONNECTOR;
    else if(s == "crossing") return CROSSING;
    else if(s == "walkingarea") return WALKINGAREA;
    else return NORMAL;
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
    for (auto it = net.first_node("junction"); string(it->name()) == "junction"; it = it->next_sibling()) {
        Junction junction;

        junction.id = it->first_attribute("id")->value();
        junction.pos = Coord(
            atof(it->first_attribute("x")->value()),
            atof(it->first_attribute("y")->value())
        );

        network.junctions[junction.id] = junction;
    }

    // Edges
    for (auto it = net.first_node("edge"); string(it->name()) == "edge"; it = it->next_sibling()) {
        Edge edge;

        edge.id = it->first_attribute("id")->value();
        { auto *fromAttr = it->first_attribute("from"); if(fromAttr) edge.from = fromAttr->value(); }
        { auto *toAttr   = it->first_attribute("to"  ); if(toAttr  ) edge.to   = toAttr  ->value(); }
        { auto *priorityAttr = it->first_attribute("priority"); if(priorityAttr) edge.priority = atoi(priorityAttr->value()); }
        { auto *functionAttr = it->first_attribute("function"); if(functionAttr) edge.function = Edge::charArrayToFunction(functionAttr->value()); }

        if(!edge.from.empty() && network.junctions.find(edge.from) == network.junctions.end()){ cerr << "Edge " << edge.id << " 'from' is invalid" << endl; }
        if(!edge.to  .empty() && network.junctions.find(edge.to  ) == network.junctions.end()){ cerr << "Edge " << edge.id << " 'to' is invalid"   << endl; }

        network.edges[edge.id] = edge;
    }

    return network;
}
