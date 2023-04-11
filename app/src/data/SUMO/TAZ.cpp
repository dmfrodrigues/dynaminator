#include "data/SUMO/TAZ.hpp"

#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>

#include "utils/io.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "rapidxml.hpp"
#pragma GCC diagnostic pop

#include <iostream>

using namespace std;
using namespace rapidxml;
using namespace SUMO;
using namespace utils::stringify;

TAZs TAZ::loadFromFile(const string &path) {
    TAZs ret;

    // Parse XML
    string textStr = utils::readWholeFile(path);
    unique_ptr<char[]> text(new char[textStr.size() + 1]);
    strcpy(text.get(), textStr.c_str());
    xml_document<> doc;
    doc.parse<0>(text.get());

    // Get data from XML parser
    auto tazs = doc.first_node();
    if(!tazs->first_node("taz")) tazs = tazs->first_node();

    // Junctions
    for(auto it = tazs->first_node("taz"); it; it = it->next_sibling("taz")) {
        TAZ taz;

        taz.id = it->first_attribute("id")->value();

        for(auto it2 = it->first_node("tazSource"); it2; it2 = it2->next_sibling("tazSource")) {
            taz.sources.push_back({
                it2->first_attribute("id")->value(),
                stringify<TAZ::Weight>::fromString(
                    it2->first_attribute("weight")->value()
                )
            });
        }
        for(auto it2 = it->first_node("tazSink"); it2; it2 = it2->next_sibling("tazSink")) {
            taz.sinks.push_back({
                it2->first_attribute("id")->value(),
                stringify<TAZ::Weight>::fromString(
                    it2->first_attribute("weight")->value()
                )
            });
        }

        ret[taz.id] = taz;
    }

    return ret;
}
