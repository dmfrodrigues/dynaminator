#include "data/SumoTAZs.hpp"

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

typedef SumoTAZs::TAZ TAZ;

void SumoTAZs::addTAZ(TAZ taz) {
    tazs[taz.id] = taz;
}

vector<TAZ> SumoTAZs::getTAZs() const {
    vector<TAZ> ret;
    ret.reserve(tazs.size());
    for (const auto &p : tazs)
        ret.push_back(p.second);
    return ret;
}

SumoTAZs SumoTAZs::loadFromFile(const string &path) {
    SumoTAZs ret;

    // Parse XML
    string textStr = utils::readWholeFile(path);
    unique_ptr<char[]> text(new char[textStr.size() + 1]);
    strcpy(text.get(), textStr.c_str());
    xml_document<> doc;
    doc.parse<0>(text.get());

    // Get data from XML parser
    const auto &tazs = *doc.first_node()->first_node();

    // Junctions
    for (auto it = tazs.first_node("taz"); it; it = it->next_sibling("taz")) {
        TAZ taz;

        taz.id = it->first_attribute("id")->value();

        for (auto it2 = it->first_node("tazSource"); it2; it2 = it2->next_sibling("tazSource")) {
            taz.sources.push_back({it2->first_attribute("id")->value(),
                                   atof(it2->first_attribute("weight")->value())});
        }
        for (auto it2 = it->first_node("tazSink"); it2; it2 = it2->next_sibling("tazSink")) {
            taz.sinks.push_back({it2->first_attribute("id")->value(),
                                 atof(it2->first_attribute("weight")->value())});
        }

        ret.addTAZ(taz);
    }

    return ret;
}
