#include "data/SUMO/Additionals/Additionals.hpp"

#include <spdlog/spdlog.h>

#include <iostream>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include "data/SUMO/Additionals/Location.hpp"
#include "data/SUMO/Additionals/Poi.hpp"
#include "data/SUMO/Additionals/Poly.hpp"
#include "utils/stringify.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO::Additionals;
using namespace utils::stringify;

vector<unique_ptr<Additional>> SUMO::Additionals::loadFromFile(const filesystem::path &path) {
    vector<unique_ptr<Additional>> result;

    file<> xmlFile(path.c_str());

    xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    xml_node<> *additionalEl = doc.first_node("additional");
    assert(additionalEl != nullptr);

    Additionals::XMLFactory xmlFactory;

    for(auto el = additionalEl->first_node(); el; el = el->next_sibling()) {
        result.emplace_back(xmlFactory.create(*el));
    }

    return result;
}

unique_ptr<Additional> XMLFactory::create(const rapidxml::xml_node<> &node) {
    if(string(node.name()) == "location") {
        unique_ptr<Location> result = make_unique<Location>();

        result->loadFromXMLNode(node);

        return result;
    }
    if(string(node.name()) == "poly") {
        xml_attribute<> *idAttr    = node.first_attribute("id");
        xml_attribute<> *shapeAttr = node.first_attribute("shape");

        assert(idAttr != nullptr);
        assert(shapeAttr != nullptr);

        unique_ptr<Poly> result = make_unique<Poly>(
            idAttr->value(),
            stringify<SUMO::Shape>::fromString(shapeAttr->value())
        );

        result->loadFromXMLNode(node);

        return result;
    }
    if(string(node.name()) == "poi") {
        xml_attribute<> *idAttr = node.first_attribute("id");

        assert(idAttr != nullptr);

        unique_ptr<Poi> result = make_unique<Poi>(idAttr->value());

        result->loadFromXMLNode(node);

        return result;
    }

    throw runtime_error("Unknown additional type: " + string(node.name()));
}
