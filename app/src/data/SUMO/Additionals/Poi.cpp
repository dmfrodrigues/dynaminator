#include "data/SUMO/Additionals/Poi.hpp"

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include "utils/stringify.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO::Additionals;
using namespace utils::stringify;

Poi::Poi(const string &id_):
    id(id_) {}

void Poi::loadFromXMLNode(const rapidxml::xml_node<> &node) {
    xml_attribute<> *typeEl = node.first_attribute("type");
    if(typeEl) type = typeEl->value();

    xml_attribute<> *colorEl = node.first_attribute("color");
    if(colorEl) color = stringify<color::rgb<float>>::fromString(colorEl->value());

    xml_attribute<> *layerEl = node.first_attribute("layer");
    if(layerEl) layer = stringify<float>::fromString(layerEl->value());

    xml_attribute<> *xEl = node.first_attribute("x");
    xml_attribute<> *yEl = node.first_attribute("y");
    if(xEl && yEl) {
        posXY = Coord(
            stringify<float>::fromString(xEl->value()),
            stringify<float>::fromString(yEl->value())
        );
    }

    xml_attribute<> *lonEl = node.first_attribute("lon");
    xml_attribute<> *latEl = node.first_attribute("lat");
    if(lonEl && latEl) {
        posLonLat = Coord(
            stringify<float>::fromString(lonEl->value()),
            stringify<float>::fromString(latEl->value())
        );
    }

    xml_attribute<> *laneEl = node.first_attribute("lane");
    if(laneEl) lane = laneEl->value();

    xml_attribute<> *posEl = node.first_attribute("pos");
    if(posEl) pos = stringify<float>::fromString(posEl->value());

    xml_attribute<> *angleEl = node.first_attribute("angle");
    if(angleEl) angle = stringify<float>::fromString(angleEl->value());

    xml_attribute<> *imgFileEl = node.first_attribute("imgFile");
    if(imgFileEl) imgFile = imgFileEl->value();

    xml_attribute<> *widthEl = node.first_attribute("width");
    if(widthEl) width = stringify<float>::fromString(widthEl->value());

    xml_attribute<> *heightEl = node.first_attribute("height");
    if(heightEl) height = stringify<float>::fromString(heightEl->value());
}
