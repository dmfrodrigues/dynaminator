#include "data/SUMO/Additionals/Poly.hpp"

#include <spdlog/spdlog.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include "utils/stringify.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO::Additionals;
using namespace utils::stringify;

Poly::Poly(const string &id_, const SUMO::Shape &shape_):
    id(id_), shape(shape_) {}

void Poly::loadFromXMLNode(const rapidxml::xml_node<> &node) {
    xml_attribute<> *typeEl = node.first_attribute("type");
    if(typeEl) type = typeEl->value();

    xml_attribute<> *colorEl = node.first_attribute("color");
    if(colorEl)
        color = stringify<color::rgb<float>>::fromString(colorEl->value());
    else
        spdlog::warn("No color specified for poly '{}'", id);

    xml_attribute<> *fillEl = node.first_attribute("fill");
    if(fillEl) fill = stringify<bool>::fromString(fillEl->value());

    xml_attribute<> *geoEl = node.first_attribute("geo");
    if(geoEl) geo = stringify<bool>::fromString(geoEl->value());

    xml_attribute<> *layerEl = node.first_attribute("layer");
    if(layerEl) layer = stringify<float>::fromString(layerEl->value());

    xml_attribute<> *angleEl = node.first_attribute("angle");
    if(angleEl) angle = stringify<float>::fromString(angleEl->value());

    xml_attribute<> *imgFileEl = node.first_attribute("imgFile");
    if(imgFileEl) imgFile = imgFileEl->value();

    xml_attribute<> *lineWidthEl = node.first_attribute("lineWidth");
    if(lineWidthEl) lineWidth = stringify<float>::fromString(lineWidthEl->value());
}
