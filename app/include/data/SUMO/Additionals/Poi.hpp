#pragma once

#include "data/SUMO/Additionals/Additionals.hpp"

namespace SUMO::Additionals {

class Poi: public Additional {
   public:
    const std::string                id;
    std::optional<std::string>       type;
    std::optional<color::rgb<float>> color;
    std::optional<float>             layer;
    std::optional<Coord>             posXY;
    std::optional<Coord>             posLonLat;
    std::optional<std::string>       lane;
    std::optional<float>             pos;
    std::optional<float>             angle;
    std::optional<std::string>       imgFile;
    std::optional<float>             width;
    std::optional<float>             height;

    Poi(const std::string& id);

    virtual void loadFromXMLNode(const rapidxml::xml_node<>& node) override;
};

}  // namespace SUMO::Additionals
