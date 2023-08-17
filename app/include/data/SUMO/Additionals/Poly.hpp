#pragma once

#include "data/SUMO/Additionals/Additionals.hpp"

namespace SUMO::Additionals {

class Poly: public Additional {
   public:
    const std::string                id;
    std::optional<std::string>       type;
    std::optional<color::rgb<float>> color;
    bool                             fill = false;
    std::optional<bool>              geo;
    std::optional<float>             layer;
    const SUMO::Shape                shape;
    std::optional<float>             angle;
    std::optional<std::string>       imgFile;
    std::optional<float>             lineWidth;

    Poly(const std::string& id, const SUMO::Shape& shape);

    virtual void loadFromXMLNode(const rapidxml::xml_node<>& node) override;
};

}  // namespace SUMO::Additionals
