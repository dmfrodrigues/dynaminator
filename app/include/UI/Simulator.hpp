#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <filesystem>

#include "data/SUMO/Network.hpp"

namespace UI {
class Simulator {
    std::shared_ptr<SUMO::Network> network;

    std::shared_ptr<sf::RenderWindow> window;

    std::vector<sf::Vertex> roads;
    std::vector<sf::Vertex> junctions;

    sf::View     view;
    float        scale  = 1.0;
    sf::Vector2f center = sf::Vector2f(0, 0);
    SUMO::Coord  offset;

    void loadNetworkGUI();

    void onScroll(float delta);
    void onResize();
    void recalculateView();

    const float SCALE_DELTA = 1.25;

    const float     LANE_WIDTH = 3.15;
    const sf::Color EDGE_COLOR = sf::Color::Black;

    const float     ARROW_LENGTH           = 1.5;
    const float     ARROW_WIDTH            = 1.2;
    const float     ARROW_DIST_TO_JUNCTION = 1.0;
    const sf::Color ARROW_COLOR            = sf::Color::White;

    const sf::Color JUNCTION_COLOR = sf::Color(102, 0, 0);

    const float     CONNECTION_WIDTH = 0.3;
    const sf::Color CONNECTION_COLOR = sf::Color::White;

   public:
    Simulator(std::filesystem::path configFile);

    void run();
};
}  // namespace UI
