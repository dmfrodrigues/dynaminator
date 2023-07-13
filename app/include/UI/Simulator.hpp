#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <chrono>
#include <filesystem>
#include <optional>
#include <random>
#include <unordered_map>

#include "Dynamic/Env/Env.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/Network.hpp"

namespace UI {
class Simulator {
    using clk = std::chrono::steady_clock;

    std::shared_ptr<SUMO::Network> network;

    std::shared_ptr<sf::RenderWindow> window;

    std::optional<SUMO::NetState> netState;

    SUMO::NetState::Timestep timestep;

    std::vector<sf::Vertex> roads;
    std::vector<sf::Vertex> junctions;
    std::vector<sf::Vertex> vehicles;
    std::vector<sf::Vertex> trafficLights;

    bool running = false;

    sf::View     view;
    float        scale  = 1.0;
    sf::Vector2f center = sf::Vector2f(0, 0);
    SUMO::Coord  offset;

    std::chrono::milliseconds delayDelta;
    std::chrono::milliseconds delay;

    mutable std::mt19937 gen = std::mt19937(0);

    std::unordered_map<SUMO::NetState::Timestep::Edge::Lane::Vehicle::ID, sf::Color> vehicleColor;

    sf::Color generateNewVehicleColor() const;

    void loadNetworkGUI();
    void loadVehicles();
    void loadTrafficLights();

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

    static const float VEHICLE_LENGTH;
    const float        VEHICLE_WIDTH = 2.0;
    const sf::Color    VEHICLE_COLOR = sf::Color::Yellow;

    const float TRAFFIC_LIGHT_LENGTH = 0.5;

    static constexpr unsigned int FPS_GOAL = 60;

   public:
    Simulator(std::filesystem::path configFile);

    void run();
};
}  // namespace UI
