#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <chrono>
#include <color/color.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <unordered_map>

#include "Dynamic/Env/Env.hpp"
#include "data/SUMO/Additionals/Additionals.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"

namespace UI {
class Simulator {
    using clk = std::chrono::steady_clock;

    std::shared_ptr<SUMO::Network> network;

    std::shared_ptr<sf::RenderWindow> window;

    std::optional<SUMO::NetState> netState;

    std::optional<SUMO::Time> begin;
    std::optional<SUMO::Time> end;

    SUMO::NetState::Timestep timestep;

    bool screenCaptures = false;

    std::vector<std::unique_ptr<SUMO::Additionals::Additional>> additionals;

    color::rgb<float> backgroundColor = color::rgb<float>({1.0, 1.0, 1.0});

    std::multimap<SUMO::Length, std::vector<sf::Vertex>> networkMap;
    std::multimap<SUMO::Length, std::vector<sf::Vertex>> vehiclesMap;
    std::multimap<SUMO::Length, std::vector<sf::Vertex>> trafficLightsMap;

    std::vector<sf::Vertex> backgroundVertices;
    std::vector<sf::Vertex> networkVertices;
    std::vector<sf::Vertex> vehicleVertices;
    std::vector<sf::Vertex> trafficLightVertices;

    bool running = false;

    sf::View     view;
    float        scale  = 1.0;
    sf::Vector2f center = sf::Vector2f(0, 0);
    SUMO::Coord  offset;

    std::chrono::milliseconds       delay;
    const std::chrono::milliseconds delayDelta;

    mutable std::mt19937 gen = std::mt19937(0);

    std::unordered_map<SUMO::NetState::Timestep::Edge::Lane::Vehicle::ID, sf::Color> vehicleColor;

    sf::Color generateNewVehicleColor() const;

    void loadBackground();
    void loadNetworkGUI();
    void loadVehicles();
    void loadTrafficLights();

    void onScroll(float delta);
    void onResize();
    void recalculateView();

    void loadEdges();
    void loadJunctions();

    void draw();

    bool zOrder = true;

    color::hsv<float> edgeColorHeight(SUMO::Length height) const;

    const float SCALE_DELTA = 1.25;

    const float             LANE_WIDTH = 3.15f;
    const color::rgb<float> EDGE_COLOR = color::rgb<float>({0.0, 0.0, 0.0});

    const float     ARROW_LENGTH           = 1.5;
    const float     ARROW_WIDTH            = 1.2f;
    const float     ARROW_DIST_TO_JUNCTION = 1.0;
    const sf::Color ARROW_COLOR            = sf::Color::White;

    const sf::Color JUNCTION_COLOR = sf::Color(102, 0, 0);

    const float     CONNECTION_WIDTH = 0.3f;
    const sf::Color CONNECTION_COLOR = sf::Color::White;

    static const SUMO::Length VEHICLE_LENGTH;
    const float               VEHICLE_WIDTH = 2.0;
    const sf::Color           VEHICLE_COLOR = sf::Color::Yellow;

    const float TRAFFIC_LIGHT_LENGTH = 0.5;

    const unsigned int FPS_GOAL;

   public:
    Simulator(std::filesystem::path configFile);

    void run();
};
}  // namespace UI
