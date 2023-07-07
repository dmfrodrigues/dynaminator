#include "UI/Simulator.hpp"

#include <SFML/Graphics/PrimitiveType.hpp>
#include <SFML/System/Vector2.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <queue>

#include "data/SUMO/Network.hpp"
#include "rapidxml_utils.hpp"

using namespace std;
using namespace UI;

using namespace rapidxml;

using clk = chrono::steady_clock;

Simulator::Simulator(filesystem::path configFilePath) {
    file<>         configFile(configFilePath.c_str());
    xml_document<> doc;
    doc.parse<0>(configFile.data());

    const xml_node<> &configuration = *doc.first_node("configuration");

    const xml_node<> &input = *configuration.first_node("input");

    filesystem::path netFilePath = configFilePath.parent_path() / input.first_node("net-file")->first_attribute("value")->value();

    network = SUMO::Network::loadFromFile(netFilePath);

    loadNetworkGUI();
}

void Simulator::loadNetworkGUI() {
    offset = network->location.center();

    assert(network->getEdge("1016617006").to.has_value());

    for(const SUMO::Network::Edge &edge: network->getEdges()) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        for(const SUMO::Network::Edge::Lane &lane: edge.lanes) {
            assert(lane.shape.size() >= 2);
            for(
                auto it1 = lane.shape.begin(),
                     it2 = ++lane.shape.begin();
                it2 != lane.shape.end();
                ++it1, ++it2
            ) {
                SUMO::Coord u = *it1 - offset;
                SUMO::Coord v = *it2 - offset;

                Vector2 uv = v - u;
                Vector2 uvPerp(uv.Y, -uv.X);
                uvPerp /= Vector2::Magnitude(uvPerp);

                Vector2 u1Coord = u + uvPerp * LANE_WIDTH / 2;
                Vector2 u2Coord = u - uvPerp * LANE_WIDTH / 2;

                Vector2 v1Coord = v + uvPerp * LANE_WIDTH / 2;
                Vector2 v2Coord = v - uvPerp * LANE_WIDTH / 2;

                sf::Vector2f u1(u1Coord.X, -u1Coord.Y);
                sf::Vector2f u2(u2Coord.X, -u2Coord.Y);
                sf::Vector2f v1(v1Coord.X, -v1Coord.Y);
                sf::Vector2f v2(v2Coord.X, -v2Coord.Y);

                roads.emplace_back(u1, EDGE_COLOR);
                roads.emplace_back(u2, EDGE_COLOR);
                roads.emplace_back(v1, EDGE_COLOR);

                roads.emplace_back(u2, EDGE_COLOR);
                roads.emplace_back(v1, EDGE_COLOR);
                roads.emplace_back(v2, EDGE_COLOR);
            }

            SUMO::Coord u = *(++lane.shape.rbegin()) - offset;
            SUMO::Coord v = *lane.shape.rbegin() - offset;

            Vector2 uv = v - u;
            uv /= Vector2::Magnitude(uv);

            Vector2 uvPerp(uv.Y, -uv.X);

            Vector2 arrowFrontCoord = v - uv * ARROW_DIST_TO_JUNCTION;
            Vector2 arrowBackCoord  = arrowFrontCoord - uv * ARROW_LENGTH;
            Vector2 arrowBack1Coord = arrowBackCoord + uvPerp * (ARROW_WIDTH / 2);
            Vector2 arrowBack2Coord = arrowBackCoord - uvPerp * (ARROW_WIDTH / 2);

            sf::Vector2f arrowFront(arrowFrontCoord.X, -arrowFrontCoord.Y);
            sf::Vector2f arrowBack1(arrowBack1Coord.X, -arrowBack1Coord.Y);
            sf::Vector2f arrowBack2(arrowBack2Coord.X, -arrowBack2Coord.Y);

            roads.emplace_back(arrowFront, ARROW_COLOR);
            roads.emplace_back(arrowBack1, ARROW_COLOR);
            roads.emplace_back(arrowBack2, ARROW_COLOR);
        }
    }

    for(const SUMO::Network::Junction &junction: network->getJunctions()) {
        if(junction.type == SUMO::Network::Junction::Type::INTERNAL) continue;

        if(junction.shape.size() < 2) {
            throw runtime_error("Junction " + junction.id + " has less than 2 points in shape");
        }

        SUMO::Coord centerCoord = junction.pos - offset;
        // SUMO::Coord  centerCoord = junction.shape.front() - offset;
        sf::Vector2f center(centerCoord.X, -centerCoord.Y);

        for(
            auto it1 = junction.shape.begin(), it2 = ++junction.shape.begin();
            it2 != junction.shape.end();
            ++it1, ++it2
        ) {
            SUMO::Coord uCoord = *it1 - offset;
            SUMO::Coord vCoord = *it2 - offset;

            sf::Vector2f u(uCoord.X, -uCoord.Y);
            sf::Vector2f v(vCoord.X, -vCoord.Y);

            junctions.emplace_back(center, JUNCTION_COLOR);
            junctions.emplace_back(u, JUNCTION_COLOR);
            junctions.emplace_back(v, JUNCTION_COLOR);
        }

        SUMO::Coord uCoord = junction.shape.back() - offset;
        SUMO::Coord vCoord = junction.shape.front() - offset;

        sf::Vector2f u(uCoord.X, -uCoord.Y);
        sf::Vector2f v(vCoord.X, -vCoord.Y);

        junctions.emplace_back(center, JUNCTION_COLOR);
        junctions.emplace_back(u, JUNCTION_COLOR);
        junctions.emplace_back(v, JUNCTION_COLOR);
    }

    for(const SUMO::Network::Edge &edge: network->getEdges()) {
        if(edge.function != SUMO::Network::Edge::Function::INTERNAL) continue;

        for(const SUMO::Network::Edge::Lane &lane: edge.lanes) {
            for(
                auto it1 = lane.shape.begin(),
                     it2 = ++lane.shape.begin();
                it2 != lane.shape.end();
                ++it1, ++it2
            ) {
                SUMO::Coord u = *it1 - offset;
                SUMO::Coord v = *it2 - offset;

                Vector2 uv = v - u;
                Vector2 uvPerp(uv.Y, -uv.X);
                uvPerp /= Vector2::Magnitude(uvPerp);

                Vector2 u1Coord = u + uvPerp * CONNECTION_WIDTH / 2;
                Vector2 u2Coord = u - uvPerp * CONNECTION_WIDTH / 2;

                Vector2 v1Coord = v + uvPerp * CONNECTION_WIDTH / 2;
                Vector2 v2Coord = v - uvPerp * CONNECTION_WIDTH / 2;

                sf::Vector2f u1(u1Coord.X, -u1Coord.Y);
                sf::Vector2f u2(u2Coord.X, -u2Coord.Y);
                sf::Vector2f v1(v1Coord.X, -v1Coord.Y);
                sf::Vector2f v2(v2Coord.X, -v2Coord.Y);

                junctions.emplace_back(u1, CONNECTION_COLOR);
                junctions.emplace_back(u2, CONNECTION_COLOR);
                junctions.emplace_back(v1, CONNECTION_COLOR);

                junctions.emplace_back(u2, CONNECTION_COLOR);
                junctions.emplace_back(v1, CONNECTION_COLOR);
                junctions.emplace_back(v2, CONNECTION_COLOR);
            }
        }
    }
}

void Simulator::onScroll(float delta) {
    scale *= pow(SCALE_DELTA, -delta);
    recalculateView();
}

void Simulator::onResize() {
    recalculateView();
}

void Simulator::recalculateView() {
    sf::Vector2f size((float)window->getSize().x, (float)window->getSize().y);
    view = sf::View(center, size * scale);
}

void Simulator::run() {
    window = make_shared<sf::RenderWindow>(sf::VideoMode(1900, 1000), "SUMO Simulator");

    center = sf::Vector2f(
        (float)network->location.center().X - offset.X,
        (float)-network->location.center().Y + offset.Y
    );
    scale = max(
        (float)network->location.size().X / (float)window->getSize().x,
        (float)network->location.size().Y / (float)window->getSize().y
    );

    recalculateView();

    window->setView(view);

    bool         isLeftClickPressed = false;
    sf::Vector2f centerInitial;
    sf::Vector2f posMouseInitial;

    queue<clk::time_point> frames;

    clk::time_point lastFrame = clk::now();

    while(window->isOpen()) {
        sf::Event event;
        while(window->pollEvent(event)) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
            switch(event.type) {
                case sf::Event::Closed:
                    window->close();
                    break;
                case sf::Event::Resized:
                    onResize();
                    break;
                case sf::Event::MouseWheelScrolled:
                    onScroll(event.mouseWheelScroll.delta);
                    break;
                case sf::Event::MouseButtonPressed:
                    switch(event.mouseButton.button) {
                        case sf::Mouse::Button::Left:
                            isLeftClickPressed = true;
                            centerInitial      = center;
                            posMouseInitial    = sf::Vector2f(
                                (float)event.mouseButton.x,
                                (float)event.mouseButton.y
                            );
                            break;
                        default:
                            break;
                    }
                    break;
                case sf::Event::MouseButtonReleased:
                    switch(event.mouseButton.button) {
                        case sf::Mouse::Button::Left:
                            isLeftClickPressed = false;
                            break;
                        default:
                            break;
                    }
                    break;
                case sf::Event::MouseMoved:
                    if(isLeftClickPressed) {
                        sf::Vector2f mouse_pos(
                            (float)event.mouseMove.x,
                            (float)event.mouseMove.y
                        );
                        center = centerInitial - (mouse_pos - posMouseInitial) * scale;
                        recalculateView();
                    }
                    break;
                default:
                    break;
            }
#pragma GCC diagnostic pop
        }

        window->clear(sf::Color::White);

        window->setView(view);

        window->draw(roads.data(), roads.size(), sf::Triangles);
        window->draw(junctions.data(), junctions.size(), sf::Triangles);

        window->display();

        clk::time_point now = clk::now();
        frames.push(now);
        while(frames.front() < now - 1s) frames.pop();

        if(now - lastFrame >= 1s) {
            cout << frames.size() << " FPS" << endl;
            lastFrame = now;
        }
    }
}
