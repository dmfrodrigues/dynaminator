#include "UI/Simulator.hpp"

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/PrimitiveType.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <chrono>
#include <filesystem>
#include <limits>
#include <mapbox/earcut.hpp>
#include <queue>
#include <stdexcept>

#include "Dynamic/Env/Env.hpp"
#include "color/rgb/rgb.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "rapidxml_utils.hpp"

using namespace std;
using namespace UI;

using namespace rapidxml;

using namespace std::chrono_literals;

const float Simulator::VEHICLE_LENGTH = Dynamic::Env::Vehicle::LENGTH - 1.0;

Simulator::Simulator(filesystem::path configFilePath):
    delay(0ms),
    delayDelta(10ms),
    zRender(true),
    FPS_GOAL(120) {
    file<>         configFile(configFilePath.c_str());
    xml_document<> doc;
    doc.parse<0>(configFile.data());

    const xml_node<> &configuration = *doc.first_node("configuration");

    const xml_node<> &input = *configuration.first_node("input");

    filesystem::path netFilePath = configFilePath.parent_path() / input.first_node("net-file")->first_attribute("value")->value();

    network = SUMO::Network::loadFromFile(netFilePath);

    loadNetworkGUI();

    const xml_node<> *netstateEl = input.first_node("netstate-file");
    if(netstateEl) {
        filesystem::path netstateFilePath = configFilePath.parent_path() / netstateEl->first_attribute("value")->value();

        netState.emplace(netstateFilePath, ios_base::in);
    }
}

sf::Color Simulator::generateNewVehicleColor() const {
    uniform_real_distribution<float> hDist(0.0, 360.0);
    uniform_real_distribution<float> sDist(25.0, 100.0);
    uniform_real_distribution<float> vDist(50.0, 100.0);

    float h = hDist(gen);
    float s = sDist(gen);
    float v = vDist(gen);

    color::hsv<float> colorHSV({h, s, v});
    color::rgb<float> colorRGB;
    colorRGB = colorHSV;

    sf::Uint8 r = sf::Uint8(255.0 * color::get::red(colorRGB));
    sf::Uint8 g = sf::Uint8(255.0 * color::get::green(colorRGB));
    sf::Uint8 b = sf::Uint8(255.0 * color::get::blue(colorRGB));

    return sf::Color(r, g, b);
}

sf::Color color2SFML(const color::hsv<float> &c) {
    float r = color::get::red(c);
    float g = color::get::green(c);
    float b = color::get::blue(c);

    return sf::Color(
        255.0 * color::get::red(c),
        255.0 * color::get::green(c),
        255.0 * color::get::blue(c)
    );
}

color::hsv<float> Simulator::edgeColorHeight(SUMO::Length height) const {
    color::hsv<float> c;
    c = EDGE_COLOR;

    float z = height / 8.0;
    z       = max(z, 0.0f);

    float v = 100.0 * (1.0 - exp(log(1 - 0.25) * z));

    c.set(2, v);

    return c;
}

void Simulator::loadEdges() {
    for(const SUMO::Network::Edge &edge: network->getEdges()) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const SUMO::Length fromZ = edge.from->get().pos.Z;
        const SUMO::Length toZ   = edge.to->get().pos.Z;

        for(const SUMO::Network::Edge::Lane &lane: edge.lanes) {
            const SUMO::Shape &shape = lane.shape;
            assert(shape.size() >= 2);
            SUMO::Length L = 0.0;
            for(
                auto it1 = shape.begin(),
                     it2 = ++shape.begin();
                it2 != shape.end();
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

                SUMO::Length l1 = L;
                SUMO::Length l2 = L + Vector2::Magnitude(uv);

                double uProgress = shape.getProgress(l1);
                double vProgress = shape.getProgress(l2);

                double uZ = u.Z;
                if(uZ == 0.0)
                    uZ = (1.0 - uProgress) * fromZ + uProgress * toZ;
                color::hsv<float> uC = edgeColorHeight(uZ);

                double vZ = v.Z;
                if(vZ == 0.0)
                    vZ = (1.0 - vProgress) * fromZ + vProgress * toZ;
                color::hsv<float> vC = edgeColorHeight(vZ);

                // clang-format off
                vector<sf::Vertex> vertices = {
                    sf::Vertex(u1, color2SFML(uC)),
                    sf::Vertex(u2, color2SFML(uC)),
                    sf::Vertex(v1, color2SFML(vC)),

                    sf::Vertex(u2, color2SFML(uC)),
                    sf::Vertex(v1, color2SFML(vC)),
                    sf::Vertex(v2, color2SFML(vC))
                };
                // clang-format on

                // Add connection between consecutive segments of a lane
                auto it3 = it2;
                ++it3;
                if(it3 != shape.end()) {
                    SUMO::Coord w = *it3 - offset;

                    Vector2 vw = w - v;

                    Vector2 vwPerp(vw.Y, -vw.X);
                    vwPerp /= Vector2::Magnitude(vwPerp);

                    Vector2 v1_Coord = v + vwPerp * LANE_WIDTH / 2;
                    Vector2 v2_Coord = v - vwPerp * LANE_WIDTH / 2;

                    sf::Vector2f v1_(v1_Coord.X, -v1_Coord.Y);
                    sf::Vector2f v2_(v2_Coord.X, -v2_Coord.Y);

                    vertices.emplace_back(v1, color2SFML(vC));
                    vertices.emplace_back(v2, color2SFML(vC));
                    vertices.emplace_back(v1_, color2SFML(vC));

                    vertices.emplace_back(v2, color2SFML(vC));
                    vertices.emplace_back(v1_, color2SFML(vC));
                    vertices.emplace_back(v2_, color2SFML(vC));
                }

                if(!zRender) {
                    networkVertices.insert(
                        networkVertices.end(),
                        vertices.begin(),
                        vertices.end()
                    );
                } else {
                    networkMap.emplace((uZ + vZ) / 2.0, vertices);
                }

                L += Vector2::Magnitude(uv);
            }

            SUMO::Coord u = lane.shape.at(lane.shape.size() - 2) - offset;
            SUMO::Coord v = lane.shape.at(lane.shape.size() - 1) - offset;

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

            double vZ = v.Z;
            if(vZ == 0.0)
                vZ = toZ;

            vZ += 1e-3;

            // clang-format off
            vector<sf::Vertex> vertices = {
                sf::Vertex(arrowFront, ARROW_COLOR),
                sf::Vertex(arrowBack1, ARROW_COLOR),
                sf::Vertex(arrowBack2, ARROW_COLOR)
            };
            // clang-format on

            if(!zRender) {
                networkVertices.insert(
                    networkVertices.end(),
                    vertices.begin(),
                    vertices.end()
                );
            } else {
                networkMap.emplace(vZ, vertices);
            }
        }
    }
}

void Simulator::loadJunctions() {
    for(const SUMO::Network::Junction &junction: network->getJunctions()) {
        if(junction.type == SUMO::Network::Junction::Type::INTERNAL) continue;

        if(junction.shape.size() < 2) {
            throw runtime_error("Junction " + junction.id + " has less than 2 points in shape");
        }

        SUMO::Shape shape = junction.shape;
        for(SUMO::Coord &coord: shape)
            coord -= offset;

        using Point = std::array<double, 2>;

        vector<vector<Point>> shapes(1);
        for(size_t i = 0; i < shape.size(); ++i) {
            shapes[0].emplace_back(Point({shape.at(i).X, shape.at(i).Y}));
        }

        std::vector<size_t> indices = mapbox::earcut<size_t>(shapes);

        vector<sf::Vertex> junctionVertex;
        for(size_t i = 0; i < indices.size(); ++i) {
            SUMO::Coord uCoord = shape.at(indices[i]);

            sf::Vector2f u(uCoord.X, -uCoord.Y);

            junctionVertex.emplace_back(u, JUNCTION_COLOR);
        }

        for(const SUMO::Network::Edge::Lane &lane: junction.intLanes) {
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

                junctionVertex.emplace_back(u1, CONNECTION_COLOR);
                junctionVertex.emplace_back(u2, CONNECTION_COLOR);
                junctionVertex.emplace_back(v1, CONNECTION_COLOR);

                junctionVertex.emplace_back(u2, CONNECTION_COLOR);
                junctionVertex.emplace_back(v1, CONNECTION_COLOR);
                junctionVertex.emplace_back(v2, CONNECTION_COLOR);
            }
        }

        if(!zRender) {
            networkVertices.insert(
                networkVertices.end(),
                junctionVertex.begin(),
                junctionVertex.end()
            );
        } else {
            networkMap.emplace(junction.pos.Z, junctionVertex);
        }
    }
}

void Simulator::loadNetworkGUI() {
    offset = network->location.center();

    networkMap.clear();
    networkVertices.clear();

    loadEdges();

    loadJunctions();
}

double EPSILON = 1e-3;

void Simulator::loadVehicles() {
    vehiclesMap.clear();
    vehicleVertices.clear();

    for(const auto &[edgeID, tsEdge]: timestep.edges) {
        const SUMO::Network::Edge &edge = network->getEdge(edgeID);

        const SUMO::Length fromZ = edge.from->get().pos.Z;
        const SUMO::Length toZ   = edge.to->get().pos.Z;

        for(const auto &[laneID, tsLane]: tsEdge.lanes) {
            for(const auto &vehicle: tsLane.vehicles) {
                const SUMO::Network::Edge::Lane &lane = network->getEdge(edgeID).lanes.at(tsLane.index());

                double progress = vehicle.pos / lane.length;
                progress        = max(0.0, min(1.0, progress));

                SUMO::Coord pos = lane.shape.locationAtProgress(progress) - offset;
                Vector2     dir = lane.shape.directionAtProgress(progress);

                Vector2 dirPerp(dir.Y, -dir.X);

                SUMO::Coord rear = pos - dir * Dynamic::Env::Vehicle::LENGTH;

                SUMO::Coord rear1 = rear + dirPerp * VEHICLE_WIDTH / 2;
                SUMO::Coord rear2 = rear - dirPerp * VEHICLE_WIDTH / 2;

                sf::Vector2f sfPos(pos.X, -pos.Y);
                sf::Vector2f sfRear1(rear1.X, -rear1.Y);
                sf::Vector2f sfRear2(rear2.X, -rear2.Y);

                sf::Color c;

                if(vehicleColor.count(vehicle.id))
                    c = vehicleColor.at(vehicle.id);
                else
                    c = vehicleColor[vehicle.id] = generateNewVehicleColor();

                SUMO::Length z = pos.Z;
                if(z == 0.0)
                    z = (1.0 - progress) * fromZ + progress * toZ;

                z += 0.50;

                // clang-format off
                vector<sf::Vertex> vertices = {
                    sf::Vertex(sfPos, c),
                    sf::Vertex(sfRear1, c),
                    sf::Vertex(sfRear2, c)
                };
                // clang-format on

                if(!zRender) {
                    vehicleVertices.insert(
                        vehicleVertices.end(),
                        vertices.begin(),
                        vertices.end()
                    );
                } else {
                    vehiclesMap.emplace(z, vertices);
                }
            }
        }
    }
}

void Simulator::loadTrafficLights() {
    trafficLightsMap.clear();
    trafficLightVertices.clear();

    for(const SUMO::Network::Edge &edge: network->getEdges()) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        for(const SUMO::Network::Edge::Lane &lane: edge.lanes) {
            vector<SUMO::Network::TrafficLightLogic::Phase::State> states;
            for(const SUMO::Network::Connection &connection: lane.getOutgoing()) {
                if(!connection.tl.has_value()) continue;
                states.emplace_back(connection.getTrafficLightState(timestep.time));
            }

            if(states.empty()) continue;

            SUMO::Shape shape = lane.getShape();

            SUMO::Coord p   = shape.back() - offset;
            Vector2     dir = shape.directionAtProgress(1.0);
            dir /= Vector2::Magnitude(dir);
            SUMO::Coord dirPerp(-dir.Y, dir.X);

            SUMO::Coord pRight = p - dirPerp * LANE_WIDTH / 2;

            float TRAFFIC_LIGHT_WIDTH = LANE_WIDTH / states.size();

            for(size_t i = 0; i < states.size(); ++i) {
                SUMO::Coord l = pRight + dirPerp * double(i) * TRAFFIC_LIGHT_WIDTH;
                SUMO::Coord r = pRight + dirPerp * double(i + 1) * TRAFFIC_LIGHT_WIDTH;

                SUMO::Coord p1Coord = l + dir * TRAFFIC_LIGHT_LENGTH / 2;
                SUMO::Coord p2Coord = l - dir * TRAFFIC_LIGHT_LENGTH / 2;
                SUMO::Coord p3Coord = r + dir * TRAFFIC_LIGHT_LENGTH / 2;
                SUMO::Coord p4Coord = r - dir * TRAFFIC_LIGHT_LENGTH / 2;

                sf::Vector2f p1(p1Coord.X, -p1Coord.Y);
                sf::Vector2f p2(p2Coord.X, -p2Coord.Y);
                sf::Vector2f p3(p3Coord.X, -p3Coord.Y);
                sf::Vector2f p4(p4Coord.X, -p4Coord.Y);

                sf::Color c;
                switch(states[i]) {
                    case SUMO::Network::TrafficLightLogic::Phase::State::RED:
                        c = sf::Color::Red;
                        break;
                    case SUMO::Network::TrafficLightLogic::Phase::State::YELLOW_START:
                    case SUMO::Network::TrafficLightLogic::Phase::State::YELLOW_STOP:
                        c = sf::Color::Yellow;
                        break;
                    case SUMO::Network::TrafficLightLogic::Phase::State::GREEN_PRIORITY:
                    case SUMO::Network::TrafficLightLogic::Phase::State::GREEN_RIGHT:
                        c = sf::Color::Green;
                        break;
                    case SUMO::Network::TrafficLightLogic::Phase::State::GREEN_NOPRIORITY:
                        c = sf::Color(0, 128, 0);
                        break;
                    case SUMO::Network::TrafficLightLogic::Phase::State::OFF:
                    case SUMO::Network::TrafficLightLogic::Phase::State::OFF_YIELD:
                        c = sf::Color(128, 128, 128);
                        break;
                    default:
                        throw runtime_error("Unknown traffic light state");
                }

                // clang-format off
                vector<sf::Vertex> vertices = {
                    sf::Vertex(p1, c),
                    sf::Vertex(p2, c),
                    sf::Vertex(p3, c),
                    sf::Vertex(p2, c),
                    sf::Vertex(p3, c),
                    sf::Vertex(p4, c),
                };
                // clang-format on

                if(!zRender) {
                    trafficLightVertices.insert(
                        trafficLightVertices.end(),
                        vertices.begin(),
                        vertices.end()
                    );
                } else {
                    trafficLightsMap.emplace(numeric_limits<double>::infinity(), vertices);
                }
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
    window = make_shared<sf::RenderWindow>(sf::VideoMode(1900, 1000), "DynamiNATOR");
    window->setFramerateLimit(FPS_GOAL);

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

    clk::time_point lastFrame          = clk::now();
    clk::time_point lastTimestepUpdate = clk::now();

    if(
        netState.has_value() && netState.value()
    ) {
        // clang-format on

        netState.value() >> timestep;

        loadVehicles();
        loadTrafficLights();

        lastTimestepUpdate = clk::now();
    }

    while(window->isOpen()) {
        clk::time_point now = clk::now();

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
                // case sf::Event::TextEntered:
                //     switch(toupper((int)event.text.unicode)) {
                //         case ' ':
                //             running = !running;
                //             break;
                //         default:
                //             break;
                //     }
                //     break;
                case sf::Event::KeyPressed:
                    switch(event.key.code) {
                        case sf::Keyboard::Key::Space:
                            running = !running;
                            break;
                        case sf::Keyboard::Key::Up:
                            delay += delayDelta;
                            cerr << "Delay: " << delay.count() << " [ms]" << endl;
                            break;
                        case sf::Keyboard::Key::Down:
                            delay -= delayDelta;
                            delay = max(delay, 0ms);
                            cerr << "Delay: " << delay.count() << " [ms]" << endl;
                            break;
                        case sf::Keyboard::Key::Right:
                            if(netState.has_value() && netState.value()) {
                                netState.value() >> timestep;
                                loadVehicles();
                                loadTrafficLights();
                                lastTimestepUpdate = now;
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
#pragma GCC diagnostic pop
        }

        // clang-format off
        if(
            running &&
            netState.has_value() &&
            netState.value() &&
            now - lastTimestepUpdate >= delay   
        ) {
            // clang-format on

            netState.value() >> timestep;

            loadVehicles();
            loadTrafficLights();

            lastTimestepUpdate = now;
        }

        window->clear(sf::Color::White);

        window->setView(view);

        draw();

        window->display();

        frames.push(now);
        while(frames.front() < now - 1s) frames.pop();

        if(now - lastFrame >= 1s) {
            cout << frames.size() << " FPS, t=" << timestep.time << endl;
            lastFrame = now;
        }
    }
}

void Simulator::draw() {
    const double deltaZ = 0.01;

    if(!zRender) {
        window->draw(networkVertices.data(), networkVertices.size(), sf::Triangles);
        window->draw(vehicleVertices.data(), vehicleVertices.size(), sf::Triangles);
        window->draw(trafficLightVertices.data(), trafficLightVertices.size(), sf::Triangles);
    } else {
        // auto it1 = networkMap.begin();
        // auto it2 = vehiclesMap.begin();
        // auto it3 = trafficLightsMap.begin();

        // double z = numeric_limits<double>::infinity();

        // z = min(z, it1->first);
        // z = min(z, it2->first);
        // z = min(z, it3->first);

        // z += deltaZ;

        // vector<sf::Vertex> vertices;
        // vertices.reserve(networkMap.size() + vehiclesMap.size() + trafficLightsMap.size());

        // while(
        //     it1 != networkMap.end() || it2 != vehiclesMap.end() || it3 != trafficLightsMap.end()
        // ) {
        //     double zNext = numeric_limits<double>::infinity();

        //     while(it1 != networkMap.end() && it1->first <= z) {
        //         vertices.insert(vertices.end(), it1->second.begin(), it1->second.end());
        //         ++it1;
        //         if(it1 != networkMap.end()) zNext = min(zNext, it1->first);
        //     }
        //     while(it2 != vehiclesMap.end() && it2->first <= z) {
        //         vertices.insert(vertices.end(), it2->second.begin(), it2->second.end());
        //         ++it2;
        //         if(it2 != vehiclesMap.end()) zNext = min(zNext, it2->first);
        //     }
        //     while(it3 != trafficLightsMap.end() && it3->first <= z) {
        //         vertices.insert(vertices.end(), it3->second.begin(), it3->second.end());
        //         ++it3;
        //         if(it3 != trafficLightsMap.end()) zNext = min(zNext, it3->first);
        //     }

        //     z = zNext;
        //     z += deltaZ;
        // }

        multimap<double, vector<sf::Vertex>> verticesMap;
        verticesMap.insert(networkMap.begin(), networkMap.end());
        verticesMap.insert(vehiclesMap.begin(), vehiclesMap.end());
        verticesMap.insert(trafficLightsMap.begin(), trafficLightsMap.end());

        vector<sf::Vertex> vertices;
        for(const auto &[_, v]: verticesMap) {
            vertices.insert(vertices.end(), v.begin(), v.end());
        }

        window->draw(vertices.data(), vertices.size(), sf::Triangles);
    }
}
