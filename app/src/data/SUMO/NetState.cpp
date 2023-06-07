#include "data/SUMO/NetState.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "utils/xml.hpp"

using namespace std;
using namespace SUMO;
using namespace rapidxml;

namespace xml = utils::xml;

namespace fs = std::filesystem;

NetState::Timestep::Edge::Lane::Vehicle &NetState::Timestep::Edge::Lane::addVehicle(Vehicle::ID id, Length pos, Speed speed) {
    vehicles.emplace_back(Vehicle{id, pos, speed});
    return vehicles.back();
}

NetState::Timestep::Edge::Lane &NetState::Timestep::Edge::addLane(Network::Edge::Lane::ID id) {
    lanes[id] = Lane{id};
    return lanes[id];
}

NetState::Timestep::Edge &NetState::Timestep::addEdge(Network::Edge::ID id) {
    edges[id] = Edge{id};
    return edges[id];
}

NetState::Timestep &NetState::addTimestep(Time time) {
    timesteps.emplace_back(Timestep{time});
    return timesteps.back();
}

NetState::Timestep &NetState::addTimestep(const Timestep &timestep) {
    timesteps.emplace_back(timestep);
    return timesteps.back();
}

void NetState::saveToFile(const string &filePath) {
    sort(
        timesteps.begin(),
        timesteps.end(),
        [](const Timestep &a, const Timestep &b) -> bool {
            return a.time < b.time;
        }
    );

    xml_document<> doc;
    xml_node<>    &netstateEl = *doc.allocate_node(node_element, "netstate");
    doc.append_node(&netstateEl);

    for(const Timestep &timestep: timesteps) {
        xml_node<> &timestepEl = *doc.allocate_node(node_element, "timestep");
        netstateEl.append_node(&timestepEl);

        xml::add_attribute(timestepEl, "time", timestep.time);

        for(const auto &[edgeID, edge]: timestep.edges) {
            xml_node<> &edgeEl = *doc.allocate_node(node_element, "edge");
            timestepEl.append_node(&edgeEl);

            xml::add_attribute(edgeEl, "id", edge.id);

            for(const auto &[laneID, lane]: edge.lanes){
                xml_node<> &laneEl = *doc.allocate_node(node_element, "lane");
                edgeEl.append_node(&laneEl);

                xml::add_attribute(laneEl, "id", lane.id);

                for(const Timestep::Edge::Lane::Vehicle &vehicle: lane.vehicles) {
                    xml_node<> &vehicleEl = *doc.allocate_node(node_element, "vehicle");
                    laneEl.append_node(&vehicleEl);

                    xml::add_attribute(vehicleEl, "id", vehicle.id);
                    xml::add_attribute(vehicleEl, "pos", vehicle.pos);
                    xml::add_attribute(vehicleEl, "speed", vehicle.speed);
                }
            }
        }
    }

    ofstream os;
    os.exceptions(ios_base::failbit | ios_base::badbit);
    fs::path p = fs::path(filePath).parent_path();
    if(!fs::is_directory(p)) {
        cerr << "Creating directory " << p << endl;
        if(!fs::create_directory(p)) {
            throw ios_base::failure("Could not create directory " + p.string());
        }
    }
    try {
        os.open(filePath);
    } catch(const ios_base::failure &ex) {
        throw ios_base::failure("Could not open file " + filePath);
    }

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    os << doc;
}
