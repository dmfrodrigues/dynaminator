#include "data/SUMO/NetState.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
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

NetState::NetState(const string &filePath) {
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

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << "\n";
    os << "<netstate>" << "\n";
}

void NetState::Timestep::toXML(xml_document<> &doc) const {
    xml_node<> &timestepEl = *doc.allocate_node(node_element, "timestep");
    doc.append_node(&timestepEl);

    xml::add_attribute(timestepEl, "time", time);

    for(const auto &[edgeID, edge]: edges) {
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

NetState &NetState::operator<<(const NetState::Timestep &timestep) {
    xml_document<> doc;
    timestep.toXML(doc);
    os << doc;
    return *this;
}

void NetState::close() {
    os << "</netstate>" << "\n" << flush;
    os.close();
}

NetState::~NetState() {
    close();
}
