#include "data/SUMO/NetState.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <mutex>

#include "rapidxml.hpp"
#include "utils/stringify.hpp"
#include "utils/xml.hpp"

using namespace std;
using namespace SUMO;
using namespace rapidxml;
using namespace utils::stringify;

namespace xml = utils::xml;

namespace fs = std::filesystem;

NetState::Timestep::Edge::Lane::Vehicle &NetState::Timestep::Edge::Lane::addVehicle(Vehicle::ID vehicleID, Length pos, Speed speed) {
    vehicles.emplace_back(Vehicle{vehicleID, pos, speed});
    return vehicles.back();
}

Index NetState::Timestep::Edge::Lane::index() const {
    size_t i = id.find_last_of("_");
    return stringify<size_t>::fromString(id.substr(i + 1));
}

NetState::Timestep::Edge::Lane &NetState::Timestep::Edge::addLane(Network::Edge::Lane::ID laneID) {
    lanes[laneID] = Lane{laneID};
    return lanes[laneID];
}

NetState::Timestep::Edge &NetState::Timestep::addEdge(Network::Edge::ID edgeID) {
    edges[edgeID] = Edge{edgeID};
    return edges[edgeID];
}

NetState::NetState(const string &filePath, ios_base::openmode openMode) {
    // is.exceptions(ios_base::failbit | ios_base::badbit);
    os.exceptions(ios_base::failbit | ios_base::badbit);

    fs::path p = fs::path(filePath).parent_path();

    if(openMode & ios_base::out) {
        if(!fs::is_directory(p)) {
            spdlog::info("Creating directory {}", p.string());
            if(!fs::create_directory(p)) {
                throw ios_base::failure("Could not create directory " + p.string());
            }
        }
    }

    try {
        if(openMode & ios_base::in) is.open(filePath);
        if(openMode & ios_base::out) {
            os.open(filePath);
            os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
               << "\n";
            os << "<netstate>"
               << "\n";
        }
    } catch(const ios_base::failure &ex) {
        throw ios_base::failure("Could not open file " + filePath);
    }
}

NetState::operator bool() const {
    return is && os;
}

void NetState::Timestep::toXML(xml_document<> &doc) const {
    xml_node<> &timestepEl = *doc.allocate_node(node_element, "timestep");
    doc.append_node(&timestepEl);

    xml::add_attribute(timestepEl, "time", time);

    for(const auto &[edgeID, edge]: edges) {
        xml_node<> &edgeEl = *doc.allocate_node(node_element, "edge");
        timestepEl.append_node(&edgeEl);

        xml::add_attribute(edgeEl, "id", edge.id);

        for(const auto &[laneID, lane]: edge.lanes) {
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

NetState::Timestep NetState::Timestep::fromXML(xml_node<> &timestepEl) {
    // clang-format off
    Timestep ret{
        stringify<Time>::fromString(timestepEl.first_attribute("time")->value())
    };
    // clang-format on

    for(
        xml_node<> *edgeEl = timestepEl.first_node("edge");
        edgeEl;
        edgeEl = edgeEl->next_sibling("edge")
    ) {
        Edge &edge = ret.addEdge(
            edgeEl->first_attribute("id")->value()
        );

        for(
            xml_node<> *laneEl = edgeEl->first_node("lane");
            laneEl;
            laneEl = laneEl->next_sibling("lane")
        ) {
            xml_attribute<> *laneIDAttr = laneEl->first_attribute("id");

            if(laneIDAttr == nullptr) throw runtime_error("Lane ID attribute not found; t=" + to_string(ret.time) + ", edge is " + edge.id);

            Edge::Lane &lane = edge.addLane(
                laneIDAttr->value()
            );

            for(
                xml_node<> *vehicleEl = laneEl->first_node("vehicle");
                vehicleEl;
                vehicleEl = vehicleEl->next_sibling("vehicle")
            ) {
                lane.addVehicle(
                    vehicleEl->first_attribute("id")->value(),
                    stringify<Length>::fromString(vehicleEl->first_attribute("pos")->value()),
                    stringify<Speed>::fromString(vehicleEl->first_attribute("speed")->value())
                );
            }
        }
    }

    return ret;
}

NetState &NetState::operator<<(const NetState::Timestep &timestep) {
    lock_guard<mutex> lockAddQueue(*this);

    futuresQueue.push(async(launch::async, [timestep]() -> stringstream {
        stringstream   ss;
        xml_document<> doc;
        timestep.toXML(doc);
        ss << doc;
        return ss;
    }));

    if(futuresQueue.size() > maxQueueSize) {
        pool.push([this, timestep](int) -> void {
            lock_guard<mutex> lockPrint(*this);
            while(futuresQueue.size() > maxQueueSize) {
                stringstream ss = futuresQueue.front().get();
                os << ss.rdbuf();
                futuresQueue.pop();
            }
        });
    }

    return *this;
}

NetState &NetState::operator>>(NetState::Timestep &timestep) {
    if(tsBuffer.empty()) {
        stringstream ss;
        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";

        string line;

        while(getline(is, line)) {
            if(line.find("<netstate>") != string::npos) continue;
            if(line.find("<?xml") != string::npos) continue;
            ss << line;
            if(line.find("</timestep>") != string::npos) break;
        }

        if(!is) {
            return *this;
        }

        const string       s = ss.str();
        unique_ptr<char[]> c = make_unique<char[]>(s.size() + 1);
        strcpy(c.get(), s.c_str());

        xml_document<> doc;
        doc.parse<0>(c.get());

        for(
            xml_node<> *timestepEl = doc.first_node("timestep");
            timestepEl;
            timestepEl = timestepEl->next_sibling("timestep")
        ) {
            Timestep ts = Timestep::fromXML(*timestepEl);
            tsBuffer.push(ts);
        }
    }

    assert(tsBuffer.size() > 0);

    timestep = tsBuffer.front();

    tsBuffer.pop();

    return *this;
}

void NetState::close() {
    if(is.is_open()) is.close();
    if(os.is_open()) {
        lock_guard<mutex> lock(*this);
        while(!futuresQueue.empty()) {
            stringstream ss = futuresQueue.front().get();
            os << ss.rdbuf();
            futuresQueue.pop();
        }
        os << "</netstate>"
           << "\n"
           << flush;
        os.close();
    }
}

NetState::~NetState() {
    close();
}
