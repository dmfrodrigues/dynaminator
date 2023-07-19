#include "data/SUMO/Routes.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rapidxml.hpp"
#include "utils/xml.hpp"

using namespace std;
using namespace SUMO;
using namespace rapidxml;
using namespace utils::stringify;

namespace fs = std::filesystem;

namespace xml = utils::xml;

Routes::VehicleFlow::VehicleFlow(
    ID    id_,
    Route route_
):
    id(id_),
    route(route_) {}

// clang-format off
void Routes::VehicleFlow::setColor         (color::rgb<float>  color_      ) { color       = color_        ; }
void Routes::VehicleFlow::setFromTaz       (TAZ::ID            fromTaz_    ) { fromTaz     = fromTaz_      ; }
void Routes::VehicleFlow::setToTaz         (TAZ::ID            toTaz_      ) { toTaz       = toTaz_        ; }
void Routes::VehicleFlow::setDepartPos     (DepartPos          departPos_  ) { departPos   = departPos_    ; }
void Routes::VehicleFlow::setDepartSpeed   (DepartSpeed        departSpeed_) { departSpeed = departSpeed_  ; }
// clang-format on

void Routes::VehicleFlow::toXML(xml_node<> &vehicleFlowEl) const {
    xml_document<> &doc = *vehicleFlowEl.document();

    // clang-format off
    xml::add_attribute(vehicleFlowEl, "id", id   );
    if(color       .has_value()) xml::add_attribute(vehicleFlowEl, "color"        , color        .value());
    if(fromTaz     .has_value()) xml::add_attribute(vehicleFlowEl, "fromTaz"      , fromTaz      .value());
    if(toTaz       .has_value()) xml::add_attribute(vehicleFlowEl, "toTaz"        , toTaz        .value());
    if(departPos   .has_value()) xml::add_attribute(vehicleFlowEl, "departPos"    , departPos    .value());
    if(departSpeed .has_value()) xml::add_attribute(vehicleFlowEl, "departSpeed"  , departSpeed  .value());
    // clang-format on

    xml_node<> &routeEl = *doc.allocate_node(node_element, "route");
    vehicleFlowEl.append_node(&routeEl);

    xml::add_attribute(routeEl, "edges", route);
}

Routes::Vehicle::Vehicle(
    ID    id_,
    Route route_,
    Time  depart_
):
    VehicleFlow(id_, route_),
    depart(depart_) {}

void Routes::Vehicle::toXML(xml_node<> &vehicleEl) const {
    VehicleFlow::toXML(vehicleEl);

    xml::add_attribute(vehicleEl, "depart", depart);
}

void Routes::Vehicle::addToXML(xml_node<> &routesEl) const {
    xml_document<> &doc = *routesEl.document();

    xml_node<> &vehicleEl = *doc.allocate_node(node_element, "vehicle");
    routesEl.append_node(&vehicleEl);

    toXML(vehicleEl);
}

bool Routes::Vehicle::operator<(const Vehicle &other) const {
    return (depart < other.depart || (!(depart > other.depart) && id < other.id));
}

Routes::Flow::PolicyVehsPerHour::PolicyVehsPerHour(double vehsPerHour_):
    vehsPerHour(vehsPerHour_) {}

Routes::Flow::Flow(
    ID                 id_,
    Route              route_,
    Time               begin_,
    Time               end_,
    shared_ptr<Policy> policy_
):
    VehicleFlow(id_, route_),
    begin(begin_),
    end(end_),
    policy(policy_) {}

void Routes::Flow::toXML(xml_node<> &flowEl) const {
    VehicleFlow::toXML(flowEl);

    // clang-format off
    xml::add_attribute(flowEl, "begin"  , begin);
    xml::add_attribute(flowEl, "end"    , end  );
    // clang-format on

    policy->saveToXML(flowEl);
}

void Routes::Flow::addToXML(xml_node<> &routesEl) const {
    xml_document<> &doc = *routesEl.document();

    xml_node<> &flowEl = *doc.allocate_node(node_element, "flow");
    routesEl.append_node(&flowEl);

    toXML(flowEl);
}

Routes::Vehicle &Routes::createVehicle(
    ID    id,
    Route route,
    Time  depart
) {
    shared_ptr<Vehicle> vehicle(new Vehicle(id, route, depart));
    vehicles.emplace(vehicle);
    return *vehicle;
}

Routes::Flow &Routes::createFlow(
    SUMO::ID                 id,
    Route                    route,
    SUMO::Time               begin,
    SUMO::Time               end,
    shared_ptr<Flow::Policy> policy
) {
    shared_ptr<Flow> flow(new Flow(id, route, begin, end, policy));
    flows.emplace_back(flow);
    return *flow;
}

void Routes::saveToFile(const string &filePath) const {
    xml_document<> doc;
    xml_node<>    &routesEl = *doc.allocate_node(node_element, "routes");
    doc.append_node(&routesEl);

    for(const shared_ptr<Flow> &flow: flows) {
        flow->addToXML(routesEl);
    }

    for(const shared_ptr<Vehicle> &vehicle: vehicles) {
        vehicle->addToXML(routesEl);
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

void Routes::Flow::PolicyVehsPerHour::saveToXML(xml_node<> &flowEl) const {
    xml::add_attribute(flowEl, "vehsPerHour", vehsPerHour);
}

void Routes::Flow::PolicyPeriod::saveToXML(xml_node<> &flowEl) const {
    if(f.has_value()) {
        xml::add_attribute(flowEl, "period", f.value());
        return;
    } else if(s.has_value()) {
        xml::add_attribute(flowEl, "period", s.value());
        return;
    }
    throw runtime_error("PolicyPeriod::saveToXML: either f or s must be set, but neither were set.");
}

void Routes::Flow::PolicyProbability::saveToXML(xml_node<> &flowEl) const {
    xml::add_attribute(flowEl, "probability", probability);
}
