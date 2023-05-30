#include "data/SUMO/Routes.hpp"
#include <stdexcept>

#include "utils/xml.hpp"

using namespace std;
using namespace SUMO;
using namespace rapidxml;
using namespace utils::stringify;

namespace xml = utils::xml;

Routes::Flow::PolicyVehsPerHour::PolicyVehsPerHour(double vehsPerHour_): vehsPerHour(vehsPerHour_) {}

Routes::Flow::Flow(ID id_, Time begin_, Time end_, shared_ptr<Policy> policy_):
    id(id_),
    begin(begin_),
    end(end_),
    policy(policy_) {}

// clang-format off
void Routes::Flow::setColor         (color::rgb<float>  color_      ) { color       = color_        ; }
void Routes::Flow::setFromTaz       (TAZ::ID            fromTaz_    ) { fromTaz     = fromTaz_      ; }
void Routes::Flow::setToTaz         (TAZ::ID            toTaz_      ) { toTaz       = toTaz_        ; }
void Routes::Flow::setDepartPos     (DepartPos          departPos_  ) { departPos   = departPos_    ; }
void Routes::Flow::setDepartSpeed   (DepartSpeed        departSpeed_) { departSpeed = departSpeed_  ; }
// clang-format on

Routes::Flow &Routes::createFlow(SUMO::ID id, SUMO::Time begin, SUMO::Time end, std::shared_ptr<Flow::Policy> policy) {
    flows.emplace(id, Flow(id, begin, end, policy));
    return flows.at(id);
}

void Routes::saveToFile(const string &filePath) const {
    xml_document<> doc;
    xml_node<>    &routesEl = *doc.allocate_node(node_element, "routes");
    doc.append_node(&routesEl);

    for(const auto &[flowID, flow]: flows){
        xml_node<> &flowEl = *doc.allocate_node(node_element, "flow");
        routesEl.append_node(&flowEl);

        // clang-format off
        xml::add_attribute(flowEl, "id"     , flow.id   );
        xml::add_attribute(flowEl, "begin"  , flow.begin);
        xml::add_attribute(flowEl, "end"    , flow.end  );
        if(flow.color       .has_value()) xml::add_attribute(flowEl, "color"        , flow.color        .value());
        if(flow.fromTaz     .has_value()) xml::add_attribute(flowEl, "fromTaz"      , flow.fromTaz      .value());
        if(flow.toTaz       .has_value()) xml::add_attribute(flowEl, "toTaz"        , flow.toTaz        .value());
        if(flow.departPos   .has_value()) xml::add_attribute(flowEl, "departPos"    , flow.departPos    .value());
        if(flow.departSpeed .has_value()) xml::add_attribute(flowEl, "departSpeed"  , flow.departSpeed  .value());
        // clang-format on

        flow.policy->saveToXML(flowEl);
    }
}

void Routes::Flow::PolicyVehsPerHour::saveToXML(xml_node<> &flowEl) const {
    xml::add_attribute(flowEl, "vehsPerHour", vehsPerHour);
}

void Routes::Flow::PolicyPeriod::saveToXML(xml_node<> &flowEl) const {
    if(f.has_value())
        xml::add_attribute(flowEl, "period", f.value());
    else if(s.has_value())
        xml::add_attribute(flowEl, "period", s.value());
    throw runtime_error("PolicyPeriod::saveToXML: either f or s must be set, but neither were set.");
}

void Routes::Flow::PolicyProbability::saveToXML(xml_node<> &flowEl) const {
    xml::add_attribute(flowEl, "probability", probability);
}
