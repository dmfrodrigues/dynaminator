#include "data/SUMO/Routes.hpp"

using namespace std;
using namespace SUMO;

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
