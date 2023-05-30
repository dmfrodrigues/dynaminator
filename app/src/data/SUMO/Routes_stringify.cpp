#include "data/SUMO/Routes.hpp"
#include "utils/invertMap.hpp"

using namespace std;
using namespace SUMO;
using namespace utils::stringify;

// clang-format off
const unordered_map<string, Routes::Flow::DepartPos::Enum> str2departPos = {
    {"random"       , Routes::Flow::DepartPos::Enum::RANDOM     },
    {"free"         , Routes::Flow::DepartPos::Enum::FREE       },
    {"random_free"  , Routes::Flow::DepartPos::Enum::RANDOM_FREE},
    {"base"         , Routes::Flow::DepartPos::Enum::BASE       },
    {"last"         , Routes::Flow::DepartPos::Enum::LAST       },
    {"stop"         , Routes::Flow::DepartPos::Enum::STOP       }
};
const unordered_map<Routes::Flow::DepartPos::Enum, string> departPos2str = utils::invertMap(str2departPos);

const unordered_map<string, Routes::Flow::DepartSpeed::Enum> str2departSpeed = {
    {"random"       , Routes::Flow::DepartSpeed::Enum::RANDOM     },
    {"max"          , Routes::Flow::DepartSpeed::Enum::MAX        },
    {"desired"      , Routes::Flow::DepartSpeed::Enum::DESIRED    },
    {"speedLimit"   , Routes::Flow::DepartSpeed::Enum::SPEED_LIMIT},
    {"last"         , Routes::Flow::DepartSpeed::Enum::LAST       },
    {"avg"          , Routes::Flow::DepartSpeed::Enum::AVG        }
};
const unordered_map<Routes::Flow::DepartSpeed::Enum, string> departSpeed2str = utils::invertMap(str2departSpeed);
// clang-format on

Routes::Flow::DepartPos stringify<Routes::Flow::DepartPos>::fromString(const string &s) {
    if(str2departPos.count(s))
        return { .e = str2departPos.at(s) };
    else
        return { .f = stringify<float>::fromString(s) };
}

string stringify<Routes::Flow::DepartPos>::toString(const SUMO::Routes::Flow::DepartPos &t){
    if(t.e.has_value())
        return departPos2str.at(*t.e);
    else if(t.f.has_value())
        return stringify<float>::toString(*t.f);
    throw runtime_error("stringify<Routes::Flow::DepartPos>::toString: either f or s must be set, but neither were set.");
}

Routes::Flow::DepartSpeed stringify<Routes::Flow::DepartSpeed>::fromString(const string &s) {
    if(str2departPos.count(s))
        return { .e = str2departSpeed.at(s) };
    else
        return { .f = stringify<float>::fromString(s) };
}

string stringify<Routes::Flow::DepartSpeed>::toString(const SUMO::Routes::Flow::DepartSpeed &t){
    if(t.e.has_value())
        return departSpeed2str.at(*t.e);
    else if(t.f.has_value())
        return stringify<float>::toString(*t.f);
    throw runtime_error("stringify<Routes::Flow::DepartSpeed>::toString: either f or s must be set, but neither were set.");
}
