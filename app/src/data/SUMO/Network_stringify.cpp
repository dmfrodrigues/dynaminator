#include "data/SUMO/Network.hpp"
#include "utils/invertMap.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO;
using namespace utils::stringify;

typedef Network::Junction          Junction;
typedef Network::Edge              Edge;
typedef Network::Edge::Lane        Lane;
typedef Network::TrafficLightLogic TrafficLightLogic;
typedef Network::Connection        Connection;

// clang-format off
const unordered_map<string, Edge::Function> str2function = {
    {"internal"     , Edge::Function::INTERNAL},
    {"connector"    , Edge::Function::CONNECTOR},
    {"crossing"     , Edge::Function::CROSSING},
    {"walkingarea"  , Edge::Function::WALKINGAREA},
    {"normal"       , Edge::Function::NORMAL}
};
const unordered_map<Edge::Function, string> function2str = utils::invertMap(str2function);

const unordered_map<string, Junction::Type> str2junctionType = {
    {"priority"                     , Junction::Type::PRIORITY                   },
    {"traffic_light"                , Junction::Type::TRAFFIC_LIGHT              },
    {"right_before_left"            , Junction::Type::RIGHT_BEFORE_LEFT          },
    {"left_before_right"            , Junction::Type::LEFT_BEFORE_RIGHT          },
    {"unregulated"                  , Junction::Type::UNREGULATED                },
    {"traffic_light_unregulated"    , Junction::Type::TRAFFIC_LIGHT_UNREGULATED  },
    {"priority_stop"                , Junction::Type::PRIORITY_STOP              },
    {"allway_stop"                  , Junction::Type::ALLWAY_STOP                },
    {"rail_signal"                  , Junction::Type::RAIL_SIGNAL                },
    {"zipper"                       , Junction::Type::ZIPPER                     },
    {"rail_crossing"                , Junction::Type::RAIL_CROSSING              },
    {"traffic_light_right_on_red"   , Junction::Type::TRAFFIC_LIGHT_RIGHT_ON_RED },
    {"dead_end"                     , Junction::Type::DEAD_END                   },

    {"internal"                     , Junction::Type::INTERNAL                   },

    {"unknown"                      , Junction::Type::UNKNOWN                    },
    {"district"                     , Junction::Type::DISTRICT                   }
};
const unordered_map<Junction::Type, string> junctionType2str = utils::invertMap(str2junctionType);

const unordered_map<string, TrafficLightLogic::Type> str2tlType = {
    {"static"       , TrafficLightLogic::Type::STATIC},
    {"actuated"     , TrafficLightLogic::Type::ACTUATED},
    {"delay_based"  , TrafficLightLogic::Type::DELAY_BASED}
};
const unordered_map<TrafficLightLogic::Type, string> tlType2str = utils::invertMap(str2tlType);

const unordered_map<string, TrafficLightLogic::Phase::State> str2tlState = {
    {"r", TrafficLightLogic::Phase::State::RED},
    {"y", TrafficLightLogic::Phase::State::YELLOW_STOP},
    {"g", TrafficLightLogic::Phase::State::GREEN_NOPRIORITY},
    {"G", TrafficLightLogic::Phase::State::GREEN_PRIORITY},
    {"s", TrafficLightLogic::Phase::State::GREEN_RIGHT},
    {"u", TrafficLightLogic::Phase::State::YELLOW_START},
    {"o", TrafficLightLogic::Phase::State::OFF_YIELD},
    {"O", TrafficLightLogic::Phase::State::OFF}
};
const unordered_map<TrafficLightLogic::Phase::State, string> tlState2str = utils::invertMap(str2tlState);

const unordered_map<string, Connection::Direction> str2connDir = {
    {"invalid"  , Connection::Direction::INVALID},
    {"s"        , Connection::Direction::STRAIGHT},
    {"t"        , Connection::Direction::TURN},
    {"l"        , Connection::Direction::LEFT},
    {"r"        , Connection::Direction::RIGHT},
    {"L"        , Connection::Direction::PARTIALLY_LEFT},
    {"R"        , Connection::Direction::PARTIALLY_RIGHT}
};
const unordered_map<Connection::Direction, string> connDir2str = utils::invertMap(str2connDir);

const unordered_map<string, Connection::State> str2connState = {
    {"-", Connection::State::DEAD_END},
    {"=", Connection::State::EQUAL},
    {"m", Connection::State::MINOR_LINK},
    {"M", Connection::State::MAJOR_LINK},
    {"O", Connection::State::CONTROLLER_OFF},
    {"o", Connection::State::YELLOW_FLASHING},
    {"y", Connection::State::YELLOW_MINOR_LINK},
    {"Y", Connection::State::YELLOW_MAJOR_LINK},
    {"r", Connection::State::RED},
    {"g", Connection::State::GREEN_MINOR},
    {"G", Connection::State::GREEN_MAJOR}
};
const unordered_map<Connection::State, string> connState2str = utils::invertMap(str2connState);
// clang-format on

Edge::Function stringify<Edge::Function>::fromString(const string &s) {
    auto it = str2function.find(s);
    if(it != str2function.end())
        return it->second;
    else
        return Edge::Function::NORMAL;
}

string stringify<Edge::Function>::toString(const Edge::Function &t) {
    return function2str.at(t);
}

Junction::Type stringify<Junction::Type>::fromString(const string &s) {
    return str2junctionType.at(s);
}

string stringify<Junction::Type>::toString(const Junction::Type &t) {
    return junctionType2str.at(t);
}

TrafficLightLogic::Type stringify<TrafficLightLogic::Type>::fromString(const string &s) {
    return str2tlType.at(s);
}

string stringify<TrafficLightLogic::Type>::toString(const TrafficLightLogic::Type &t) {
    return tlType2str.at(t);
}

TrafficLightLogic::Phase::State stringify<TrafficLightLogic::Phase::State>::fromString(const string &s) {
    return str2tlState.at(s);
}

string stringify<TrafficLightLogic::Phase::State>::toString(const TrafficLightLogic::Phase::State &t) {
    return tlState2str.at(t);
}

vector<TrafficLightLogic::Phase::State>
stringify<vector<TrafficLightLogic::Phase::State>>::fromString(const string &s) {
    vector<TrafficLightLogic::Phase::State> ret;
    ret.reserve(s.size());
    for(const char &c: s) {
        ret.emplace_back(stringify<TrafficLightLogic::Phase::State>::fromString(string(1, c)));
    }
    return ret;
}

string stringify<vector<TrafficLightLogic::Phase::State>>::toString(const vector<TrafficLightLogic::Phase::State> &t) {
    char *arr = new char[t.size() + 1];
    for(size_t i = 0; i < t.size(); ++i) {
        string s = stringify<TrafficLightLogic::Phase::State>::toString(t[i]);
        if(s.size() != 1)
            throw logic_error("Stringification of tlLogic::Phase::State should always have only 1 char");
        arr[i] = s[0];
    }
    arr[t.size()] = '\0';
    return string(arr);
}

Connection::Direction stringify<Connection::Direction>::fromString(const string &s) {
    return str2connDir.at(s);
}

string stringify<Connection::Direction>::toString(const Connection::Direction &t) {
    return connDir2str.at(t);
}

Connection::State stringify<Connection::State>::fromString(const string &s) {
    return str2connState.at(s);
}

string stringify<Connection::State>::toString(const Connection::State &t) {
    return connState2str.at(t);
}
