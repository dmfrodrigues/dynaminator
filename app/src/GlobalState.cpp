#include "GlobalState.hpp"

using namespace std;

unordered_map<GlobalState::ResourceId, pair<StaticNetwork*, SumoAdapterStatic>> GlobalState::staticNetworks;
unordered_map<GlobalState::ResourceId, StaticDemand> GlobalState::staticDemands;
