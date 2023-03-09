#include "GlobalState.hpp"

GlobalState::~GlobalState(){
    for(const auto &p: staticNetworks){
        delete p.second;
    }
}
