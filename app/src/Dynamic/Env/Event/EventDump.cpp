#include "Dynamic/Env/Event/EventDump.hpp"

#include "data/SUMO/NetState.hpp"

using namespace std;
using namespace Dynamic::Env;

EventDump::EventDump(Time t, SUMO::NetState &netState_, const Dynamic::SUMOAdapter &adapter_, bool closeAfterDump_):
    Event(t), netState(netState_), adapter(adapter_), closeAfterDump(closeAfterDump_) {}

void EventDump::process(Env &env) {
    SUMO::NetState::Timestep::Loader<
        Dynamic::Env::Env &,
        const Dynamic::SUMOAdapter &,
        Dynamic::Time>
        timestepLoader;

    SUMO::NetState::Timestep timestep = timestepLoader.load(env, adapter, env.getTime());

    netState << timestep;

    if(closeAfterDump) {
        netState.close();
    }
}
