#include "Dynamic/Env/Event/Event.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Event::Event(Time t_):
    t(t_) {}

const Time &Event::getTime() const {
    return t;
}

bool Event::operator<(const Event &event) const {
    return t < event.t;
}

bool Event::operator>(const Event &event) const {
    return event < *this;
}
