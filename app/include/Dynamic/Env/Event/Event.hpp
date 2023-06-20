#pragma once

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {

class Env;

class Event {
    friend Env;

    Time t;

   protected:
    const Time &getTime() const;

   public:
    Event(Time t);
    virtual void process(Env &env) = 0;

    bool operator<(const Event &event) const;
    bool operator>(const Event &event) const;
};
}  // namespace Dynamic::Env
