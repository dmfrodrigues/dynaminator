#pragma once

#include "network/Message.hpp"

class MessageFactory {
public:
    Message* factoryMethod(std::stringstream &ss) const;
};
