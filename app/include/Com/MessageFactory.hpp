#pragma once

#include "Com/Message.hpp"

namespace Com {
class MessageFactory {
   public:
    Message* factoryMethod(std::stringstream& ss) const;
};
}  // namespace Com
