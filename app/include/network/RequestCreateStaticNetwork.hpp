#pragma once

#include "network/Message.hpp"

class RequestCreateStaticNetwork : public MessageRequest {
   public:

    virtual Operation getOperation() const;
    virtual void serializeContents(std::stringstream &ss) const;
    virtual bool deserializeContents(std::stringstream &ss);

    class Response : public MessageResponse {
    };
};
