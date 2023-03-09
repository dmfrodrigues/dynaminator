#pragma once

#include "network/Message.hpp"

class CreateBPRNetwork : public MessageRequest {
   private:
    
    std::string resourceId;
    std::string path;

   public:
    virtual Operation getOperation() const;
    virtual void serializeContents(std::stringstream &ss) const;
    virtual bool deserializeContents(std::stringstream &ss);

    class Response : public MessageResponse {
        virtual Operation getOperation() const;
        virtual void serializeContents(std::stringstream &ss) const;
        virtual bool deserializeContents(std::stringstream &ss);
    };

    virtual Response* process();
};
