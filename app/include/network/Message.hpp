#pragma once

// class MessageFactory;

#include <functional>
#include <sstream>
#include <unordered_map>

class MessageRequest;
class MessageResponse;

class Message {
    friend MessageRequest;
    friend MessageResponse;

    // friend MessageFactory;

   public:
    typedef std::string Operation;

   protected:
    static std::unordered_map<Operation, std::function<Message *()>> generators;

    enum Type : uint8_t {
        REQUEST = 1,
        RESPONSE = 2
    };

    virtual Type getType() const = 0;
    virtual Operation getOperation() const = 0;

   protected:
    virtual void serializeContents(std::stringstream &ss) const = 0;
    virtual bool deserializeContents(std::stringstream &ss) = 0;

   public:
    std::string serialize() const;

    static void registerOperation(
        const Operation &operation,
        std::function<Message *()> generator);
};

class MessageRequest : public Message {
   private:
    virtual Type getType() const;
};

class MessageResponse : public Message {
   private:
    virtual Type getType() const;
};

#define MESSAGE_REGISTER_MAIN(MessageSubclass)          \
    Message::registerOperation(#MessageSubclass, []() { \
        return new MessageSubclass();                   \
    })

#define MESSAGE_REGISTER_DEF(MessageSubclass)                  \
    Message::Operation MessageSubclass::getOperation() const { \
        return #MessageSubclass;                               \
    }
