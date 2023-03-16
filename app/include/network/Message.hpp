#pragma once

#include <functional>
#include <sstream>
#include <unordered_map>

#include "utils/serialize.hpp"

class MessageRequest;
class MessageResponse;

class MessageFactory;

class Message {
    friend MessageFactory;

    friend MessageRequest;
    friend MessageResponse;

   public:
    typedef std::string Operation;

   protected:
    static std::unordered_map<Operation, std::function<Message *()>> generators;

   public:
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
    std::stringstream serialize() const;

    static void registerOperation(
        const Operation &operation,
        std::function<Message *()> generator);

    virtual ~Message() {}
};

namespace std {
ostream &operator<<(ostream &os, const utils::serialize<Message::Type> &s);
istream &operator>>(istream &os, utils::deserialize<Message::Type> s);
}  // namespace std

namespace utils {
template <>
class serialize<Message::Type> {
    const Message::Type &t;

   public:
    serialize(const Message::Type &obj);
    friend std::ostream &std::operator<<(
        std::ostream &os, const serialize<Message::Type> &s);
};

template <>
class deserialize<Message::Type> {
    Message::Type &t;

   public:
    deserialize(Message::Type &obj);
    friend std::istream &std::operator>>(
        std::istream &is,
        deserialize<Message::Type> s);
};
}  // namespace utils

class MessageRequest : public Message {
   private:
    virtual Type getType() const;

   public:
    virtual MessageResponse *process() = 0;
};

class MessageResponse : public Message {
   private:
    bool success = true;

    virtual Type getType() const;

   public:
    virtual void setSuccess(bool s);
    virtual bool getSuccess() const;
    virtual void handle(std::ostream &is) = 0;
};

#define MESSAGE_REGISTER_MAIN(MessageSubclass)          \
    Message::registerOperation(#MessageSubclass, []() { \
        return new MessageSubclass();                   \
    })

#define MESSAGE_REGISTER_DEF(MessageSubclass)                  \
    Message::Operation MessageSubclass::getOperation() const { \
        return #MessageSubclass;                               \
    }
