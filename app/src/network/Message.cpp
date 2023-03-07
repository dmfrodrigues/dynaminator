#include "network/Message.hpp"

#include "utils/serialize.hpp"

using namespace std;

typedef Message::Type Type;

std::unordered_map<
    Message::Operation,
    std::function<Message *()> >
    Message::generators;

void Message::registerOperation(
    const Operation &operation,
    std::function<Message *()> generator) {
    generators[operation] = generator;
}

string Message::serialize() const {
    stringstream ss;

    ss << utils::serialize<Operation>(getOperation());

    serializeContents(ss);
    return ss.str();
}

utils::serialize<Type>::serialize(const Type &obj):t(obj){}
ostream &std::operator<<(ostream &os, const utils::serialize<Type> &s){
    os.write(reinterpret_cast<const char*>(&s.t), sizeof(s.t));
    return os;
}

utils::deserialize<Type>::deserialize(Type &obj): t(obj){}
istream &std::operator>>(istream &is, utils::deserialize<Type> s) {
    is.read(reinterpret_cast<char*>(&s.t), sizeof(s.t));
    return is;
}

MessageRequest::Type MessageRequest::getType() const {
    return REQUEST;
}

MessageResponse::Type MessageResponse::getType() const {
    return RESPONSE;
}
