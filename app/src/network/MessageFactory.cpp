#include "network/MessageFactory.hpp"

#include <iostream>

using namespace std;

Message* MessageFactory::factoryMethod(stringstream &ss) const {
    Message::Operation op;
    ss >> utils::deserialize<Message::Operation>(op);

    Message *m = Message::generators.at(op)();

    m->deserializeContents(ss);
    return m;
}
