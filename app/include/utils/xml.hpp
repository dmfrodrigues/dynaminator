#pragma once

#include <rapidxml.hpp>

#include "utils/stringify.hpp"

namespace utils::xml {
template<typename T>
void add_attribute(rapidxml::xml_node<> &xmlNode, const std::string &key, const T &val) {
    rapidxml::xml_document<> &doc  = *xmlNode.document();
    auto            attr = doc.allocate_attribute(
        doc.allocate_string(
            key.c_str()
        ),
        doc.allocate_string(
            utils::stringify::stringify<T>::toString(val).c_str()
        )
    );
    xmlNode.append_attribute(attr);
}
}
