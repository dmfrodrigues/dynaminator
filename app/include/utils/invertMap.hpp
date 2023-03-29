#pragma once

#include <unordered_map>

namespace utils {
template<class K, class V>
std::unordered_map<V, K> invertMap(const std::unordered_map<K, V> &m){
    std::unordered_map<V, K> ret;
    for(const auto &p: m)
        ret[p.second] = p.first;
    return ret;
}
}
