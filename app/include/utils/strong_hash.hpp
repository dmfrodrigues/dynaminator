#pragma once

#include <cstddef>
#include <vector>

namespace utils {
template<class T>
struct strong_hash;

template<>
struct strong_hash<long> {
    std::size_t operator()(long v) const;
};

template<class T>
struct strong_hash<std::vector<T>> {
    std::size_t operator()(const std::vector<T> &v) const {
        // From:
        // - https://stackoverflow.com/a/72073933/12283316
        auto h = utils::strong_hash<T>();

        size_t seed = v.size();

        for(T x: v) {
            x = h(x);
            seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

}  // namespace utils
