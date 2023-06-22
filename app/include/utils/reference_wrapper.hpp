#pragma once

#include <functional>

namespace utils::reference_wrapper {

template<typename T>
struct hash {
    size_t operator()(const std::reference_wrapper<T> &t) const {
        return std::hash<T>()(t.get());
    }
};

template<typename T>
struct equal_to {
    bool operator()(const std::reference_wrapper<T> &a, const std::reference_wrapper<T> &b) const {
        return a.get() == b.get();
    }
};

template<typename T>
struct less {
    bool operator()(const std::reference_wrapper<T> &a, const std::reference_wrapper<T> &b) const {
        return a.get() < b.get();
    }
};

}  // namespace utils::reference_wrapper
