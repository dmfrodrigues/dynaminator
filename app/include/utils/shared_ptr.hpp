#pragma once

#include <memory>

namespace utils::shared_ptr {

template<typename T>
struct greater {
    bool operator()(const std::shared_ptr<T> &a, const std::shared_ptr<T> &b) const {
        return *a > *b;
    }
};

}  // namespace utils::shared_ptr
