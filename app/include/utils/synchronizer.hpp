#pragma once

#include <mutex>

namespace utils {
template<class T>
class synchronizer: public std::mutex {
    T t;

   public:
    T &operator*() {
        return t;
    }

    T *operator->() {
        return &t;
    }
};
}  // namespace utils
