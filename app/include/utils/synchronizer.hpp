#pragma once

#include <mutex>

namespace utils {
template<class T>
class synchronizer {
    std::mutex m;
    T t;

   public:
    T &operator*(){
        return t;
    }

    T *operator->(){
        return &t;
    }

    std::mutex &mutex(){
        return m;
    }
};
}  // namespace utils
