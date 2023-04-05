#pragma once

namespace cuda {
template <class T>
__device__ __host__
void swap(T &a, T &b){
  T c(a); a=b; b=c;
};

template <class U, class V>
struct pair {
    U first;
    V second;
    __device__ __host__
    bool operator<(const pair<U,V> &p) const{
        if(first != p.first) return first < p.first;
        else return second < p.second;
    }
};
}
