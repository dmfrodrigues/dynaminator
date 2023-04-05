#pragma once

#include <exception>

#define cudaErrchk(ans) \
    { cuda::Errchk((ans), __FILE__, __LINE__); }

namespace cuda {

inline void Errchk(cudaError_t code, const char *file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        std::cerr
            << "GPUassert: " << cudaGetErrorString(code)
            << " (" << file << ":" << line << ")"
            << std::endl;
        if (abort) exit(code);
    }
}

template <class T>
__device__ __host__ void swap(T &a, T &b) {
    T c(a);
    a = b;
    b = c;
};

template <class U, class V>
struct pair {
    U first;
    V second;
    __device__ __host__ bool operator<(const pair<U, V> &p) const {
        if (first != p.first)
            return first < p.first;
        else
            return second < p.second;
    }
};

template <class T>
class vector {
    size_t capacity;
    size_t sz = 0;
    T *arr = nullptr;

   public:
    vector(size_t cap, size_t s = 0, T val = T()) {
        capacity = cap;
        cudaErrchk(cudaMallocManaged(&arr, capacity * sizeof(T)));
        sz = s;
        for(size_t i = 0; i < sz; ++i)
            arr[i] = val;
    }

    __device__ __host__
    T &operator[](size_t i) {
        return arr[i];
    }

    __device__ __host__
    const T &operator[](size_t i) const {
        return arr[i];
    }

    __device__ __host__
    T &at(size_t i) {
        assert(i < sz);
        return arr[i];
    }

    __device__ __host__
    const T &at(size_t i) const {
        assert(i < sz);
        return arr[i];
    }

    __device__ __host__
    size_t size() const {
        return sz;
    }

    __device__ __host__
    bool empty() const {
        return sz == 0;
    }

    __device__ __host__
    void push_back(const T &val) {
        assert(sz + 1 <= capacity);
        arr[sz++] = val;
    }

    __device__ __host__
    void pop_back() {
        assert(!empty());
        --sz;
    }
};
}  // namespace cuda
