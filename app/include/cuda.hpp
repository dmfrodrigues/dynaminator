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
    pair<U, V> &operator=(const std::pair<U, V> &p){
        first = p.first;
        second = p.second;
        return *this;
    }
};

template <class T>
class vector {
    size_t capacity;
    size_t sz = 0;
    T *arr = nullptr;

   public:
    vector(size_t cap) {
        capacity = cap;
        cudaErrchk(cudaMallocManaged(&arr, capacity * sizeof(T)));
    }

    vector(size_t cap, size_t s, T val = T()) : vector(cap) {
        sz = s;
        for(size_t i = 0; i < sz; ++i)
            arr[i] = val;
    }

    static vector<T> *constructShared(size_t cap){
        vector<T> *ret;
        cudaErrchk(cudaMallocManaged(&ret, sizeof(vector<T>)));
        return new (ret) vector<T>(cap);
    }
    static vector<T> *constructShared(size_t cap, size_t s, T val = T()){
        vector<T> *ret;
        cudaErrchk(cudaMallocManaged(&ret, sizeof(vector<T>)));
        return new (ret) vector<T>(cap, s, val);
    }

    void destroyShared(){
        this->~vector<T>();
        cudaErrchk(cudaFree(this));
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
    T &push_back(const T &val) {
        assert(sz + 1 <= capacity);
        return arr[sz++] = val;
    }

    __device__ __host__
    void pop_back() {
        assert(!empty());
        arr[--sz].~T();
    }

    template<class... Args>
    __device__ __host__
    T &emplace_back(Args&&... args) {
        assert(sz + 1 <= capacity);
        return *new (arr + sz++) T(args...);
    }

    __device__ __host__
    vector<T> &operator=(const vector<T> &v) = delete;

    ~vector(){
        while(!empty())
            arr[--sz].~T();
        cudaErrchk(cudaFree(arr));
    }
};
}  // namespace cuda
