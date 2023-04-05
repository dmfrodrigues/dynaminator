#pragma once

template<class T>
class PriorityQueueCuda {
public:
    class Element {
    public:
        __device__ __host__ virtual T getValue() = 0;
        __device__ __host__ virtual void decreaseKey(T t) = 0;
        __device__ __host__ virtual ~Element(){};
    };

    __device__ __host__ virtual T top() = 0;
    __device__ __host__ virtual size_t size() = 0;
    __device__ __host__ bool empty(){ return size() == 0; }
    __device__ __host__ virtual Element& push(T t) = 0;
    __device__ __host__ virtual T pop() = 0;
};
