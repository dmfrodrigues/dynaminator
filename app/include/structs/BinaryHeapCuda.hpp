#pragma once

#include <vector>

#include "cuda.hpp"
#include "structs/PriorityQueueCuda.hpp"

template <class T>
class BinaryHeapCuda : public PriorityQueueCuda<T> {
   public:
    class Element : public PriorityQueueCuda<T>::Element {
        friend BinaryHeapCuda;

       private:
        BinaryHeapCuda &binaryHeap;
        size_t index;
        T value;
        __device__ __host__ Element(BinaryHeapCuda &heap, size_t i, T t)
            : binaryHeap(heap), index(i), value(t) {}

       public:
        __device__ __host__ virtual T getValue() { return value; }
        __device__ __host__ virtual void decreaseKey(T t) {
            value = t;
            binaryHeap.heapifyDown(index);
        }

       private:
        __device__ __host__ static void swap(Element *&e1, Element *&e2) {
            cuda::swap(e1->index, e2->index);
            cuda::swap(e1, e2);
        }
    };

   private:
    size_t arrSize = 0, sz = 1;
    Element** container = nullptr;

   public:
    __device__ __host__
    BinaryHeapCuda(size_t s){
        arrSize = s+1;
        container = new Element*[arrSize];
        for(size_t i = 0; i < arrSize; ++i)
            container[i] = nullptr;
    }

    __device__ __host__
    ~BinaryHeapCuda() {
        for (size_t i = 0; i < arrSize; ++i)
            delete container[i];
        delete container;
    }

    __device__ __host__
    virtual T top() {
        return container[1]->getValue();
    }

    __device__ __host__
    virtual size_t size() {
        return sz - 1;
    }

    __device__ __host__
    virtual Element &push(T t) {
        Element *it = new Element(*this, sz, t);
        container[sz++] = it;

        heapifyDown(sz - 1);

        return *it;
    }

    __device__ __host__
    virtual T pop() {
        T ret = container[1]->getValue();

        Element::swap(container[1], container[sz - 1]);
        delete container[sz-1];
        container[sz-1] = nullptr;
        --sz;

        heapifyUp(1);

        return ret;
    }

   private:
    __device__ __host__
    void heapifyUp(size_t i) {
        while (true) {
            size_t l = i << 1;
            size_t r = l | 1;
            size_t smallest = i;

            if (l < sz && container[l]->getValue() < container[smallest]->getValue()) {
                smallest = l;
            }

            if (r < sz && container[r]->getValue() < container[smallest]->getValue()) {
                smallest = r;
            }

            if (smallest == i) break;

            Element::swap(container[i], container[smallest]);
            i = smallest;
        }
    }

    __device__ __host__
    void heapifyDown(size_t i) {
        while (i > 1) {
            size_t p = i >> 1;
            if (container[i]->getValue() < container[p]->getValue()) {
                Element::swap(container[i], container[p]);
            } else
                break;
            i = p;
        }
    }
};
