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
    typedef cuda::vector<Element *> Container;

    Container container;

   public:
    BinaryHeapCuda(size_t s) : container(s) {
        container.push_back(nullptr);
    }

    __device__ __host__ virtual T top() {
        return container[1]->getValue();
    }

    __device__ __host__ virtual size_t size() {
        return container.size() - 1;
    }

    __device__ __host__ virtual Element &push(T t) {
        Element *it = new Element(*this, container.size(), t);
        container.push_back(it);

        heapifyDown(container.size() - 1);

        return *it;
    }

    __device__ __host__ virtual T pop() {
        T ret = container.at(1)->getValue();

        Element::swap(container[1], container[container.size() - 1]);
        delete container[container.size() - 1];
        container.pop_back();

        heapifyUp(1);

        return ret;
    }

   private:
    __device__ __host__ void heapifyUp(size_t i) {
        while (true) {
            size_t l = i << 1;
            size_t r = l | 1;
            size_t smallest = i;

            if (l < container.size() && container[l]->getValue() < container[smallest]->getValue()) {
                smallest = l;
            }

            if (r < container.size() && container[r]->getValue() < container[smallest]->getValue()) {
                smallest = r;
            }

            if (smallest == i) break;

            Element::swap(container[i], container[smallest]);
            i = smallest;
        }
    }

    __device__ __host__ void heapifyDown(size_t i) {
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
