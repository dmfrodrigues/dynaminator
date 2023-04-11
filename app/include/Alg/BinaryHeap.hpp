#pragma once

#include <vector>

#include "Alg/PriorityQueue.hpp"

namespace Alg {
template <class T>
/**
 * @brief Binary heap.
 */
class BinaryHeap : public PriorityQueue<T> {
    class BinHeapElement : public PriorityQueue<T>::Element {
        friend BinaryHeap;

       private:
        BinaryHeap &binaryHeap;
        size_t index;
        T value;
        BinHeapElement(BinaryHeap &heap, size_t i, T t)
            : binaryHeap(heap), index(i), value(t) {}

       public:
        virtual T getValue() { return value; }
        virtual void decreaseKey(T t) {
            value = t;
            binaryHeap.heapifyDown(index);
        }

       private:
        static void swap(BinHeapElement *&e1, BinHeapElement *&e2) {
            std::swap(e1->index, e2->index);
            std::swap(e1, e2);
        }
    };

    typedef std::vector<BinHeapElement *> Container;

    Container container = Container(1, nullptr);

   public:
    ~BinaryHeap(){
        for(BinHeapElement *e: container)
            delete e;
    }

    /**
     * @brief Reserve space in the queue.
     *
     * Can make subsequent calls to push(T) faster. See the documentation of
     * e.g. std::vector::reserve(size_t) for more details on the way this
     * function works.
     *
     * @param sz Capacity of the queue
     */
    void reserve(size_t sz){
        container.reserve(sz);
    }

    /**
     * @brief Get top element
     *
     * Complexity is \f$O(1)\f$.
     */
    virtual T top() {
        return container[1]->getValue();
    }

    virtual size_t size() {
        return container.size() - 1;
    }

    /**
     * @brief Push new element into queue
     *
     * Complexity is \f$O(\log N)\f$.
     */
    virtual typename PriorityQueue<T>::Element &push(T t) {
        BinHeapElement *it = new BinHeapElement(*this, container.size(), t);
        container.push_back(it);

        heapifyDown(container.size() - 1);

        return *it;
    }

    /**
     * @brief Pop smallest element of the queue
     *
     * Complexity is \f$O(\log N)\f$.
     */
    virtual T pop() {
        T ret = container.at(1)->getValue();

        BinHeapElement::swap(container[1], container[container.size() - 1]);
        delete container[container.size() - 1];
        container.pop_back();

        heapifyUp(1);

        return ret;
    }

   private:
    void heapifyUp(size_t i) {
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

            BinHeapElement::swap(container[i], container[smallest]);
            i = smallest;
        }
    }

    void heapifyDown(size_t i) {
        while (i > 1) {
            size_t p = i >> 1;
            if (container[i]->getValue() < container[p]->getValue()) {
                BinHeapElement::swap(container[i], container[p]);
            } else
                break;
            i = p;
        }
    }
};
}
