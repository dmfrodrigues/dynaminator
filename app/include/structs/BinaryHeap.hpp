#pragma once

#include <vector>

#include "PriorityQueue.hpp"

template <class T>
class BinaryHeap : public PriorityQueue<T> {
   public:
    class Element : public PriorityQueue<T>::Element {
        friend BinaryHeap;

       private:
        BinaryHeap &binaryHeap;
        size_t index;
        T value;
        Element(BinaryHeap &heap, size_t i, T t)
            : binaryHeap(heap), index(i), value(t) {}

       public:
        virtual T getValue() { return value; }
        virtual void decreaseKey(T t) {
            value = t;
            binaryHeap.heapifyDown(index);
        }

       private:
        static void swap(Element *&e1, Element *&e2) {
            std::swap(e1->index, e2->index);
            std::swap(e1, e2);
        }
    };

   private:
    typedef std::vector<Element *> Container;

    Container container = Container(1, nullptr);

   public:
    ~BinaryHeap(){
        for(Element *e: container)
            delete e;
    }

    void reserve(size_t sz){
        container.reserve(sz);
    }

    virtual T top() {
        return container[1]->getValue();
    }

    virtual size_t size() {
        return container.size() - 1;
    }

    virtual Element &push(T t) {
        Element *it = new Element(*this, container.size(), t);
        container.push_back(it);

        heapifyDown(container.size() - 1);

        return *it;
    }

    virtual T pop() {
        T ret = container.at(1)->getValue();

        Element::swap(container[1], container[container.size() - 1]);
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

            Element::swap(container[i], container[smallest]);
            i = smallest;
        }
    }

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
