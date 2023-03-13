#pragma once

template<class T>
class PriorityQueue {
public:
    class Element {
    public:
        virtual T getValue() = 0;
        virtual void decreaseKey(T t) = 0;
        virtual ~Element(){};
    };

    virtual T top() = 0;
    virtual size_t size() = 0;
    bool empty(){ return size() == 0; }
    virtual Element& push(T t) = 0;
    virtual T pop() = 0;
};
