#pragma once

#include <cstddef>

namespace Alg {
/**
 * @brief Minimum priority queue.
 *
 * Unlike std::priority_queue, PriorityQueue exposes a decreaseKey operation.
 * You can use it by storing the Element& objects returned when calling push(T),
 * and then calling Element::decreaseKey(T).
 *
 * @tparam T Type of elements being stored in queue
 */
template<class T>
class PriorityQueue {
public:
    /**
     * @brief Priority queue element. Allows to decrease key
     */
    class Element {
    public:
        /// @brief Get element value
        virtual T getValue() = 0;
        /// @brief Decrease value of element
        virtual void decreaseKey(T t) = 0;
        virtual ~Element(){};
    };

    /// @brief Get top element
    virtual T top() = 0;
    /// @brief Get queue size
    virtual size_t size() = 0;
    /// @brief Test if queue is empty
    bool empty(){ return size() == 0; }
    /// @brief Push new element into queue
    virtual Element& push(T t) = 0;
    /// @brief Pop smallest element of the queue
    virtual T pop() = 0;
};
}
