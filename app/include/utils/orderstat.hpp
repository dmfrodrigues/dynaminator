#pragma once

#include <deque>
#include <ext/pb_ds/assoc_container.hpp>  // Common file
#include <ext/pb_ds/tree_policy.hpp>      // tree_order_statistics_node_update
#include <stdexcept>

/**
 * @brief Order-statistics trees.
 *
 * These data structures behave similarly to sets and maps, but they also
 * provide order statistics; i.e., you can ask for the k-th element in the set,
 * or the number of elements in the set less than x.
 *
 * The vast majority of logic required to implement order-statistics trees is
 * already hidden in a few GNU G++ headers, so we are just wrapping them here
 * into more convenient, STL-like abstractions.
 */
namespace utils::orderstat {

namespace gnu = __gnu_pbds;

/// @brief Internal definitions
namespace internal {

template<class Key, class T, class CMP>
using orderstat = gnu::tree<Key, T, CMP, gnu::rb_tree_tag, gnu::tree_order_statistics_node_update>;

template<class Key>
using orderstat_multiset = orderstat<Key, gnu::null_type, std::less_equal<Key> >;

}  // namespace internal

/// Useful definitions

/**
 * @brief Set with order statistics.
 */
template<class Key>
using set = internal::orderstat<Key, gnu::null_type, std::less<Key> >;

/**
 * @brief Multiset with order statistics.
 */
template<class Key>
struct multiset: public internal::orderstat_multiset<Key> {
    size_t erase(const Key& val) {
        auto   it  = this->lower_bound(val - 1);
        size_t ret = 0;
        while(*it == val) {
            internal::orderstat_multiset<Key>::erase(it);
            ++ret;
            it = this->lower_bound(val - 1);
        }
        return ret;
    }
};

/**
 * @brief Map with order statistics.
 */
template<class Key, class T>
using map = internal::orderstat<Key, T, std::less<Key> >;

/**
 * @brief Multimap with order statistics.
 */
template<class Key, class T>
using multimap = internal::orderstat<Key, T, std::less_equal<Key> >;

// clang-format off
template<
    class T,
    class Compare = std::less<T>
>
// clang-format on
class queue {
    size_t        start = 0;
    std::deque<T> data;

    std::map<T, size_t, Compare> dataToCounter;

   public:
    void push(const T& val) {
        size_t i = start + data.size();

        if(dataToCounter.count(val))
            throw std::runtime_error("utils::orderstat::queue: duplicate value");

        dataToCounter[val] = i;

        data.emplace_back(val);
    }

    template<class... Args>
    void emplace(Args&&... args) {
        push(T(args...));
    }

    void pop() {
        assert(dataToCounter.erase(front()) == 1);
        data.pop_front();
        if(!data.empty())
            start = dataToCounter.at(data.front());
        else {
            assert(dataToCounter.empty());
            start = 0;
        }
    }

    T& at(size_t i) {
        return data.at(i);
    }
    const T& at(size_t i) const {
        return data.at(i);
    }

    T&       operator[](size_t i) { return data[i]; }
    const T& operator[](size_t i) const { return data[i]; }

    size_t size() const { return data.size(); }

    bool empty() const { return size() == 0; }

    T&       front() { return at(0); }
    const T& front() const { return at(0); }

    size_t order_of(const T& val) const {
        size_t i;
        try {
            i = dataToCounter.at(val);
        } catch(const std::out_of_range& e) {
            throw std::out_of_range("utils::orderstat::queue: value not found");
        }
        return i - start;
    }

    // int erase(const T& val) {
    //     size_t i;
    //     try {
    //         i = order_of(val);
    //     } catch(const std::out_of_range& e) {
    //         return 0;
    //     }

    //     if(i == 0) {
    //         pop();
    //         return 1;
    //     }

    //     size_t pos = start + i;

    //     data.erase(data.begin() + i);

    //     assert(dataToCounter.erase(val) == 1);

    //     for(auto& [k, v]: dataToCounter)
    //         if(v > pos)
    //             --v;

    //     return 1;
    // }
};

}  // namespace utils::orderstat
