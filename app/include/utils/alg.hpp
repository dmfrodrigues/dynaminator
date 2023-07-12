#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <list>
#include <set>
#include <stdexcept>

namespace utils::alg {

template<class T>
void topological_sort(std::vector<T> &v) {
    return topological_sort(v, std::less<T>());
}

template<class T, class Compare>
void topological_sort(std::vector<T> &v, Compare comp) {
    const size_t N = v.size();

    std::vector<ssize_t> inDegree(N, 0);

    std::vector<T> L;
    L.reserve(N);

    std::set<size_t> S;

    for(size_t i = 0; i < N; ++i) {
        if(inDegree[i] == 0) {
            S.insert(i);
        }
    }

    while(!S.empty()) {
        size_t n = *S.begin();
        S.erase(S.begin());
        L.emplace_back(v[n]);

        for(size_t m = 0; m < N; ++m) {
            if(inDegree[m] > 0) {
                if(comp(v[n], v[m])) {
                    --inDegree[m];
                    if(inDegree[m] == 0) {
                        S.insert(m);
                    }
                }
            }
        }
    }

    if(L.size() != N)
        throw std::invalid_argument("Graph has a cycle");

    v = L;
}

}  // namespace utils::alg
