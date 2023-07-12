#include <catch2/catch_test_macros.hpp>

#include "utils/alg.hpp"

using namespace std;

TEST_CASE("Topological sort", "[topological-sort]") {
    SECTION("Test 1") {
        vector<int> v = {2, 9, 5, 4, 1, 8, 0, 6, 3, 7};

        utils::alg::topological_sort(
            v,
            [](const int &lhs, const int &rhs) -> bool { return lhs < rhs; }
        );

        REQUIRE(vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}) == v);
    }

    SECTION("Test 2") {
        vector<int>      v   = {0, 1, 2, 3, 4, 5};
        vector<set<int>> adj = {
            {},
            {},
            {3},
            {1},
            {0, 1},
            {0, 2}};

        utils::alg::topological_sort(
            v,
            [&adj](const int &lhs, const int &rhs) -> bool {
                return adj.at(lhs).count(rhs);
            }
        );

        REQUIRE(vector<int>({4, 5, 0, 2, 3, 1}) == v);
    }

    SECTION("Test 3") {
        vector<int>      v   = {0, 1, 2, 3, 4, 5};
        vector<set<int>> adj = {
            {1, 3},
            {2},
            {},
            {1, 4, 5},
            {5},
            {}};

        utils::alg::topological_sort(
            v,
            [&adj](const int &lhs, const int &rhs) -> bool {
                return adj.at(lhs).count(rhs);
            }
        );

        REQUIRE(vector<int>({0, 3, 1, 2, 4, 5}) == v);
    }
}
