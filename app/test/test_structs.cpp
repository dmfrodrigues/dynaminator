#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <queue>
#include <random>

#include "Alg/BinaryHeap.hpp"

using namespace std;

template<class T>
void dump_queue(Alg::PriorityQueue<T> &q, const vector<T> &v) {
    REQUIRE(q.size() == v.size());

    for(size_t i = 0; i < v.size(); ++i) {
        REQUIRE(q.pop() == v[i]);
        REQUIRE(q.size() == v.size() - i - 1);
    }
}

TEST_CASE("Binary heap - small tests", "[binary-heap]") {
    Alg::PriorityQueue<int> *q = new Alg::BinaryHeap<int>();

    SECTION("Pop empty queue gives error") {
        REQUIRE_THROWS(q->pop());
    }

    SECTION("Insert") {
        q->push(3);
        q->push(2);
        q->push(1);

        REQUIRE(q->size() == 3);
    }

    SECTION("Pop") {
        q->push(9);
        q->push(5);
        q->push(1);

        dump_queue(*q, {1, 5, 9});
    }

    SECTION("Decrease key") {
        Alg::PriorityQueue<int>::Element &el = q->push(9);
        q->push(5);
        q->push(1);

        el.decreaseKey(3);

        dump_queue(*q, {1, 3, 5});
    }

    delete q;
}

TEST_CASE("Binary heap - stress test", "[binary-heap]") {
    std::mt19937 gen(0);

    std::uniform_real_distribution<float> frand(0.0f, 1.0f);
    std::uniform_int_distribution<int>    rand(0);

    Alg::PriorityQueue<int> *q = new Alg::BinaryHeap<int>();

    size_t NUMBER_OPERATIONS = 100;
    float  PROB_INSERT       = 0.6f;
    float  PROB_POP          = 0.3f;
    // float PROB_DECREASE_KEY = 0.1f;
    float PROB_TOTAL = PROB_INSERT + PROB_POP;

    priority_queue<int, vector<int>, greater<int>> s;

    for(size_t i = 0; i < NUMBER_OPERATIONS; ++i) {
        float p = frand(gen) * PROB_TOTAL;
        if(p <= PROB_INSERT) {
            int r = rand(gen);
            q->push(r);
            s.push(r);
        } else if(p < PROB_INSERT + PROB_POP && s.size() > 0) {
            int r1 = q->pop();
            REQUIRE(r1 == s.top());
            s.pop();
        }

        REQUIRE(q->size() == s.size());
    }

    while(!s.empty()) {
        REQUIRE(q->pop() == s.top());
        s.pop();
    }

    delete q;
}
