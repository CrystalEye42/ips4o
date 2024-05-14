/******************************************************************************
 * src/example.cpp
 *
 * In-Place Parallel Super Scalar Samplesort (IPS⁴o)
 *
 ******************************************************************************
 * BSD 2-Clause License
 *
 * Copyright © 2020, Michael Axtmann <michael.axtmann@gmail.com>
 * Copyright © 2020, Sascha Witt <sascha.witt@kit.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <algorithm>
#include <atomic>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <unordered_set>
#include <chrono>

#include "ips4o.hpp"

size_t n = 10000000;
size_t kNumTests = 4;
constexpr int NUM_ROUNDS = 5;

template<class T>
void check_correctness(const std::vector<T> in) {
    std::unordered_set<T> s{};
    T prev = in[0];
    s.insert(prev);
    for (auto &e : in) {
        if (e != prev) {
            if (s.find(e) != s.end()) {
                fprintf(stderr, "Error: not semisorted\n");
                std::cout << "\nfound duplicate of " << e << std::endl;
                exit(EXIT_FAILURE);
            } else {
                s.insert(e);
            }
            prev = e;
        }
    }
    printf("Pass\n");
}

template<class T>
double test(const std::vector<T> in) {
    std::cout << "test_name: semisort" << std::endl;
    double total_time = 0;
    for (int i = 0; i <= NUM_ROUNDS; i++) {
        auto last = std::chrono::system_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
        if (i == 0) {
        printf("Warmup round: %f\n", diff);
        } else {
        printf("Round %d: %f\n", i, diff);
        total_time += diff;
        }
    }
    double avg = total_time / NUM_ROUNDS;
    printf("Average: %f\n", avg);
    return avg;
}

template<class T>
void run_all(const std::vector<T> &seq) {
  // get_distribution(seq);
    test(seq);
    //check_correctness(seq);
    printf("\n");
}

template<class T>
std::vector<T> uniform_pairs_generator(size_t num_keys) {
    std::random_device r;
    std::default_random_engine gen(r());
    std::uniform_real_distribution<double> dist;
    printf("uniform distribution with num_keys: %zu\n", num_keys);
    std::vector<T> seq(n);
    for(auto& e : seq) {
        e = T(dist(gen) * num_keys);
    }
    return seq;
}

template<class T>
std::vector<T> exponential_pairs_generator(double lambda) {
    std::random_device r;
    std::default_random_engine gen(r());
    std::exponential_distribution<double> dist(lambda);
    printf("exponential distribution with lambda: %.10f\n", lambda);
    std::vector<T> seq(n);
    for(auto& e : seq) {
        e = T(std::max(1.0, dist(gen) * n));
    }
    return seq;
}

template<class T>
void run_all_dist() {
    // uniform distribution
    std::vector<size_t> num_keys{100000, 1000, 10};
    for (auto v : num_keys) {
        auto seq = uniform_pairs_generator<T>(v);
        std::cout << "\ntest_name: semisort" << std::endl;
        double total_time = 0;
        for (int i = 0; i <= NUM_ROUNDS; i++) {
            std::cout << "\ngenerating sequence..." << std::endl;
            auto seq = uniform_pairs_generator<T>(v);
            std::cout << "sorting..." << std::endl;
            auto last = std::chrono::system_clock::now();
            ips4o::parallel::sort(seq.begin(), seq.end(), std::less<>{});
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
            if (i == 0) {
            printf("Warmup round: %f\n", diff);
            } else {
            printf("Round %d: %f\n", i, diff);
            total_time += diff;
            check_correctness(seq);
            }
        }
        double avg = total_time / NUM_ROUNDS;
        printf("Average: %f\n", avg);
    }

    // exponential distribution
    std::vector<double> lambda{0.00001, 0.00002, 0.00005, 0.00007, 0.0001};
    for (auto v : lambda) {
        auto seq = exponential_pairs_generator<T>(v);
        std::cout << "\ntest_name: semisort" << std::endl;
        double total_time = 0;
        for (int i = 0; i <= NUM_ROUNDS; i++) {
            auto last = std::chrono::system_clock::now();
            ips4o::parallel::sort(seq.begin(), seq.end(), std::less<>{});
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
            if (i == 0) {
            printf("Warmup round: %f\n", diff);
            } else {
            printf("Round %d: %f\n", i, diff);
            total_time += diff;
            }
        }
        double avg = total_time / NUM_ROUNDS;
        printf("Average: %f\n", avg);
        check_correctness(seq);
    }
}

int main(int argc, char** argv) {
    run_all_dist<int32_t>();
    // run_all_dist<uint64_t>();
//     std::random_device r;
//     std::default_random_engine gen(r());
//     std::uniform_real_distribution<double> dist;

//     std::vector<int> v(100);
//     for (auto& e : v) {
//         e = int(dist(gen) * 100);
//     }

// #if defined(_REENTRANT)
//     ips4o::parallel::sort(v.begin(), v.end(), std::less<>{});
// #else
//     ips4o::sort(v.begin(), v.end(), std::less<>{});
// #endif
//     for (auto e : v) {
//         std::cout << e << " ";
//     }
//     std::cout << std::endl;
//     //const bool sorted = std::is_sorted(v.begin(), v.end(), std::less<>{});
//     //std::cout << "Elements are sorted: " << std::boolalpha << sorted << std::endl;

//     return 0;
}
