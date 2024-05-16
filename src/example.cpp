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

std::vector<size_t> n_sizes{100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
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
std::vector<T> uniform_pairs_generator(size_t num_keys, size_t n) {
    printf("uniform distribution with num_keys: %zu, n: %zu\n", num_keys, n);
    std::vector<T> seq(n);
    for (int i=0; i < n; i++) {
        seq[i] = i % num_keys;
    }
    return seq;
}

template<class T>
std::vector<T> exponential_pairs_generator(double lambda, size_t n) {
    printf("exponential distribution with lambda: %.10f, n: %zu\n", lambda, n);
    std::vector<T> seq(n);
    double ratio = n > 100000 ? 100000/(n * lambda) : n/(n * lambda);
    for(int i=0; i < n; i++) {
        seq[i] = std::max(1.0, ratio * n * (lambda * exp(-lambda * (i + 0.5))));
    }
    return seq;
}

template<class T>
std::vector<T> zipfian_pairs_generator(double s, size_t n) {
    printf("zipfian distribution with s: %.10f, n: %zu\n", s, n);
    std::vector<T> seq(n);
    double total = 0;
    for(int i=0; i < n; i++) {
        total += 1.0 / pow(i + 1, s);
    }
    double v = total/n;
    double harmonic = 1.0;
    int harmonicidx = 2;
    double tmptotal = 0;
    double ratio = n > 100000 ? n/100000 : 1;
    for(int i=0; i < n; i++) {
        seq[i] = (harmonicidx + (ratio - 2))/ratio;
        tmptotal += v;
        while (tmptotal > harmonic) {
            harmonic += 1.0/pow(harmonicidx, s);
            harmonicidx += 1;
        }
    }
    return seq;
}

template<class T>
void run_all_dist() {
    std::random_device r;
    std::default_random_engine gen(r());
    // uniform distribution
    for (auto n: n_sizes) {
        std::vector<size_t> num_keys{100000, 1000, 10};
        std::cout << "test_name: uniform, n: " << n << std::endl;
        for (auto v : num_keys) {
            double total_time = 0;
            auto seq = uniform_pairs_generator<T>(v, n);
            for (int i = 0; i <= NUM_ROUNDS; i++) {
                std::shuffle(seq.begin(), seq.end(), gen);
                auto last = std::chrono::system_clock::now();
                ips4o::parallel::sort(seq.begin(), seq.end(), std::less<>{});
                auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
                if (i == 0) {
                    printf("Warmup round: %f ", diff);
                } else {
                    printf("Round %d: %f ", i, diff);
                    total_time += diff;
                    check_correctness(seq);
                }
            }
            double avg = total_time / NUM_ROUNDS;
            printf("Average: %f\n", avg);
        }

        // exponential distribution
        std::vector<double> lambda{10.0/n, 20.0/n, 50.0/n, 70.0/n, 100.0/n};
        std::cout << "test_name: exponential, n: " << n << std::endl;
        for (auto v : lambda) {
            double total_time = 0;
            auto seq = exponential_pairs_generator<T>(v, n);
            for (int i = 0; i <= NUM_ROUNDS; i++) {
                std::shuffle(seq.begin(), seq.end(), gen);
                auto last = std::chrono::system_clock::now();
                ips4o::parallel::sort(seq.begin(), seq.end(), std::less<>{});
                auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
                if (i == 0) {
                    printf("Warmup round: %f ", diff);
                } else {
                    printf("Round %d: %f ", i, diff);
                    total_time += diff;
                    check_correctness(seq);
                }
            }
            double avg = total_time / NUM_ROUNDS;
            printf("Average: %f\n", avg);
        }

        std::vector<double> s{0.6, 0.8, 1, 1.2, 1.5};
        std::cout << "test_name: zipfian, n: " << n << std::endl;
        for (auto v : s) {
            double total_time = 0;
            auto seq = zipfian_pairs_generator<T>(v, n);
            for (int i = 0; i <= NUM_ROUNDS; i++) {
                std::shuffle(seq.begin(), seq.end(), gen);
                auto last = std::chrono::system_clock::now();
                ips4o::parallel::sort(seq.begin(), seq.end(), std::less<>{});
                auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - last).count() / 1000000.0;
                if (i == 0) {
                    printf("Warmup round: %f ", diff);
                } else {
                    printf("Round %d: %f ", i, diff);
                    total_time += diff;
                    check_correctness(seq);
                }
            }
            double avg = total_time / NUM_ROUNDS;
            printf("Average: %f\n", avg);
        }
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
