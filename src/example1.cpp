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
#include <iostream>
#include <random>
#include <vector>
#include <unordered_set>

#include "ips4o.hpp"

template<class T>
void check_correctness(const std::vector<T> in, std::vector<int> counts) {
    std::unordered_set<T> s{};
    T prev = in[0];
    s.insert(prev);
    for (auto &e : in) {
        counts[e]--;
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
    for (int i = 0; i < counts.size(); i++) {
        if (counts[i] != 0) {
            fprintf(stderr, "Error: not semisorted\n");
            std::cout << "\nfound incorrect count of " << i << std::endl;
            exit(EXIT_FAILURE);
        }
        //std::cout << i << ": " << counts[i]  << std::endl;
    }
    printf("Pass\n");
}

int main(int argc, char** argv) {
    std::random_device r;
    std::default_random_engine gen(r());
    std::uniform_real_distribution<double> dist;

    std::vector<int> v(10000000);
    int range = 1000000;
    std::vector<int> counts(range, 0);
    
    for (auto& e : v) {
        e = int(dist(gen) * range);
        //std::cout << e << " ";
        counts[e]++;
    }
#if defined(_REENTRANT)
    ips4o::parallel::sort(v.begin(), v.end(), std::less<>{});
#else
    ips4o::sort(v.begin(), v.end(), std::less<>{});
#endif
    //const bool sorted = std::is_sorted(v.begin(), v.end(), std::less<>{});
    //std::cout << "Elements are sorted: " << std::boolalpha << sorted << std::endl;
    check_correctness(v, counts);
    return 0;
}
