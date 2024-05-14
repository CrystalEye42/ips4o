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

#include "ips4o.hpp"

int main(int argc, char** argv) {
    std::random_device r;
    std::default_random_engine gen(r());
    std::uniform_real_distribution<double> dist;

    std::vector<int> v(100);
    for (auto& e : v) {
        e = int(dist(gen) * 100);
    }

#if defined(_REENTRANT)
    ips4o::parallel::sort(v.begin(), v.end(), std::less<>{});
#else
    ips4o::sort(v.begin(), v.end(), std::less<>{});
#endif
    for (auto e : v) {
        std::cout << e << " ";
    }
    std::cout << std::endl;
    //const bool sorted = std::is_sorted(v.begin(), v.end(), std::less<>{});
    //std::cout << "Elements are sorted: " << std::boolalpha << sorted << std::endl;

    return 0;
}
