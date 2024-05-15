/******************************************************************************
 * include/ips4o/base_case.hpp
 *
 * In-place Parallel Super Scalar Samplesort (IPS⁴o)
 *
 ******************************************************************************
 * BSD 2-Clause License
 *
 * Copyright © 2017, Michael Axtmann <michael.axtmann@gmail.com>
 * Copyright © 2017, Daniel Ferizovic <daniel.ferizovic@student.kit.edu>
 * Copyright © 2017, Sascha Witt <sascha.witt@kit.edu>
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

#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>
#include <unordered_set>

#include "ips4o_fwd.hpp"
#include "utils.hpp"

namespace ips4o {
namespace detail {

/**
 * Insertion sort.
 */
template <class It, class Comp>
void insertionSort(const It begin, const It end, Comp comp) {
    IPS4OML_ASSUME_NOT(begin >= end);

    for (It it = begin + 1; it < end; ++it) {
        typename std::iterator_traits<It>::value_type val = std::move(*it);
        if (comp(val, *begin)) {
            std::move_backward(begin, it, it + 1);
            *begin = std::move(val);
        } else {
            auto cur = it;
            for (auto next = it - 1; comp(val, *next); --next) {
                *cur = std::move(*next);
                cur = next;
            }
            *cur = std::move(val);
        }
    }
}

/**
 * Hash table equal sort
 */
template <class It, class Comp>
void countingSort(const It begin, const It end, Comp comp) {
    const auto n = end - begin;
    if (n<=0) {
        return;
    }
    const size_t hash_table_size = size_t{1} << log2_up(static_cast<size_t>(n * 1.2));
    const size_t hash_table_mask = hash_table_size - 1;
    //std::cout << hash_table_size << std::endl;
    std::vector<std::pair<size_t, size_t>> hash_table(hash_table_size, {ULLONG_MAX, 0});
    for (int i = 0; i < n; ++i) {
        //std::cout << "index " << i << std::endl;
        const auto v = begin[i];
        size_t idx = hash32(v) & hash_table_mask;
        while (hash_table[idx].first != ULLONG_MAX && v != hash_table[idx].first) {
            idx = (idx + 1) & hash_table_mask;
        }
        if (hash_table[idx].first == ULLONG_MAX) {
            hash_table[idx].first = v;
        }
        hash_table[idx].second++;
    }
    It it = begin;
    for (const auto& [v, count]: hash_table) {
        for (int i = 0; i < count; ++i) {
            it[i] = v;
        }
        it += count;
    }
}

/**
 * Wrapper for base case sorter, for easier swapping.
 */
template <class It, class Comp>
inline void baseCaseSort(It begin, It end, Comp&& comp) {
    //if (begin == end) return;
    //detail::insertionSort(std::move(begin), std::move(end), std::forward<Comp>(comp));
    detail::countingSort(std::move(begin), std::move(end), std::forward<Comp>(comp));
}

template <class It, class Comp, class ThreadPool>
inline bool isSorted(It begin, It end, Comp&& comp, ThreadPool& thread_pool) {
    // Do nothing if input is already sorted.
    std::vector<bool> is_sorted(thread_pool.numThreads());
    thread_pool(
            [begin, end, &is_sorted, &comp](int my_id, int num_threads) {
                const auto size = end - begin;
                const auto stripe = (size + num_threads - 1) / num_threads;
                const auto my_begin = begin + std::min(stripe * my_id, size);
                const auto my_end = begin + std::min(stripe * (my_id + 1) + 1, size);
                is_sorted[my_id] = std::is_sorted(my_begin, my_end, comp);
            },
            thread_pool.numThreads());

    return std::all_of(is_sorted.begin(), is_sorted.end(), [](bool res) { return res; });
}

template <class It, class Comp>
inline bool sortSimpleCases(It begin, It end, Comp&& comp) {
    if (begin == end) {
        return true;
    }

    // If last element is not smaller than first element,
    // test if input is sorted (input is not reverse sorted).
    if (!comp(*(end - 1), *begin)) {
        if (std::is_sorted(begin, end, comp)) {
            return true;
        }
    } else {
        // Check whether the input is reverse sorted.
        for (It it = begin; (it + 1) != end; ++it) {
            if (comp(*it, *(it + 1))) {
                return false;
            }
        }
        std::reverse(begin, end);
        return true;
    }

    return false;
}

}  // namespace detail
}  // namespace ips4o
