/******************************************************************************
 * include/ips4o/sampling.hpp
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

#include <iterator>
#include <random>
#include <utility>

#include "ips4o_fwd.hpp"
#include "classifier.hpp"
#include "config.hpp"
#include "memory.hpp"
#include "utils.hpp"

namespace ips4o {
namespace detail {

/**
 * Builds the classifer.
 * Number of used_buckets is a power of two and at least two.
 */
template <class Cfg>
std::pair<int, bool> Sorter<Cfg>::buildClassifier(const iterator begin,
                                                  const iterator end,
                                                  Classifier& classifier, 
                                                  const uint32_t shift_bits) {
    const auto n = end - begin;
    const size_t hash_bits = sizeof(uint32_t) * 8;

    const double LOAD_FACTOR = 1.2;
    const size_t SAMPLE_QUOTIENT = 500;
    const size_t HEAVY_THRESHOLD = log(n);
    const size_t NUM_SAMPLES = SAMPLE_QUOTIENT * HEAVY_THRESHOLD;

    std::vector<std::pair<size_t, size_t>> heavy_seq;
    const size_t hash_table_size = size_t{1} << log2_up(static_cast<size_t>(NUM_SAMPLES * LOAD_FACTOR));
    const size_t hash_table_mask = hash_table_size - 1;
    std::vector<std::pair<size_t,size_t>> hash_table(hash_table_size, {ULLONG_MAX, 0});
    //std::cout << "num samples " << NUM_SAMPLES << ", threshold " << HEAVY_THRESHOLD << std::endl;
    // Select the sample
    for (size_t i = 0; i < NUM_SAMPLES; i++) {
        size_t v = (hash64(i)>>shift_bits) % n;
        size_t idx = hash32(begin[v]) & hash_table_mask;
        while (hash_table[idx].first != ULLONG_MAX && begin[v] != begin[hash_table[idx].first]) {
            idx = (idx + 1) & hash_table_mask;
        }
        if (hash_table[idx].first == ULLONG_MAX) {
            hash_table[idx].first = v;
        }
        hash_table[idx].second++;
    }
    size_t heavy_buckets = 0;
    for (size_t i = 0; i < hash_table_size; i++) {
        if (hash_table[i].second >= HEAVY_THRESHOLD) {
            heavy_seq.push_back({hash_table[i].first, heavy_buckets++});
            //std::cout << hash_table[i].first <<", " << hash_table[i].second<< ", "<<n << std::endl;
        }
    }
    const size_t LOG2_LIGHT_KEYS = (n > 512) ? 10 : log2_up(n);
    const size_t LIGHT_MASK = (1 << LOG2_LIGHT_KEYS) - 1;
    const size_t light_buckets = 1 << LOG2_LIGHT_KEYS;
    this->num_buckets_ = light_buckets + heavy_buckets;
    
    // Construct lookup table
    size_t heavy_id_size = size_t{1} << log2_up(heavy_seq.size() * 5 + 1);
    size_t heavy_id_mask = heavy_id_size - 1;
    std::vector<std::pair<value_type, size_t>> heavy_id(heavy_id_size, {ULLONG_MAX, ULLONG_MAX});
    for (const auto& [k, v] : heavy_seq) {
        size_t idx = hash32(begin[k]) & heavy_id_mask;
        while (heavy_id[idx].first != ULLONG_MAX) {
            idx = (idx + 1) & heavy_id_mask;
        }
        heavy_id[idx] = {begin[k], v};
        //std::cout << k <<", "<<v <<std::endl;
    }
    //std::cout << "done" << std::endl;

    classifier.build(heavy_id, heavy_id_mask, heavy_buckets, shift_bits, LOG2_LIGHT_KEYS);
    this->classifier_ = &classifier;
    
    //std::cout << "done" << std::endl;
    return {heavy_buckets + light_buckets, false};

    /*
    auto lookup = [&](size_t k) {
        size_t idx = hash(g(In[k])) >> shift_bits & heavy_id_mask;
        while (heavy_id[idx].first != ULLONG_MAX && !equal(g(In[heavy_id[idx].first]), g(In[k]))) {
        idx = (idx + 1) & heavy_id_mask;
        }
        return heavy_id[idx].second;
    };
    // 2. count the number of light/heavy keys
    size_t LOG2_LIGHT_KEYS = std::min<size_t>(hash_bits - shift_bits, 10);
    size_t LIGHT_MASK = (1 << LOG2_LIGHT_KEYS) - 1;
    size_t light_buckets = 1 << LOG2_LIGHT_KEYS;
    size_t heavy_buckets = heavy_seq.size();
    size_t num_buckets = heavy_buckets + light_buckets;
    auto f = [&](size_t i) {
        size_t it = lookup(i);
        if (it != ULLONG_MAX) {
        // In[i] is a heavy key
        return it + light_buckets;
        } else {
        // In[i] is a light key
        size_t hash_v = hash(g(In[i])) >> shift_bits;
        return hash_v & LIGHT_MASK;
        }
    };*/
    /*
    // Sort the sample
    sequential(begin, begin + num_samples);
    auto splitter = begin + step - 1;
    auto sorted_splitters = classifier.getSortedSplitters();
    const auto comp = classifier.getComparator();

    // Choose the splitters
    IPS4OML_ASSUME_NOT(sorted_splitters == nullptr);
    new (sorted_splitters) typename Cfg::value_type(*splitter);
    for (int i = 2; i < num_buckets; ++i) {
        splitter += step;
        // Skip duplicates
        if (comp(*sorted_splitters, *splitter)) {
            IPS4OML_ASSUME_NOT(sorted_splitters + 1 == nullptr);
            new (++sorted_splitters) typename Cfg::value_type(*splitter);
        }
    }

    // Check for duplicate splitters
    const auto diff_splitters = sorted_splitters - classifier.getSortedSplitters() + 1;
    const bool use_equal_buckets = Cfg::kAllowEqualBuckets
            && num_buckets - 1 - diff_splitters >= Cfg::kEqualBucketsThreshold;

    // Fill the array to the next power of two
    log_buckets = log2(diff_splitters) + 1;
    num_buckets = 1 << log_buckets;
    for (int i = diff_splitters + 1; i < num_buckets; ++i) {
        IPS4OML_ASSUME_NOT(sorted_splitters + 1 == nullptr);
        new (++sorted_splitters) typename Cfg::value_type(*splitter);
    }

    // Build the tree
    classifier.build(log_buckets);
    this->classifier_ = &classifier;

    const int used_buckets = num_buckets * (1 + use_equal_buckets);
    return {used_buckets, use_equal_buckets};*/
}

}  // namespace detail
}  // namespace ips4o
