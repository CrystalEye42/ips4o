/******************************************************************************
 * include/ips4o/classifier.hpp
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

#include <type_traits>
#include <utility>

#include "ips4o_fwd.hpp"
#include "utils.hpp"

namespace ips4o {
namespace detail {

/**
 * Branch-free classifier.
 */
template <class Cfg>
class Sorter<Cfg>::Classifier {
    using iterator = typename Cfg::iterator;
    using value_type = typename Cfg::value_type;
    using bucket_type = typename Cfg::bucket_type;
    using less = typename Cfg::less;

 public:
    Classifier(less comp) : comp_(std::move(comp)) {}

    ~Classifier() {
        if (log_buckets_) cleanup();
    }

    /**
     * Calls destructors on splitter elements.
     */
    void reset() {
        if (log_buckets_) cleanup();
    }

    /**
     * The sorted array of splitters, to be filled externally.
     */
    value_type* getSortedSplitters() {
        return static_cast<value_type*>(static_cast<void*>(sorted_storage_));
    }

    /**
     * The comparison operator.
     */
    less getComparator() const { return comp_; }

    void build(std::vector<std::pair<value_type, size_t>> h_id, size_t mask, size_t hb, uint32_t sb) {
        heavy_buckets = hb;
        heavy_id = std::vector<std::pair<value_type, size_t>>();
        for (auto [e1, e2]: h_id) {
            heavy_id.push_back({e1,e2});
        }
        heavy_id_mask = mask;
        shift_bits = sb;
    }

    /**
     * Classifies a single element.
     */
    template <bool kEqualBuckets>
    bucket_type classify(const value_type& value) const {
        const size_t num_buckets = heavy_buckets + light_buckets;
        size_t idx = hash32(value) & heavy_id_mask;
        while (heavy_id[idx].first != ULLONG_MAX && heavy_id[idx].first != value) {
            idx = (idx + 1) & heavy_id_mask;
        }
        size_t it = heavy_id[idx].second;
        if (it != ULLONG_MAX) {
        // In[i] is a heavy key
            //std::cout << "local classify result " << value << ", " << it + light_buckets << std::endl;
            return it + light_buckets;
        } else {
            // In[i] is a light key
            size_t hash_v = (hash32(value)>>shift_bits);
            //std::cout << "local classify result " << value << ", " << (hash_v & LIGHT_MASK) << std::endl;
            return hash_v & LIGHT_MASK;
        }
    }

    /**
     * Classifies all elements using a callback.
     */
    template <bool kEqualBuckets, class Yield> 
    void classify(iterator begin, iterator end, Yield&& yield) const {
        for (auto [e1, e2]: heavy_id) {
            if (e1 != ULLONG_MAX) {
            //std::cout << "found "  <<orig_begin[e1] <<", "<< e1 << ", " << e2 << std::endl;
            }
        }
        const size_t num_buckets = heavy_buckets + light_buckets;
        bool flag = true;
        for (int i = 0; i < end - begin; i++) {
            const value_type value = begin[i];
            size_t idx = hash32(value) & heavy_id_mask;

            while (heavy_id[idx].first != ULLONG_MAX && heavy_id[idx].first != value) {
                idx = (idx + 1) & heavy_id_mask;
            }
            if (flag && heavy_id[idx].first != value) {
              //  std::cout << value<< ", " <<idx1 << ", "<<idx << ", "<<heavy_id[idx1].first << ", "<< heavy_id[idx].first <<std::endl;
                flag = false;
            }
            size_t it = heavy_id[idx].second;
            if (it != ULLONG_MAX) {
            // In[i] is a heavy key
                        //std::cout << "classify result " << " " << value << ", " << it + light_buckets << std::endl;

                yield(it + light_buckets, begin + i);
            } else {
                //std::cout << "ERROR " << i << ", " << value  << std::endl;
                // In[i] is a light key
                size_t hash_v = hash32(value)>>shift_bits;
                            //std::cout << "classify result " << value << ", " << (hash_v & LIGHT_MASK) << std::endl;

                yield(hash_v & LIGHT_MASK, begin + i);
            }
        }
    }

 private:
    const size_t LOG2_LIGHT_KEYS = 10;
    const size_t LIGHT_MASK = (1 << LOG2_LIGHT_KEYS) - 1;
    const size_t light_buckets = 1 << LOG2_LIGHT_KEYS;
    std::vector<std::pair<value_type, size_t>> heavy_id;
    size_t heavy_buckets;
    size_t hash_table_mask;
    size_t heavy_id_mask;
    uint32_t shift_bits;
    

    const value_type& splitter(bucket_type i) const {
        return static_cast<const value_type*>(static_cast<const void*>(storage_))[i];
    }

    const value_type& sortedSplitter(bucket_type i) const {
        return static_cast<const value_type*>(
                static_cast<const void*>(sorted_storage_))[i];
    }

    value_type* data() {
        return static_cast<value_type*>(static_cast<void*>(storage_));
    }

    /**
     * Recursively builds the tree.
     */
    void build(const value_type* const left, const value_type* const right,
               const bucket_type pos) {
        const auto mid = left + (right - left) / 2;
        IPS4OML_ASSUME_NOT(data() + pos == nullptr);
        new (data() + pos) value_type(*mid);
        if (2 * pos < num_buckets_) {
            build(left, mid, 2 * pos);
            build(mid, right, 2 * pos + 1);
        }
    }

    /**
     * Destructs splitters.
     */
    void cleanup() {
        auto p = data() + 1;
        auto q = getSortedSplitters();
        for (int i = num_buckets_ - 1; i; --i) {
            p++->~value_type();
            q++->~value_type();
        }
        q->~value_type();
        log_buckets_ = 0;
    }

    // Filled from 1 to num_buckets_
    std::aligned_storage_t<sizeof(value_type), alignof(value_type)>
            storage_[Cfg::kMaxBuckets / 2];
    // Filled from 0 to num_buckets_, last one is duplicated
    std::aligned_storage_t<sizeof(value_type), alignof(value_type)>
            sorted_storage_[Cfg::kMaxBuckets / 2];
    int log_buckets_ = 0;
    bucket_type num_buckets_ = 0;
    less comp_;
};

}  // namespace detail
}  // namespace ips4o
