/*
 * Copyright 2017 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <set>
#include <iterator>
#include <cassert>

template<class T>
class subsets {
public:
    using set_type = std::set<T>;

    class iterator {
    public:
        using difference_type = unsigned;
        using value_type = std::set<T>;
        using pointer = value_type*;
        using reference = value_type;
        using iterator_category = std::input_iterator_tag;

        bool operator!=(iterator const& other) const {
            return (pos_ != other.pos_) || (set_ != other.set_);
        }
        
        bool operator==(iterator const& other) const {
			return (pos_ == other.pos_) && (set_ == other.set_);
		}
		
        reference operator*() const {
            assert(pos_ < (1u<<set_->size()));
            return subset_;
        }

        pointer operator->() const {
            assert(pos_ < (1u<<set_->size()));
            return &subset_;
        }

        iterator& operator++() {
            ++pos_;
            compute_subset();
            return *this;
        }

        iterator operator++(int) {
            iterator ret {*this};
            ++pos_;
            compute_subset();
            return ret;
        }

    private:
        iterator(unsigned pos, set_type const& set) : pos_(pos), set_(&set) {
            compute_subset();
        }

        void compute_subset() {
            subset_.clear();
            unsigned tmp = pos_;
            for(auto& x: *set_) {
                if (tmp&1)
                    subset_.emplace(x);
                tmp >>= 1;
            }
        }

        unsigned pos_;
        set_type const* set_;
        set_type subset_;

        friend class subsets;
    };


    subsets(set_type const& set) : set_(set) {
        assert(set.size() < sizeof(unsigned)*8);
    }

    iterator begin() const {
        return iterator(0, set_);
    }

    iterator end() const {
        return iterator((1<<set_.size()), set_);
    }

private:
    set_type const& set_;
};
