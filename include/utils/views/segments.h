// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SEGMENTS_H
#define UTILS_VIEWS_SEGMENTS_H

#include <iterator>
#include <range/v3/range/concepts.hpp>
#include <range/v3/utility/common_tuple.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/cycle.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/zip.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{

template<ranges::input_range Rng>
requires concepts::poly_range<Rng> class segment_view : public ranges::view_interface<segment_view<Rng>>
{
private:
    Rng base_{};
    ranges::iterator_t<Rng> it_{ std::begin(base_) };
    ranges::iterator_t<Rng> it_next_{ std::next(std::begin(base_)) };

    class iterator
    {
    private:
        using base_iterator_t = ranges::iterator_t<Rng>;

    public:
        // FIXME: These should be private
        base_iterator_t it_;
        base_iterator_t it_next_;

        using value_type = ranges::range_value_t<Rng>;
        using difference_type = ranges::range_difference_t<Rng>;
        using reference = ranges::common_pair<value_type&, value_type&>;
        using pointer = ranges::common_pair<value_type*, value_type*>;
        using iterator_category = std::input_iterator_tag;

        iterator(base_iterator_t it, base_iterator_t it_next) : it_(it), it_next_(it_next)
        {
        }

        reference operator*() const
        {
            return { *it_, *it_next_ };
        }

        iterator& operator++()
        {
            ++it_;
            ++it_next_;
            return *this;
        }

        bool operator==(const iterator& other) const
        {
            return it_ == other.it_ && it_next_ == other.it_next_;
        }

        bool operator!=(const iterator& other) const
        {
            return ! (*this == other);
        }
    };

    class sentinel
    {
    private:
        using base_sentinel_t = ranges::sentinel_t<Rng>;
        base_sentinel_t end_;

    public:
        sentinel(base_sentinel_t end) : end_(end)
        {
        }

        friend bool operator!=(const iterator& it, const sentinel& s)
        {
            return it.it_next_ != s.end_;
        }
    };

    bool equal(ranges::default_sentinel_t) const
    {
        return it_next_ == std::end(base_);
    }

    constexpr auto read() const
    {
        return ranges::make_common_pair(*it_, *it_next_);
    }

    void next()
    {
        std::next(it_);
        std::next(it_next_);
    }

    void prev()
    {
        std::prev(it_);
        std::prev(it_next_);
    }

public:
    constexpr segment_view() noexcept = default;
    constexpr explicit segment_view(Rng rng) noexcept : base_{ std::move(rng) }
    {
    }

    constexpr auto begin()
    {
        return iterator{ std::begin(base_), std::next(std::begin(base_)) };
    }

    constexpr auto begin() const
    {
        return iterator{ std::begin(base_), std::next(std::begin(base_)) };
    }

    constexpr auto end()
    {
        return sentinel{ std::end(base_) };
    }

    constexpr auto end() const
    {
        return sentinel{ std::end(base_) };
    }

    constexpr auto size() requires ranges::sized_range<Rng>
    {
        if constexpr (is_closed_v<Rng>)
        {
            return std::distance(std::begin(base_), std::end(base_)) + 1;
        }
        return std::distance(std::begin(base_), std::end(base_));
    }

    constexpr auto size() const requires ranges::sized_range<const Rng>
    {
        if constexpr (is_closed_v<Rng>)
        {
            return std::distance(std::begin(base_), std::end(base_)) + 1;
        }
        return std::distance(std::begin(base_), std::end(base_));
    }
};

template<class Rng>
segment_view(Rng base) -> segment_view<ranges::views::all_t<Rng>>;

namespace details
{
struct segment_view_range_adaptor_closure
{
    template<ranges::viewable_range Rng>
    constexpr auto operator()(Rng&& rng) const
    {
        return segment_view(std::forward<Rng>(rng));
    }
};

template<ranges::viewable_range Rng>
constexpr auto operator|(Rng&& rng, const segment_view_range_adaptor_closure& adaptor)
{
    return adaptor(std::forward<Rng>(rng));
}

} // namespace details

constexpr inline static details::segment_view_range_adaptor_closure segments;

} // namespace cura::views


#endif // UTILS_VIEWS_SEGMENTS_H