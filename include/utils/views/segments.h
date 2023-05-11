// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_SEGMENTS_H
#define UTILS_VIEWS_SEGMENTS_H

#include <range/v3/range/concepts.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/cycle.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/sliding.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/zip.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{

// TODO: this view should loop over the lines of a polyline or polygon, not the points.
//   It should return a tuple of two points, not a single point.
//   If it is a polygon, the last point should be connected to the first point.

template<ranges::input_range Rng>
requires concepts::poly_range<Rng> class segment_view : public ranges::view_interface<segment_view<Rng>>
{
private:
    Rng base_{};
    mutable ranges::iterator_t<Rng> begin_{ std::begin(base_) };
    mutable ranges::iterator_t<Rng> end_{ std::end(base_) };

public:
    constexpr segment_view() noexcept = default;
    constexpr explicit segment_view(Rng base) noexcept : base_(std::move(base)), begin_(std::begin(base_)), end_(std::end(base_))
    {
    }

    constexpr Rng base() const&
    {
        return base_;
    }

    constexpr Rng base() &&
    {
        return std::move(base_);
    }

    constexpr auto begin() const
    {
        if constexpr (is_closed_v<decltype(base_)>)
        {
            return base_ | ranges::views::cycle | ranges::views::sliding(2) | ranges::views::take(std::distance(begin_, end_) + 1) | ranges::views::transform([](auto tup) { return std::make_tuple(tup.front(), tup.back()); });
        }
        else
        {
            return base_ | ranges::views::sliding(2) | ranges::views::transform([](auto tup) { return std::make_tuple(tup.front(), tup.back()); });
        }
    }

    constexpr auto end() const
    {
        if constexpr (is_closed_v<decltype(base_)>)
        {
            return std::next(std::begin(base_));
        }
        else
        {
            return std::end(base_);
        }
    }

    constexpr auto size() requires ranges::sized_range<Rng>
    {
        if constexpr (is_closed_v<decltype(base_)>)
        {
            return std::distance(begin_, end_) + 1;
        }
        else
        {
            return std::distance(begin_, end_);
        }
    }

    constexpr auto size() const requires ranges::sized_range<const Rng>
    {
        if constexpr (is_closed_v<decltype(base_)>)
        {
            return std::distance(begin_, end_) + 1;
        }
        else
        {
            return std::distance(begin_, end_);
        }
    }
};

template<class Rng>
segment_view(Rng&& base) -> segment_view<ranges::views::all_t<Rng>>;

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