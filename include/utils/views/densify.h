// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_UTILS_VIEWS_DENSIFY_H
#define CURAENGINE_INCLUDE_UTILS_VIEWS_DENSIFY_H

#include <coroutine>
#include <vector>
#include <range/v3/view/generate.hpp>

#include "utils/concepts/geometry.h"

namespace cura::views
{
namespace details
{
template<concepts::point T>
struct PointsOnLineGenerator {
    struct promise_type {
        std::vector<T> points;
        std::size_t index = 0;

        PointsOnLineGenerator get_return_object() { return {this}; }
        std::suspend_always initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        void unhandled_exception() {}

        bool done() const { return index == points.size(); }

        T current_value() const { return points[index]; }

        void advance() { ++index; }

        std::suspend_always yield_value(T value) {
            points.push_back(value);
            return {};
        }

        void return_void() {}
    };

    using Handle = std::coroutine_handle<promise_type>;

    Handle handle;

    PointsOnLineGenerator(promise_type* p)
        : handle(Handle::from_promise(*p)) {}

    ~PointsOnLineGenerator() {
        if (handle) handle.destroy();
    }

    bool done() const { return handle.done(); }

    T operator()() {
        T value = handle.promise().current_value();
        handle.promise().advance();
        if (!handle.done()) handle.resume();
        return value;
    }
};

template<concepts::point T>
PointsOnLineGenerator<T> generatePointsOnLine(const T& point_start, const T& point_end, size_t n) {
    co_yield point_start;

    double dx = (point_end.X - point_start.X) / (n + 1);
    double dy = (point_end.Y - point_start.Y) / (n + 1);

    for (int i = 1; i <= n; ++i) {
        co_yield T{point_start.X + i * dx, point_end.Y + i * dy};
    }

    co_yield point_end;
}

struct densify_view_fn
{
    template<ranges::viewable_range Rng>
    constexpr auto operator()(Rng&& rng, size_t n) const
    {
        auto generator = generatePointsOnLine(rng, n);
    }
};

} // namespace details
} // namespace cura::views

#endif // CURAENGINE_INCLUDE_UTILS_VIEWS_DENSIFY_H
