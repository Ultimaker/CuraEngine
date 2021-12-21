//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_ALGORITHM_H
#define UTILS_ALGORITHM_H

#include <algorithm>
#include <atomic>
#include <functional>
#include <future>
#include <numeric>
#include <vector>

// extensions to algorithm.h from the standard library

namespace cura
{

/*!
 * Get the order of a vector: the sorted indices of a vector
 * 
 * {1.6, 1.8, 1.7} returns {1, 3, 2} meaning {in[1], in[3], in[2]} is a sorted
 * vector
 * 
 * Thanks to Lukasz Wiklendt
 * 
 * \param in The vector for which to get the order
 * \return An ordered vector of indices into \p in 
 */
template<typename T>
std::vector<size_t> order(const std::vector<T> &in)
{
    // initialize original index locations
    std::vector<size_t> order(in.size());
    std::iota(order.begin(), order.end(), 0); // fill vector with 1, 2, 3,.. etc

    // sort indexes based on comparing values in v
    std::sort(order.begin(), order.end(),
        [&in](size_t i1, size_t i2)
        {
            return in[i1] < in[i2];
        }
    );

    return order;
}

/* An implementation of parallel for.
 * There are still a lot of compilers that claim to be fully C++17 compatible, but don't implement the Parallel Execution TS of the accompanying standard lybrary.
 * This means that we moslty have to fall back to the things that C++11/14 provide when it comes to threading/parallelism/etc.
 *
 * \param from The index starts here (inclusive).
 * \param to The index ends here (not inclusive).
 * \param increment Add this to the index each time.
 * \param body The loop-body, as a closure. Receives the index on invocation.
 */
template<typename T>
void parallel_for(T from, T to, T increment, const std::function<void(const T)>& body)
{
    // Sanity tests.
    assert(increment > 0);
    assert(from <= to);

    // Set the values so that 'to' is a whole integer multiple apart from 'from' (& early out if needed).
    to = from + increment * (((to - from) + (increment - static_cast<T>(1))) / increment);
    if (to == from)
    {
        return;
    }

    // Set an atomic countdown variable to how many tasks need to be completed.
    std::atomic<T> tasks_pending((to - from) / increment);

    // Wrap the loop-body, so that the outer scope can be notified by 'all_tasks_done'.
    std::promise<void> all_tasks_done;
    const auto func =
        [&body, &tasks_pending, &all_tasks_done](const T index)
        {
            body(index);
            if (--tasks_pending == 0)
            {
                all_tasks_done.set_value();
            }
        };

    // Run all tasks.
    std::vector<std::future<void>> scope_guard;
    for (size_t index = from; index != to; index += increment)
    {
        scope_guard.push_back(std::async(std::launch::async, func, index));
    }

    // Wait for the end-result before return.
    all_tasks_done.get_future().wait();
}

} // namespace cura

#endif // UTILS_ALGORITHM_H

