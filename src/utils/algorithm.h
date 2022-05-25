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
 * There are still a lot of compilers that claim to be fully C++17 compatible, but don't implement the Parallel Execution TS of the accompanying standard library.
 * This means that we mostly have to fall back to the things that C++11/14 provide when it comes to threading/parallelism/etc.
 *
 * \param from The index starts here (inclusive).
 * \param to The index ends here (not inclusive).
 * \param increment Add this to the index each time.
 * \param body The loop-body, as a closure. Receives the index on invocation.
 */
template<typename T>
void parallel_for(T from, T to, T increment, const std::function<void(const T)>& body)
{
    parallel_for_nowait<T>(from,to,increment,body).wait();
}

/* An implementation of parallel for nowait.
 * There are still a lot of compilers that claim to be fully C++17 compatible, but don't implement the Parallel Execution TS of the accompanying standard library.
 * This means that we mostly have to fall back to the things that C++11/14 provide when it comes to threading/parallelism/etc.
 *
 * Ensure your capture is correct if you leave scope before waiting on the returned future!
 *
 * \param from The index starts here (inclusive).
 * \param to The index ends here (not inclusive).
 * \param increment Add this to the index each time.
 * \param body The loop-body, as a closure. Receives the index on invocation.
 * \return A future to wait on.
 */
template<typename T>
std::future<void> parallel_for_nowait(T from, T to, T increment, const std::function<void(const T)>& body)
{

    // Sanity tests.
    assert((increment > 0 && from <= to) || (increment < 0 && from >= to) );

    const std::function<void()> starter = [=](){

        // Run all tasks.
        std::vector<std::future<void>> scope_guard;
        for (T index = from; index < to; index += increment)
        {
            scope_guard.push_back(std::async(std::launch::async, body, index));
        }

        for (std::future<void>& loop_iteration : scope_guard)
        {
            loop_iteration.wait();
        }

    };

    auto ret = std::async(std::launch::async, starter);
    return ret;
}


} // namespace cura

#endif // UTILS_ALGORITHM_H

