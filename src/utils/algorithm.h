/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_ALGORITHM_H
#define UTILS_ALGORITHM_H

#include <algorithm>
#include <vector>
#include <functional>

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

} // namespace cura

#endif // UTILS_ALGORITHM_H

