//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_ORDER_OPTIMIZER_H
#define UTILS_ORDER_OPTIMIZER_H

#include <stdint.h>
#include <vector>
#include <list>
#include <utility> // pair
#include "IntPoint.h"

namespace cura {
 
/*!
 * Order optimization class.
 * 
 * Utility class for optimizing the path order by minimizing the cyclic distance traveled between several items.
 * 
 * The path is heuristically optimized in a way such that each node is visited and the salesman which is travelling ends up where he started.
 */
template <typename T>
class OrderOptimizer
{
public:
    std::vector<std::pair<const Point, T>> items; //!< the items in arbitrary order

    OrderOptimizer()
    {
    }

    void addItem(const Point location, const T item);

    /*!
     * Optimize the order of \ref OrderOptimizer::items
     * layer_start_position A start layer position
     * \return A vector of the ordered indices into \ref OrderOptimizer::items
     */
    std::list<unsigned int> optimize(const Point layer_start_position = Point(0,0));


};

template <typename T>
void OrderOptimizer<T>::addItem(const Point location, const T item)
{
    this->items.emplace_back(location, item);
}

template <typename T>
std::list<unsigned int> OrderOptimizer<T>::optimize(const Point layer_start_position)
{
    // least detour insertion algorithm
    std::list<unsigned int> order;
    if (items.size() == 0)
    {
        return order;
    }

    const Point start_point = layer_start_position;

    unsigned int starting_item_index = 0;
    int64_t closest_distance = vSize(start_point - items[0].first);

    // Find the closest item's index to the starting point
    for (unsigned int item_idx = 0; item_idx < items.size(); item_idx++)
    {
        Point first_point = items[item_idx].first;
        Point temp = start_point - first_point;
        const int64_t diff = vSize(temp);

        if(diff < closest_distance)
        {
            closest_distance = diff;
            starting_item_index = item_idx;
        }
    }

    // Add first item to the order list
    order.push_front(starting_item_index);

    for (unsigned int item_idx = 0; item_idx < items.size(); item_idx++)
    {
        // skip the index because it was already added
        if(item_idx == starting_item_index)
        {
            continue;
        }

        Point to_insert_item_location = items[item_idx].first;

        // find best_item_to_insert_before
        std::list<unsigned int>::iterator best_item_to_insert_before = order.begin();
        coord_t best_detour_dist = vSize(items[*best_item_to_insert_before].first - to_insert_item_location)
                                + vSize(to_insert_item_location - items[order.back()].first)
                                - vSize(items[*best_item_to_insert_before].first - items[order.back()].first);
        std::list<unsigned int>::iterator prev = order.begin();
        for (std::list<unsigned int>::iterator nearby = ++order.begin(); nearby != order.end(); ++nearby)
        {
            coord_t detour_dist = vSize(items[*nearby].first - to_insert_item_location)
                                + vSize(to_insert_item_location - items[*prev].first)
                                - vSize(items[*nearby].first - items[*prev].first);
            if (detour_dist < best_detour_dist)
            {
                best_detour_dist = detour_dist;
                best_item_to_insert_before = nearby;
            }
            prev = nearby;
        }

        order.insert(best_item_to_insert_before, item_idx);
    }

    // Move the last item to the first position
    order.pop_back(); // Remove item from the end, the start position at the end, but should be at the beginning
    order.push_front(starting_item_index);

    return order;
}


}//namespace cura

#endif//UTILS_ORDER_OPTIMIZER_H
