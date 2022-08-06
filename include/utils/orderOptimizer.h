//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_ORDER_OPTIMIZER_H
#define UTILS_ORDER_OPTIMIZER_H

#include <cstdint>
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
     * \return A vector of the ordered indices into \ref OrderOptimizer::items
     */
    std::list<size_t> optimize(const Point& start_position);
};

template <typename T>
void OrderOptimizer<T>::addItem(const Point location, const T item)
{
    items.emplace_back(location, item);
}

template <typename T>
std::list<size_t> OrderOptimizer<T>::optimize(const Point& start_position)
{
    // Use the nearest mesh ordering
    std::list<size_t> order;
    std::vector<size_t> item_idx_list;

    if (items.size() == 0)
    {
        return order;
    }

    for (size_t i = 0; i < items.size(); i++)
    {
        item_idx_list.emplace_back(i);
    }
    const Point* last_item_position = &start_position;

    while (!item_idx_list.empty())
    {
        coord_t shortest_distance = POINT_MAX;
        size_t shortest_distance_item_idx = -1;
        size_t idx_in_list = -1;

        for (size_t idx = 0; idx < item_idx_list.size(); idx++)
        {
            const size_t item_idx = item_idx_list[idx];
            const Point& position = items[item_idx].first;
            const coord_t distance = vSize(position - *last_item_position);
            if (distance < shortest_distance)
            {
                shortest_distance = distance;
                shortest_distance_item_idx = item_idx;
                idx_in_list = idx;
            }
        }

        order.push_back(shortest_distance_item_idx);
        last_item_position = &(items[shortest_distance_item_idx].first);
        item_idx_list.erase(item_idx_list.begin() + idx_in_list);
    }
    return order;
}


}//namespace cura

#endif//UTILS_ORDER_OPTIMIZER_H
