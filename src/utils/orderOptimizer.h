/** Copyright (C) 2016 Ultimaker B.V. - Released under terms of the AGPLv3 License */
#ifndef UTILS_ORDER_OPTIMIZER_H
#define UTILS_ORDER_OPTIMIZER_H

#include <stdint.h>
#include <vector>
#include <list>
#include <utility> // pair
#include "intpoint.h"

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
    std::list<unsigned int> optimize();


};

template <typename T>
void OrderOptimizer<T>::addItem(const Point location, const T item)
{
    this->items.emplace_back(location, item);
}

template <typename T>
std::list<unsigned int> OrderOptimizer<T>::optimize()
{
    // least detour insertion algorithm
    std::list<unsigned int> order;
    if (items.size() == 0)
    {
        return order;
    }
    order.push_back(0u);
    if (items.size() == 1)
    {
        return order;
    }
    order.push_back(1u);
    if (items.size() == 2)
    {
        return order;
    }
    order.push_back(2u);

    for (unsigned int item_idx = 3; item_idx < items.size(); item_idx++)
    {
        Point to_insert_item_location = items[item_idx].first;

        // find best_item_to_insert_before
        std::list<unsigned int>::iterator best_item_to_insert_before = order.begin();
        coord_t best_detour_dist = vSize(items[*best_item_to_insert_before].first - to_insert_item_location)
                                + vSize(to_insert_item_location - items[order.back()].first)
                                - vSize(items[*best_item_to_insert_before].first - items[order.back()].first);
        std::list<unsigned int>::iterator prev = order.begin();
        for (std::list<unsigned int>::iterator near = std::next(order.begin()); near != order.end(); ++near)
        {
            coord_t detour_dist = vSize(items[*near].first - to_insert_item_location)
                                + vSize(to_insert_item_location - items[*prev].first)
                                - vSize(items[*near].first - items[*prev].first);
            if (detour_dist < best_detour_dist)
            {
                best_detour_dist = detour_dist;
                best_item_to_insert_before = near;
            }
            prev = near;
        }

        order.insert(best_item_to_insert_before, item_idx);
    }
    return order;
}


}//namespace cura

#endif//UTILS_ORDER_OPTIMIZER_H
