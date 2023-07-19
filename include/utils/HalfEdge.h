// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_HALF_EDGE_H
#define UTILS_HALF_EDGE_H

#include "../utils/IntPoint.h"
#include "Coord_t.h"

#include <forward_list>
#include <optional>

namespace cura
{

template<typename node_data_t, typename edge_data_t, typename derived_node_t, typename derived_edge_t>
class HalfEdge
{
    using edge_t = derived_edge_t;
    using node_t = derived_node_t;

public:
    edge_data_t data;
    edge_t* twin = nullptr;
    edge_t* next = nullptr;
    edge_t* prev = nullptr;
    node_t* from = nullptr;
    node_t* to = nullptr;
    HalfEdge(edge_data_t data)
        : data(data)
    {
    }
    bool operator==(const edge_t& other)
    {
        return this == &other;
    }
};


} // namespace cura
#endif // UTILS_HALF_EDGE_H
