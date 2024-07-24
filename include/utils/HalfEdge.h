// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_HALF_EDGE_H
#define UTILS_HALF_EDGE_H

#include <forward_list>
#include <optional>

#include "Coord_t.h"
#include "geometry/Point2LL.h"

namespace cura
{

template<typename node_data_t, typename edge_data_t, typename derived_node_t, typename derived_edge_t>
class HalfEdge
{
    using edge_t = derived_edge_t;
    using node_t = derived_node_t;

public:
    edge_data_t data_;
    edge_t* twin_ = nullptr;
    edge_t* next_ = nullptr;
    edge_t* prev_ = nullptr;
    node_t* from_ = nullptr;
    node_t* to_ = nullptr;

    HalfEdge(edge_data_t data)
        : data_(data)
    {
    }

    bool operator==(const edge_t& other)
    {
        return this == &other;
    }
};


} // namespace cura
#endif // UTILS_HALF_EDGE_H
