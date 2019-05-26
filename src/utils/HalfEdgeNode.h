//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_NODE_H
#define UTILS_HALF_EDGE_NODE_H

#include <list>

#include "IntPoint.h"

namespace arachne
{

template<typename node_data_t, typename edge_data_t>
class HalfEdge;

template<typename node_data_t, typename edge_data_t>
class HalfEdgeNode
{
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
public:
    node_data_t data;
    Point p;
    edge_t* some_edge = nullptr;
    HalfEdgeNode(node_data_t data, Point p)
    : data(data)
    , p(p)
    {}
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_NODE_H
