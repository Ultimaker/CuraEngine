//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_H
#define UTILS_HALF_EDGE_H

#include <forward_list>

namespace arachne
{

template<typename node_data_t, typename edge_data_t>
class HalfEdgeNode;


template<typename node_data_t, typename edge_data_t>
class HalfEdge
{
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
public:
    edge_data_t data;
    edge_t* twin;
    edge_t* next;
    edge_t* prev;
    node_t* from;
    node_t* to;
    HalfEdge(edge_data_t data)
    : data(data)
    {}
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_H
