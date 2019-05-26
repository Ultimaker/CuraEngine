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
    edge_t* twin = nullptr;
    edge_t* next = nullptr;
    edge_t* prev = nullptr;
    node_t* from = nullptr;
    node_t* to = nullptr;
    HalfEdge(edge_data_t data)
    : data(data)
    {}
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_H
