//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_H
#define UTILS_HALF_EDGE_H

#include <forward_list>

namespace arachne
{

template<typename node_t, typename edge_t>
class HalfEdgeNode;


template<typename node_t, typename edge_t>
class HalfEdge
{
    using edge_it = typename std::forward_list<HalfEdge<node_t, edge_t>>::iterator;
    using node_it = typename std::forward_list<HalfEdgeNode<node_t, edge_t>>::iterator;
public:
    edge_t data;
    edge_it twin;
    edge_it next;
    edge_it prev;
    node_it from;
    node_it to;
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_H
