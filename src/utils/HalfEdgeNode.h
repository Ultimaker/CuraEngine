//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_NODE_H
#define UTILS_HALF_EDGE_NODE_H

#include <list>

namespace arachne
{

template<typename node_t, typename edge_t>
class HalfEdge;

template<typename node_t, typename edge_t>
class HalfEdgeNode
{
    using edge_it = typename std::forward_list<HalfEdge<node_t, edge_t>>::iterator;
public:
    node_t data;
    edge_it some_edge;
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_NODE_H
