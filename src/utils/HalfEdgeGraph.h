//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_GRAPH_H
#define UTILS_HALF_EDGE_GRAPH_H


#include <forward_list>



#include "HalfEdge.h"
#include "HalfEdgeNode.h"

namespace arachne
{

template<class node_t, class edge_t> // types of data contained in nodes and edges
class HalfEdgeGraph
{
public:
    std::forward_list<HalfEdge<node_t, edge_t>> edges;
    std::forward_list<HalfEdgeNode<node_t, node_t>> nodes;
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_GRAPH_H
