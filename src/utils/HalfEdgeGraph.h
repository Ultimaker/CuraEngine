//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_GRAPH_H
#define UTILS_HALF_EDGE_GRAPH_H


#include <list>
#include <cassert>



#include "HalfEdge.h"
#include "HalfEdgeNode.h"
#include "SVG.h"

namespace arachne
{
    using namespace cura;

template<class node_data_t, class edge_data_t, class derived_node_t, class derived_edge_t> // types of data contained in nodes and edges
class HalfEdgeGraph
{
public:
    using edge_t = derived_edge_t;
    using node_t = derived_node_t;
    std::list<edge_t> edges;
    std::list<node_t> nodes;
};

} // namespace arachne
#endif // UTILS_HALF_EDGE_GRAPH_H
