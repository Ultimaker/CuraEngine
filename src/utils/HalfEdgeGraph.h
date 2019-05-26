//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_GRAPH_H
#define UTILS_HALF_EDGE_GRAPH_H


#include <forward_list>



#include "HalfEdge.h"
#include "HalfEdgeNode.h"
#include "SVG.h"

namespace arachne
{

template<class node_data_t, class edge_data_t> // types of data contained in nodes and edges
class HalfEdgeGraph
{
public:
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
    std::forward_list<edge_t> edges;
    std::forward_list<node_t> nodes;

    void debugOutput(std::string filename);
    void debugOutput(SVG& svg);
};



template<class node_data_t, class edge_data_t>
void HalfEdgeGraph<node_data_t, edge_data_t>::debugOutput(std::string filename)
{
    AABB aabb;
    for (node_t& node : nodes)
    {
        aabb.include(node.p);
    }
    SVG svg(filename.c_str(), aabb);
    debugOutput(svg);
}


template<class node_data_t, class edge_data_t>
void HalfEdgeGraph<node_data_t, edge_data_t>::debugOutput(SVG& svg)
{
//     for (node_t& node : nodes)
//     {
//         svg.writePoint(node.p);
//     }
    for (edge_t& edge : edges)
    {
        svg.writeLine(edge.from->p, edge.to->p, SVG::Color::RED);
    }
}



} // namespace arachne
#endif // UTILS_HALF_EDGE_GRAPH_H
