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

template<class node_data_t, class edge_data_t> // types of data contained in nodes and edges
class HalfEdgeGraph
{
public:
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
    std::list<edge_t> edges;
    std::list<node_t> nodes;

    void debugOutput(std::string filename);
    void debugOutput(SVG& svg);

    bool bedugCheckDataCompleteness() const;
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


template<class node_data_t, class edge_data_t>
bool HalfEdgeGraph<node_data_t, edge_data_t>::bedugCheckDataCompleteness() const
{
    size_t problems = 0;
    for (const node_t& node : nodes)
    {
        if (!node.some_edge)
        {
            problems++;
            assert(false);
        }
    }
    for (const edge_t& edge : edges)
    {
        if (!edge.twin || !edge.next || !edge.prev || !edge.from || !edge.to)
        {
            problems++;
            assert(false);
        }
    }
    
    assert(problems == 0);
    return problems == 0;
}



} // namespace arachne
#endif // UTILS_HALF_EDGE_GRAPH_H
