#include "MinimumSpanningTree.h"

namespace cura
{

MinimumSpanningTree::MinimumSpanningTree(const std::vector<Point> vertices) : adjacency_graph(prim(vertices))
{
    //Just copy over the fields.
}

int MinimumSpanningTree::Edge::length() const
{
    return vSize(*start - *end);
}

const std::unordered_map<Point, std::vector<MinimumSpanningTree::Edge>> MinimumSpanningTree::prim(const std::vector<Point>& vertices) const
{
    //TODO: Implement this.
    return std::unordered_map<Point, std::vector<MinimumSpanningTree::Edge>>();
}

}