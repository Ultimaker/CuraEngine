#include "MinimumSpanningTree.h"

namespace cura
{

MinimumSpanningTree::MinimumSpanningTree(const std::vector<Point> vertices) : vertices(vertices), edges(prim(vertices))
{
    //Just copy over the fields.
}

int MinimumSpanningTree::Edge::length() const
{
    return vSize(*start - *end);
}

const std::vector<MinimumSpanningTree::Edge> MinimumSpanningTree::prim(const std::vector<Point>& vertices) const
{
    //TODO: Implement this.
    return std::vector<MinimumSpanningTree::Edge>();
}

}