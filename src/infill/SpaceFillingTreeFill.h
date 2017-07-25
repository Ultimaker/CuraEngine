/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPACE_FILLING_TREE_FILL_H
#define INFILL_SPACE_FILLING_TREE_FILL_H

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/SpaceFillingTree.h"

#include "../utils/SVG.h"

namespace cura
{
class SpaceFillingTreeFill
{
public:
    SpaceFillingTreeFill()
    : aabb(AABB(Point(0, 0), Point(10000, 10000)))
    , tree(Point(5000, 5000), 5000, 3)
    {
        {
            SVG svg("debug.html", aabb);
            tree.debugOutput(svg);
            
            class Visitor : public SpaceFillingTree::LocationVisitor
            {
                int order_nr = 0;
                SVG& svg;
            public:
                Visitor(SVG& svg)
                :svg(svg)
                {}
                void visit(Point location)
                {
                    svg.writeText(location, std::to_string(order_nr));
                    order_nr++;
                }
            };
            Visitor visitor(svg);
            tree.walk(visitor);
        }
        std::cerr << "dsggds\n";
        std::exit(0);
    }
private:
    AABB aabb;
    SpaceFillingTree tree;
};
} // namespace cura


#endif // INFILL_SPACE_FILLING_TREE_FILL_H
