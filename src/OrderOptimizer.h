//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ORDER_OPTIMIZER_H
#define ORDER_OPTIMIZER_H

#include <stdint.h>
#include "pathOrderOptimizer.h" // for ZSeamConfig
#include "utils/polygon.h"
#include "utils/polygonUtils.h"
#include "utils/types/EnumSettings.h"

namespace arachne {

/*!
 * Parts order optimization class.
 * 
 * Utility class for optimizing the path order by minimizing the distance traveled between printing different parts in the layer.
 * The order of polygons is optimized and the startingpoint within each polygon is chosen.
 */
class OrderOptimizer
{
    struct Path
    {
        ConstPolygonPointer poly;
        bool is_closed;
        Path(ConstPolygonPointer poly, bool is_closed)
        : poly(poly)
        , is_closed(is_closed)
        {}
    };
    std::vector<Path> polys; //!< the parts of the layer (in arbitrary order)
public:
    Point startPoint; //!< A location near the prefered start location
    const ZSeamConfig config;
    std::vector<int> polyStart; //!< polys[i][polyStart[i]] = point of poly i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polys
    LocToLineGrid* loc_to_line;

    OrderOptimizer(Point startPoint, const ZSeamConfig config = ZSeamConfig())
    : startPoint(startPoint)
    , config(config)
    {
    }

    void addPoly(PolygonRef poly, bool is_closed)
    {
        polys.emplace_back(poly, is_closed);
    }

    void addPoly(ConstPolygonRef poly, bool is_closed)
    {
        polys.emplace_back(poly,  is_closed);
    }

    void addPolys(const Polygons& polys, bool is_closed)
    {
        for(unsigned int i = 0; i < polys.size(); i++)
        {
            this->polys.emplace_back(polys[i], is_closed);
        }
    }

    void optimize(); //!< sets #polyStart and #polyOrder

private:
    int getClosestPointInPoly(Point prev, int i_poly); //!< returns the index of the closest point
    int getRandomPointInPoly(int poly_idx);
};

}//namespace cura

#endif//ORDER_OPTIMIZER_H
