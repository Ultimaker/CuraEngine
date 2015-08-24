/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "utils/polygon.h"
#include "settings.h"

namespace cura {
 
/*!
 * Parts order optimization class.
 * 
 * Utility class for optimizing the path order by minimizing the distance traveled between printing different parts in the layer.
 * The order of polygons is optimized and the startingpoint within each polygon is chosen.
 */
class PathOrderOptimizer
{
public:
    EZSeamType type;
    Point startPoint; //!< The location of the nozzle before starting to print the current layer
    std::vector<PolygonRef> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons

    PathOrderOptimizer(Point startPoint, EZSeamType type = EZSeamType::SHORTEST)
    : type(type)
    , startPoint(startPoint)
    {
    }

    void addPolygon(PolygonRef polygon)
    {
        this->polygons.push_back(polygon);
    }

    void addPolygons(Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
            this->polygons.push_back(polygons[i]);
    }

    void optimize(); //!< sets #polyStart and #polyOrder

private:
    int getPolyStart(Point prev_point, int poly_idx);
    int getClosestPointInPolygon(Point prev, int i_polygon); //!< returns the index of the closest point
    int getFarthestPointInPolygon(int poly_idx); //!< return the index to the point farthest from the front (highest y)
    int getRandomPointInPolygon(int poly_idx);


};
//! Line path order optimization class.
/*!
* Utility class for optimizing the path order by minimizing the distance traveled between printing different lines within a part.
*/
class LineOrderOptimizer
{
public:
    Point startPoint; //!< The location of the nozzle before starting to print the current layer
    std::vector<PolygonRef> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons

    LineOrderOptimizer(Point startPoint)
    {
        this->startPoint = startPoint;
    }

    void addPolygon(PolygonRef polygon)
    {
        this->polygons.push_back(polygon);
    }

    void addPolygons(Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
            this->polygons.push_back(polygons[i]);
    }

    void optimize(); //!< sets #polyStart and #polyOrder

private:
    void checkIfLineIsBest(unsigned int i_line_polygon, int& best, float& bestDist, Point& prev_point, Point& incommingPerpundicularNormal);

};

}//namespace cura

#endif//PATHOPTIMIZER_H
