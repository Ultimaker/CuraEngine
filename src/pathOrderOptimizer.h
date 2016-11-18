/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "utils/polygon.h"
#include "settings/settings.h"

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
    Point startPoint; //!< A location near the prefered start location
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
    /*!
     * Update LineOrderOptimizer::polyStart if the current line is better than the current best.
     * 
     * Besides looking at the distance from the previous line segment, we also look at the angle we make.
     * 
     * We prefer 90 degree angles; 180 degree turn arounds are slow on machines where the jerk is limited.
     * 0 degree (straight ahead) 'corners' occur only when a single infill line is interrupted, 
     * in which case the travel move might involve combing, which makes it rather longer.
     * 
     * \param poly_idx[in] The index in LineOrderOptimizer::polygons for the current line to test
     * \param best[in, out] The index of current best line
     * \param best_score[in, out] The distance score for the current best line
     * \param prev_point[in] The previous point from which to find the next best line
     * \param incoming_perpundicular_normal[in] The direction of movement when the print head arrived at \p prev_point, turned 90 degrees CCW
     */
    void updateBestLine(unsigned int poly_idx, int& best, float& best_score, Point prev_point, Point incoming_perpundicular_normal);

    /*!
     * Get a score to modify the distance score for measuring how good two lines follow each other.
     * 
     * The angle score is symmetric in \p from and \p to; they can be exchanged without altering the result. (Code relies on this property)
     * 
     * \param incoming_perpundicular_normal The direction in which the head was moving while printing the previous line, turned 90 degrees CCW
     * \param from The one end of the next line
     * \param to The other end of the next line
     * \return A score measuring how good the angle is of the line between \p from and \p to when the previous line had a direction given by \p incoming_perpundicular_normal 
     * 
     */
    static float getAngleScore(Point incoming_perpundicular_normal, Point from, Point to);
};

}//namespace cura

#endif//PATHOPTIMIZER_H
