//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "settings/EnumSettings.h"
#include "settings/ZSeamConfig.h" //To group Z seam requirements together.
#include "utils/polygon.h"
#include "utils/polygonUtils.h"

namespace cura
{

/*!
 * Parts order optimization class.
 * 
 * Utility class for optimizing the path order by minimizing the distance traveled between printing different parts in the layer.
 * The order of polygons is optimized and the startingpoint within each polygon is chosen.
 */
class PathOrderOptimizer
{
public:
    Point startPoint; //!< A location near the prefered start location
    const ZSeamConfig config;
    std::vector<ConstPolygonPointer> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons
    LocToLineGrid* loc_to_line;
    const Polygons* combing_boundary;

    PathOrderOptimizer(Point startPoint, const ZSeamConfig config = ZSeamConfig(), const Polygons* combing_boundary = nullptr)
    : startPoint(startPoint)
    , config(config)
    , combing_boundary((combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr)
    {
    }

    void addPolygon(PolygonRef polygon)
    {
        polygons.emplace_back(polygon);
    }

    void addPolygon(ConstPolygonRef polygon)
    {
        polygons.emplace_back(polygon);
    }

    void addPolygons(const Polygons& polygons)
    {
        for(unsigned int i = 0; i < polygons.size(); i++)
        {
            this->polygons.emplace_back(polygons[i]);
        }
    }

    void optimize(); //!< sets #polyStart and #polyOrder

private:
    int getClosestPointInPolygon(Point prev, int i_polygon); //!< returns the index of the closest point
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
    std::vector<ConstPolygonPointer> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons
    LocToLineGrid* loc_to_line;
    const Polygons* combing_boundary; //!< travel moves that cross this boundary are penalised so they are less likely to be chosen

    LineOrderOptimizer(Point startPoint, const Polygons* combing_boundary = nullptr)
    {
        this->startPoint = startPoint;
        this->combing_boundary = (combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr;
    }

    void addPolygon(PolygonRef polygon)
    {
        polygons.push_back(polygon);
    }

    void addPolygon(ConstPolygonRef polygon)
    {
        polygons.push_back(polygon);
    }

    void addPolygons(Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
        {
            this->polygons.push_back(polygons[i]);
        }
    }

    /*!
     * Do the optimization
     *
     * \param find_chains Whether to determine when lines are chained together (i.e. zigzag infill)
     *
     * \return The squared travel distance between the two points
     */
    void optimize(bool find_chains = true); //!< sets #polyStart and #polyOrder

private:
    /*!
     * Update LineOrderOptimizer::polyStart if the current line is better than the current best.
     * 
     * \param poly_idx[in] The index in LineOrderOptimizer::polygons for the current line to test
     * \param best[in, out] The index of current best line
     * \param best_score[in, out] The distance score for the current best line
     * \param prev_point[in] The previous point from which to find the next best line
     * \param just_point[in] If not -1, only look at the line vertex with this index
     */
    void updateBestLine(unsigned int poly_idx, int& best, float& best_score, Point prev_point, int just_point = -1);

    /*!
     * Compute the squared distance from \p p0 to \p p1 using combing
     *
     * \param p0 A point
     * \param p1 Another point
     *
     * \return The squared travel distance between the two points
     */
    float combingDistance2(const Point &p0, const Point &p1);
};

}//namespace cura

#endif//PATHOPTIMIZER_H
