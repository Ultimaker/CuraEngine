/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef POLYGON_FLOW_ADJUSTER_H
#define POLYGON_FLOW_ADJUSTER_H

#include "utils/intpoint.h"
#include "utils/polygon.h"

namespace cura 
{

/*!
 * Class for computing and compensating the flow of line segments in a polygon.
 * 
 */
class PolygonFlowAdjuster
{
public:
    /*!
     * Compute the flow for a given line segment in the polygons
     * 
     * \warning should only be called once for each line segment in a polygon!
     * 
     * \param from the polygons from which to get the segment of a flow, which should be the same polygons as the ones which the PolygonFlowAdjuster was constructed with
     * \param poly_idx Index to the polygon in which to find the line segment
     * \param from_point_idx The index to the beginning of the line segment
     * \param to_point_idx The index to the ending of the line segment
     * \return a value between zero and one representing the reduced flow of the line segment
     */
    virtual float getFlow(const Polygons& from, unsigned int poly_idx, unsigned int from_point_idx, unsigned int to_point_idx) = 0;

    virtual ~PolygonFlowAdjuster()
    {
    }
};


}//namespace cura



#endif//POLYGON_FLOW_ADJUSTER_H
