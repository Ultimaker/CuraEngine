// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H

#include "ZigzagConnectorProcessor.h"
#include "geometry/OpenLinesSet.h"

namespace cura
{

class OpenPolyline;

template<class T>
class LinesSet;

/*!
 * This processor adds no connection. This is for line infill pattern.
 */
class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    NoZigZagConnectorProcessor(const PointMatrix& rotation_matrix, OpenLinesSet& result)
        : ZigzagConnectorProcessor(
            rotation_matrix,
            result,
            false,
            false, // settings for zig-zag end pieces, no use here
            false,
            0) // settings for skipping some zags, no use here
    {
    }

    void registerVertex(const Point2LL& vertex);
    void registerScanlineSegmentIntersection(const Point2LL& intersection, int scanline_index, coord_t min_distance_to_scanline);
    void registerPolyFinished();
};


} // namespace cura


#endif // INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H
