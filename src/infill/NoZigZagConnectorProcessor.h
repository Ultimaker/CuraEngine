/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H

#include "../utils/polygon.h"
#include "ZigzagConnectorProcessor.h"

namespace cura
{

/*!
 * This processor adds no connection. This is for line infill pattern.
 */
class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    // The two "false"s are settings for zig-zag end pieces, which is not applicable here
    // because this processor doesn't do anything.
    NoZigZagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result)
    : ZigzagConnectorProcessor(rotation_matrix, result, false, false)
    {
    }

    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, int scanline_index, int direction);
    void registerPolyFinished();
};


} // namespace cura


#endif // INFILL_NO_ZIGZAG_CONNECTOR_PROCESSOR_H