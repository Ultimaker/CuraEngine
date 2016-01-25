/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_NO_ENDPIECES_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_NO_ENDPIECES_H

#include "../utils/polygon.h"
#include "ActualZigzagConnectorProcessor.h"
#include "../utils/intpoint.h"

namespace cura
{

class ZigzagConnectorProcessorNoEndPieces : public ActualZigzagConnectorProcessor
{
public:
    ZigzagConnectorProcessorNoEndPieces(const PointMatrix& matrix, Polygons& result)
    : ActualZigzagConnectorProcessor(matrix, result)
    {
    }

    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};


} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_NO_ENDPIECES_H