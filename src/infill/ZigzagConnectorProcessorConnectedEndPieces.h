/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_CONNECTED_END_PIECES_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_CONNECTED_END_PIECES_H

#include "../utils/polygon.h"
#include "ZigzagConnectorProcessorEndPieces.h"
#include "../utils/intpoint.h"

namespace cura
{


class ZigzagConnectorProcessorConnectedEndPieces : public ZigzagConnectorProcessorEndPieces
{
public:
    ZigzagConnectorProcessorConnectedEndPieces(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessorEndPieces(matrix, result)
    {
    }
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_CONNECTED_END_PIECES_H