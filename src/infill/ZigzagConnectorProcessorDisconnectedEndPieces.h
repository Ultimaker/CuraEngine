/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_DISCONNECTED_END_PIECES_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_DISCONNECTED_END_PIECES_H

#include "../utils/polygon.h"
#include "ZigzagConnectorProcessorEndPieces.h"

namespace cura
{

class ZigzagConnectorProcessorDisconnectedEndPieces : public ZigzagConnectorProcessorEndPieces
{

public:
    ZigzagConnectorProcessorDisconnectedEndPieces(const PointMatrix& rotation_matrix, Polygons& result, bool skip_some_zags = false, int zag_skip_count = 0)
    : ZigzagConnectorProcessorEndPieces(rotation_matrix, result, skip_some_zags, zag_skip_count)
    {
    }
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_DISCONNECTED_END_PIECES_H