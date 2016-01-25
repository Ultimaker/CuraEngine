/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_END_PIECES_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_END_PIECES_H

#include "../utils/polygon.h"
#include "ActualZigzagConnectorProcessor.h"

namespace cura
{


class ZigzagConnectorProcessorEndPieces : public ActualZigzagConnectorProcessor
{
protected:
    Point last_connector_point; //!< last registered boundary vertex or scanline-coundary intersection

    ZigzagConnectorProcessorEndPieces(const PointMatrix& matrix, Polygons& result)
    : ActualZigzagConnectorProcessor(matrix, result)
    , last_connector_point(0,0)
    {
    }

public:
    void registerVertex(const Point& vertex);
};



} // namespace cura


#endif // INFILL_ZIGZAG_CONNECTOR_PROCESSOR_END_PIECES_H