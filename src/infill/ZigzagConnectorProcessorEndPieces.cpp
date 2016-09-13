/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessorEndPieces.h"


namespace cura 
{

void ZigzagConnectorProcessorEndPieces::registerVertex(const Point& vertex)
{
    if (is_first_zigzag_connector)
    {
        first_zigzag_connector.push_back(vertex);
    }
    else if (last_scanline_is_even)
    { // when a boundary segments starts in an even scanline it's either a normal zigzag connector or an endpiece to be included
        // note that for ZigzagConnectorProcessorDisconnectedEndPieces only the last line segment from a boundary vertex to a scanline-boundary intersection is omitted
        addLine(last_connector_point, vertex);
    }
    else
    { // it's yet unclear whether the line segment should be included, so we store it until we know
        zigzag_connector.push_back(vertex);
    }
    last_connector_point = vertex;
}


} // namespace cura 
