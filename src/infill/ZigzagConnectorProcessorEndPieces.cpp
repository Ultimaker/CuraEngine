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
    else
    { // it's yet unclear whether the line segment should be included, so we store it until we know
        zigzag_connector.push_back(vertex);
    }
}


} // namespace cura 
