/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_ACTUAL_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_ACTUAL_ZIGZAG_CONNECTOR_PROCESSOR_H


#include "../utils/polygon.h"
#include "ZigzagConnectorProcessor.h"
#include "../utils/intpoint.h"

namespace cura
{

/*!
 * In contrast to NoZigZagConnectorProcessor
 */
class ActualZigzagConnectorProcessor : public ZigzagConnectorProcessor
{
protected:
    /*!
     * The line segments belonging the zigzag connector to which the very first vertex belongs. 
     * This will be combined with the last handled zigzag_connector, which combine to a whole zigzag connector.
     * 
     * Because the boundary polygon may start in in the middle of a zigzag connector, 
     */
    std::vector<Point> first_zigzag_connector; 
    /*!
     * The currently built up zigzag connector (not the first/last) or end piece or discarded boundary segment
     */
    std::vector<Point> zigzag_connector; 

    bool is_first_zigzag_connector; //!< Whether we're still in the first zigzag connector
    bool first_zigzag_connector_ends_in_even_scanline; //!< Whether the first zigzag connector ends in an even scanline
    bool last_scanline_is_even;  //!< Whether the last seen scanline-boundary intersection was with an even scanline

    ActualZigzagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result)
    : ZigzagConnectorProcessor(rotation_matrix, result)
    , is_first_zigzag_connector(true)
    , first_zigzag_connector_ends_in_even_scanline(true)
    , last_scanline_is_even(false) 
    {
    }
};

} // namespace cura


#endif // INFILL_ACTUAL_ZIGZAG_CONNECTOR_PROCESSOR_H