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
    bool skip_some_zags;  //!< (ZigZag) Whether to skip some zags
    int zag_skip_count;  //!< (ZigZag) To skip one zag in every N if skip some zags is enabled
    int current_zag_count;

    ActualZigzagConnectorProcessor(const PointMatrix& rotation_matrix, Polygons& result, bool skip_some_zags = false, int zag_skip_count = 0)
    : ZigzagConnectorProcessor(rotation_matrix, result)
    , is_first_zigzag_connector(true)
    , first_zigzag_connector_ends_in_even_scanline(true)
    , last_scanline_is_even(false)
    , skip_some_zags(skip_some_zags)
    , zag_skip_count(zag_skip_count)
    , current_zag_count(0)
    {
    }
};

} // namespace cura


#endif // INFILL_ACTUAL_ZIGZAG_CONNECTOR_PROCESSOR_H