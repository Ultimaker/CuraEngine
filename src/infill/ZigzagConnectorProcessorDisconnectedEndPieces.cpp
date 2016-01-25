/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessorDisconnectedEndPieces.h"


namespace cura 
{

void ZigzagConnectorProcessorDisconnectedEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    bool previous_scanline_is_even = last_scanline_is_even;
    last_scanline_is_even = scanline_is_even;
    bool this_scanline_is_even = last_scanline_is_even;
    
    if (is_first_zigzag_connector) 
    {
        first_zigzag_connector.push_back(intersection);
        first_zigzag_connector_ends_in_even_scanline = this_scanline_is_even;
        is_first_zigzag_connector = false;
    }
    else
    {
        if (previous_scanline_is_even && !this_scanline_is_even)
        { // if we left from an even scanline, but not if this is the line segment connecting that zigzag_connector to an even scanline
            addLine(last_connector_point, intersection);
        }
        else if (!previous_scanline_is_even && !this_scanline_is_even) // if we end an odd boundary in an odd segment
        { // add whole oddBoundarySegment (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            // skip the last segment to the [intersection]
            zigzag_connector.clear();
        }
        
    }
    zigzag_connector.clear(); // we're starting a new (odd) zigzag connector, so clear the old one
    if (!this_scanline_is_even) // we are either in an end piece or an boundary segment starting in an odd scanline
    { // only when a boundary segment starts in an odd scanline it depends on whether it ends in an odd scanline for whether this segment should be included or not
        zigzag_connector.push_back(intersection);
    }

    last_connector_point = intersection;
}


void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyFinished()
{
    // write end segment if needed (first half of start/end-crossing segment)
    if (!last_scanline_is_even && !first_zigzag_connector_ends_in_even_scanline)
    {
        for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
        {
            addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
        }
    }
    // write begin segment if needed (second half of start/end-crossing segment)
    if (last_scanline_is_even || is_first_zigzag_connector)
    {
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size() - 1; point_idx++) // -1 cause skipping very last line segment!
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }
    // write very last line segment if needed
    if (last_scanline_is_even && !first_zigzag_connector_ends_in_even_scanline)
    { // only add last element if boundary segment ends in odd scanline
        addLine(first_zigzag_connector[first_zigzag_connector.size() - 2], first_zigzag_connector[first_zigzag_connector.size() - 1]);
    }
    // reset member variables
    is_first_zigzag_connector = true;
    first_zigzag_connector_ends_in_even_scanline = true;
    last_scanline_is_even = false; 
    first_zigzag_connector.clear();
    zigzag_connector.clear();
}


} // namespace cura 
