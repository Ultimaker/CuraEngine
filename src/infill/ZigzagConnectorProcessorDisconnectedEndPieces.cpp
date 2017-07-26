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
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            addLine(zigzag_connector.back(), intersection);
        }
        else if (!previous_scanline_is_even && !this_scanline_is_even) // if we end an odd boundary in an odd segment
        { // add whole oddBoundarySegment (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            // skip the last segment to the [intersection]
        }
    }
    zigzag_connector.clear(); // we're starting a new (odd) zigzag connector, so clear the old one
    zigzag_connector.push_back(intersection);
    last_connector_point = intersection;
}


void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyFinished()
{
    const bool is_last_piece_end_piece = last_scanline_is_even == first_zigzag_connector_ends_in_even_scanline;
    const bool add_last_piece = is_last_piece_end_piece || last_scanline_is_even;

    if (add_last_piece)
    {
        for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); ++point_idx)
        {
            addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
        }
        if (zigzag_connector.size() > 0 && first_zigzag_connector.size() > 0)
        {
            addLine(zigzag_connector.back(), first_zigzag_connector[0]);
        }
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size() - 1; ++point_idx)
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }

    // write very last line segment if needed
    if (!is_last_piece_end_piece && first_zigzag_connector.size() >= 2)
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
