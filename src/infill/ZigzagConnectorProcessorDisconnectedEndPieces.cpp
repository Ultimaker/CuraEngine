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
        const bool is_end_piece = previous_scanline_is_even == this_scanline_is_even;
        const bool add_connection = previous_scanline_is_even || is_end_piece;  // add connections on even segments or it is an endpiece

        if (add_connection)
        {
            if (skip_some_zags && ++current_zag_count >= zag_skip_count)
            {
                // skip this zag and reset the count
                current_zag_count = 0;
            }
            else
            {
                for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
                {
                    addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
                }
                // if it is an end piece, the last line will not be connected
                if (zigzag_connector.size() > 0 && !is_end_piece)
                {
                    addLine(zigzag_connector.back(), intersection);
                }
            }
        }
    }
    zigzag_connector.clear(); // we're starting a new (odd) zigzag connector, so clear the old one
    zigzag_connector.push_back(intersection);
    last_connector_point = intersection;
}


void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyFinished()
{
    const bool is_last_piece_end_piece = last_scanline_is_even == first_zigzag_connector_ends_in_even_scanline;
    const bool need_to_skip_this_piece = skip_some_zags && ++current_zag_count >= zag_skip_count;
    const bool add_last_piece = is_last_piece_end_piece || (last_scanline_is_even && !need_to_skip_this_piece);

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
