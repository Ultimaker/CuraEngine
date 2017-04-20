/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessorNoEndPieces.h"


namespace cura
{

void ZigzagConnectorProcessorNoEndPieces::registerVertex(const Point& vertex)
{
    if (is_first_zigzag_connector)
    {
        first_zigzag_connector.push_back(vertex);
    }
    else if (last_scanline_is_even)
    {
        zigzag_connector.push_back(vertex);
    }
}


void ZigzagConnectorProcessorNoEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    bool previous_scanline_is_even = last_scanline_is_even;
    last_scanline_is_even = scanline_is_even;
    bool this_scanline_is_even = last_scanline_is_even; // for conceptual clarity

    if (is_first_zigzag_connector)
    {
        first_zigzag_connector.push_back(intersection);
        first_zigzag_connector_ends_in_even_scanline = this_scanline_is_even;
        is_first_zigzag_connector = false;
    }
    else
    {
        if (previous_scanline_is_even && !this_scanline_is_even)
        { // add whole zigzag_connector (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            addLine(zigzag_connector.back(), intersection);
            zigzag_connector.clear();
        }
    }
    zigzag_connector.clear(); // we're starting a new zigzag connector, so clear the old one
    if (this_scanline_is_even) // only boundary segments starting in an even segment are considered
    {
        zigzag_connector.push_back(intersection);
    }

}


void ZigzagConnectorProcessorNoEndPieces::registerPolyFinished()
{
    if (!is_first_zigzag_connector && last_scanline_is_even && !first_zigzag_connector_ends_in_even_scanline)
    { // only if it's a normal zigzag connector; not when the whole boundary didn't cross any scanlines
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size() ; point_idx++)
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }
    // reset member variables
    is_first_zigzag_connector = true;
    first_zigzag_connector_ends_in_even_scanline = true;
    last_scanline_is_even = false;
    first_zigzag_connector.clear();
    zigzag_connector.clear();
}


} // namespace cura
