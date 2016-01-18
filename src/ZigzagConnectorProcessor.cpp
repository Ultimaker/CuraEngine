/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessor.h"


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

void ZigzagConnectorProcessorEndPieces::registerVertex(const Point& vertex)
{
    if (is_first_zigzag_connector)
    {
        first_zigzag_connector.push_back(vertex);
    }
    else if (last_scanline_is_even)
    {
        addLine(last_connector_point, vertex);
    }
    else
    {
        zigzag_connector.push_back(vertex);
    }
    last_connector_point = vertex;
}



void ZigzagConnectorProcessorNoEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
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
        { // add whole boundarySegment (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            addLine(zigzag_connector.back(), intersection);
            zigzag_connector.clear();
        }
            
    }
    zigzag_connector.clear(); // we're starting a new zigzag connector, so clear the old one
    if (this_scanline_is_even) // we are either in an end piece or an uneven boundary segment
    {
        zigzag_connector.push_back(intersection);
    }
    
}

void ZigzagConnectorProcessorConnectedEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
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
        if (previous_scanline_is_even)
        {
            addLine(last_connector_point, intersection);
        }
        else if (!previous_scanline_is_even && !this_scanline_is_even) // if we end an uneven boundary in an uneven segment
        { // add whole unevenBoundarySegment (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            addLine(zigzag_connector[zigzag_connector.size() - 1], intersection);
            zigzag_connector.clear();
        }

    }
    zigzag_connector.clear(); // we're starting a new (uneven) zigzag connector, so clear the old one
    if (!this_scanline_is_even)
    {
        zigzag_connector.push_back(intersection);
    }
        
    last_connector_point = intersection;
}

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
        else if (!previous_scanline_is_even && !this_scanline_is_even) // if we end an uneven boundary in an uneven segment
        { // add whole unevenBoundarySegment (including the just obtained point)
            for (unsigned int point_idx = 1; point_idx < zigzag_connector.size(); point_idx++)
            {
                addLine(zigzag_connector[point_idx - 1], zigzag_connector[point_idx]);
            }
            // skip the last segment to the [intersection]
            zigzag_connector.clear();
        }
        
    }
    zigzag_connector.clear(); // we're starting a new (uneven) zigzag connector, so clear the old one
    if (!this_scanline_is_even)
    {
        zigzag_connector.push_back(intersection);
    }

    last_connector_point = intersection;
}


void ZigzagConnectorProcessorNoEndPieces::registerPolyFinished()
{
    if (!is_first_zigzag_connector && last_scanline_is_even && !first_zigzag_connector_ends_in_even_scanline)
    {
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size() ; point_idx++)
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }
    is_first_zigzag_connector = true;
    first_zigzag_connector_ends_in_even_scanline = true;
    last_scanline_is_even = false; 
    first_zigzag_connector.clear();
    zigzag_connector.clear();
}


void ZigzagConnectorProcessorConnectedEndPieces::registerPolyFinished()
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
    if (last_scanline_is_even || (!last_scanline_is_even && !first_zigzag_connector_ends_in_even_scanline)
        || is_first_zigzag_connector)
    {
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size(); point_idx++)
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }
    is_first_zigzag_connector = true;
    first_zigzag_connector_ends_in_even_scanline = true;
    last_scanline_is_even = false; 
    first_zigzag_connector.clear();
    zigzag_connector.clear();
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
    { // only add last element if boundary segment ends in uneven scanline
        addLine(first_zigzag_connector[first_zigzag_connector.size() - 2], first_zigzag_connector[first_zigzag_connector.size() - 1]);
    }
    is_first_zigzag_connector = true;
    first_zigzag_connector_ends_in_even_scanline = true;
    last_scanline_is_even = false; 
    first_zigzag_connector.clear();
    zigzag_connector.clear();
}



void NoZigZagConnectorProcessor::registerVertex(const Point& vertex)
{

}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{

}

void NoZigZagConnectorProcessor::registerPolyFinished()
{

}



} // namespace cura 
