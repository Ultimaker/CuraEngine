/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessor.h"


namespace cura 
{

void ZigzagConnectorProcessorNoEndPieces::registerPolyStart(const Point& vertex)
{

}

void ZigzagConnectorProcessorConnectedEndPieces::registerPolyStart(const Point& vertex)
{
    last_connector_point = vertex;
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyStart(const Point& vertex)
{
    last_connector_point = vertex;
}

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

void ZigzagConnectorProcessorConnectedEndPieces::registerVertex(const Point& vertex)
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
        zigzag_connector_starting_in_uneven_scanline.push_back(vertex);
    }
    last_connector_point = vertex;
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerVertex(const Point& vertex)
{
    if (is_first_zigzag_connector)
    {
        first_zigzag_connector.push_back(vertex);
    }
    else if (last_scanline_is_even)
    {
        addLine(last_connector_point, vertex);
    }
//     else if (connect_zigzags) // TODO: shouldn't there be something like this here?
//     {
//         unevenBoundarySegment.push_back(p1);
//     }
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
        zigzag_connector.push_back(intersection);
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
        else if (this_scanline_is_even) // we are either in an end piece or an uneven boundary segment
        {
            zigzag_connector.clear();
            zigzag_connector.push_back(intersection);
        }
        else
        {
            zigzag_connector.clear();
        }
            
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
            for (unsigned int point_idx = 1; point_idx < zigzag_connector_starting_in_uneven_scanline.size(); point_idx++)
            {
                addLine(zigzag_connector_starting_in_uneven_scanline[point_idx - 1], zigzag_connector_starting_in_uneven_scanline[point_idx]);
            }
            addLine(zigzag_connector_starting_in_uneven_scanline[zigzag_connector_starting_in_uneven_scanline.size() - 1], intersection);
            zigzag_connector_starting_in_uneven_scanline.clear();
        } 
        if (previous_scanline_is_even && !last_scanline_is_even)
        {
            zigzag_connector_starting_in_uneven_scanline.push_back(intersection);
        }
        else 
        {
            zigzag_connector_starting_in_uneven_scanline.clear();
        }
            
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
        {
            addLine(last_connector_point, intersection);
        }
        if (previous_scanline_is_even && !last_scanline_is_even) // TODO: decide what needs to be done with thos block of code! the else case doesn't make sense! ?
        {
            zigzag_connector_starting_in_uneven_scanline.push_back(intersection);
        }
        else 
        {
            zigzag_connector_starting_in_uneven_scanline.clear();
        }
            
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
    for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size(); point_idx++)
    {
        addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
    }
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyFinished()
{
    if (last_scanline_is_even || is_first_zigzag_connector)
    {
        for (unsigned int point_idx = 1; point_idx < first_zigzag_connector.size() - 1; point_idx++)
        {
            addLine(first_zigzag_connector[point_idx - 1], first_zigzag_connector[point_idx]);
        }
    }
    if (!first_zigzag_connector_ends_in_even_scanline)
    { // only add last element if connect_zigzags or boundary segment ends in uneven scanline
        addLine(first_zigzag_connector[first_zigzag_connector.size() - 2], first_zigzag_connector[first_zigzag_connector.size() - 1]);
    }
}



void NoZigZagConnectorProcessor::registerPolyStart(const Point& vertex)
{

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
