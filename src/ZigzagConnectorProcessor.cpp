/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ZigzagConnectorProcessor.h"


namespace cura 
{

void ZigzagConnectorProcessorNoEndPieces::skipVertex(const Point& vertex)
{

}

void ZigzagConnectorProcessorNoEndPieces::registerVertex(const Point& vertex)
{
    if (isFirstBoundarySegment)
    {
        firstBoundarySegment.push_back(vertex);
    }
    else if (isEvenScanSegment)
    {
        boundarySegment.push_back(vertex);
    }
}

void ZigzagConnectorProcessorConnectedEndPieces::skipVertex(const Point& vertex)
{
    lastPoint = vertex;
}

void ZigzagConnectorProcessorConnectedEndPieces::registerVertex(const Point& vertex)
{
    if (isFirstBoundarySegment)
    {
        firstBoundarySegment.push_back(vertex);
    }
    else if (isEvenScanSegment)
    {
        addLine(lastPoint, vertex);
    }
    else
    {
        unevenBoundarySegment.push_back(vertex);
    }
    lastPoint = vertex;
}

void ZigzagConnectorProcessorDisconnectedEndPieces::skipVertex(const Point& vertex)
{
    lastPoint = vertex;
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerVertex(const Point& vertex)
{
    if (isFirstBoundarySegment)
    {
        firstBoundarySegment.push_back(vertex);
    }
    else if (isEvenScanSegment)
    {
        addLine(lastPoint, vertex);
    }
//     else if (connect_zigzags)
//     {
//         unevenBoundarySegment.push_back(p1);
//     }
    lastPoint = vertex;
}


void ZigzagConnectorProcessorNoEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    bool last_isEvenScanSegment = isEvenScanSegment;
    isEvenScanSegment = scanline_is_even;
    
    if (isFirstBoundarySegment) 
    {
        firstBoundarySegment.push_back(intersection);
        firstBoundarySegmentEndsInEven = isEvenScanSegment;
        isFirstBoundarySegment = false;
        boundarySegment.push_back(intersection);
    }
    else
    {
        if (last_isEvenScanSegment && !isEvenScanSegment)
        { // add whole boundarySegment (including the just obtained point)
            for (unsigned int p = 1; p < boundarySegment.size(); p++)
            {
                addLine(boundarySegment[p-1], boundarySegment[p]);
            }
            addLine(boundarySegment[boundarySegment.size()-1], intersection);
            boundarySegment.clear();
        } 
        else if (isEvenScanSegment) // we are either in an end piece or an uneven boundary segment
        {
            boundarySegment.clear();
            boundarySegment.push_back(intersection);
        }
        else
        {
            boundarySegment.clear();
        }
            
    }
    
}

void ZigzagConnectorProcessorConnectedEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    bool last_isEvenScanSegment = isEvenScanSegment;
    isEvenScanSegment = scanline_is_even;
    
    if (isFirstBoundarySegment)
    {
        firstBoundarySegment.push_back(intersection);
        firstBoundarySegmentEndsInEven = isEvenScanSegment;
        isFirstBoundarySegment = false;
    }
    else
    {
        if (last_isEvenScanSegment)
        {
            addLine(lastPoint, intersection);
        }
        else if (!last_isEvenScanSegment && !isEvenScanSegment) // if we end an uneven boundary in an uneven segment
        { // add whole unevenBoundarySegment (including the just obtained point)
            for (unsigned int p = 1; p < unevenBoundarySegment.size(); p++)
            {
                addLine(unevenBoundarySegment[p-1], unevenBoundarySegment[p]);
            }
            addLine(unevenBoundarySegment[unevenBoundarySegment.size()-1], intersection);
            unevenBoundarySegment.clear();
        } 
        if (last_isEvenScanSegment && !isEvenScanSegment)
        {
            unevenBoundarySegment.push_back(intersection);
        }
        else 
        {
            unevenBoundarySegment.clear();
        }
            
    }
    lastPoint = intersection;
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{
    bool last_isEvenScanSegment = isEvenScanSegment;
    isEvenScanSegment = scanline_is_even;
    
    if (isFirstBoundarySegment) 
    {
        firstBoundarySegment.push_back(intersection);
        firstBoundarySegmentEndsInEven = isEvenScanSegment;
        isFirstBoundarySegment = false;
    }
    else
    {
        if (last_isEvenScanSegment && !isEvenScanSegment)
        {
            addLine(lastPoint, intersection);
        }
        if (last_isEvenScanSegment && !isEvenScanSegment)
        {
            unevenBoundarySegment.push_back(intersection);
        }
        else 
        {
            unevenBoundarySegment.clear();
        }
            
    }
    
    lastPoint = intersection;
}


void ZigzagConnectorProcessorNoEndPieces::registerPolyFinished()
{
    if (!isFirstBoundarySegment && isEvenScanSegment && !firstBoundarySegmentEndsInEven)
    {
        for (unsigned int i = 1; i < firstBoundarySegment.size() ; i++)
        {
            addLine(firstBoundarySegment[i-1], firstBoundarySegment[i]);
        }
    }
    isFirstBoundarySegment = true;
    firstBoundarySegmentEndsInEven = true;
    isEvenScanSegment = false; 
    firstBoundarySegment.clear();
    boundarySegment.clear();
}


void ZigzagConnectorProcessorConnectedEndPieces::registerPolyFinished()
{
    {
        for (unsigned int i = 1; i < firstBoundarySegment.size() ; i++)
        {
            addLine(firstBoundarySegment[i-1], firstBoundarySegment[i]);
        }   
    }
//     else if (!firstBoundarySegmentEndsInEven)
//     {
//         addLine(firstBoundarySegment[firstBoundarySegment.size()-2], firstBoundarySegment[firstBoundarySegment.size()-1]);
//     }
}

void ZigzagConnectorProcessorDisconnectedEndPieces::registerPolyFinished()
{
    if (isEvenScanSegment || isFirstBoundarySegment)
    {
        for (unsigned int i = 1; i < firstBoundarySegment.size() ; i++)
        {
            if (i < firstBoundarySegment.size() - 1 || !firstBoundarySegmentEndsInEven) // only add last element if connect_zigzags or boundary segment ends in uneven scanline
            {
                addLine(firstBoundarySegment[i-1], firstBoundarySegment[i]);
            }
        }   
    }
    else if (!firstBoundarySegmentEndsInEven)
    {
        addLine(firstBoundarySegment[firstBoundarySegment.size()-2], firstBoundarySegment[firstBoundarySegment.size()-1]);
    }
}



void NoZigZagConnectorProcessor::registerPolyFinished()
{

}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even)
{

}

void NoZigZagConnectorProcessor::registerVertex(const Point& vertex)
{

}

void NoZigZagConnectorProcessor::skipVertex(const Point& vertex)
{

}



} // namespace cura 
