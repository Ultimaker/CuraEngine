/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef ZIGZAG_CONNECTOR_PROCESSOR_H
#define ZIGZAG_CONNECTOR_PROCESSOR_H

#include "utils/polygon.h"

namespace cura
{

class ZigzagConnectorProcessor 
{
protected:
    PointMatrix& matrix;
    Polygons& result;

    virtual ~ZigzagConnectorProcessor()
    {}
public:
    ZigzagConnectorProcessor(PointMatrix& matrix, Polygons& result)
    : matrix(matrix)
    , result(result)
    {}


    void addLine(Point from, Point to)
    {
        PolygonRef p = result.newPoly();
        p.add(matrix.unapply(from));
        p.add(matrix.unapply(to));
    }

    virtual void skipVertex(const Point& vertex) = 0;
    virtual void registerVertex(const Point& vertex) = 0;
    virtual void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even) = 0;
    virtual void registerPolyFinished() = 0;
};

class ZigzagConnectorProcessorNoEndPieces : public ZigzagConnectorProcessor
{
    std::vector<Point> firstBoundarySegment;
    std::vector<Point> boundarySegment;

    bool isFirstBoundarySegment;
    bool firstBoundarySegmentEndsInEven;
    bool isEvenScanSegment; 

public:
    ZigzagConnectorProcessorNoEndPieces(PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    , isFirstBoundarySegment(true)
    , firstBoundarySegmentEndsInEven(true)
    , isEvenScanSegment(false) 
    {
    }

    void skipVertex(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class ZigzagConnectorProcessorConnectedEndPieces : public ZigzagConnectorProcessor
{
    std::vector<Point> firstBoundarySegment;
    std::vector<Point> unevenBoundarySegment;

    Point lastPoint;

    bool isFirstBoundarySegment;
    bool firstBoundarySegmentEndsInEven;
    bool isEvenScanSegment; 

public:
    ZigzagConnectorProcessorConnectedEndPieces(PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    , lastPoint(0,0)
    , isFirstBoundarySegment(true)
    , firstBoundarySegmentEndsInEven(true)
    , isEvenScanSegment(false) 
    {
    }

    void skipVertex(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class ZigzagConnectorProcessorDisconnectedEndPieces : public ZigzagConnectorProcessor
{
    std::vector<Point> firstBoundarySegment;
    std::vector<Point> unevenBoundarySegment;

    Point lastPoint;

    bool isFirstBoundarySegment;
    bool firstBoundarySegmentEndsInEven;
    bool isEvenScanSegment; 

public:
    ZigzagConnectorProcessorDisconnectedEndPieces(PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    , lastPoint(0,0)
    , isFirstBoundarySegment(true)
    , firstBoundarySegmentEndsInEven(true)
    , isEvenScanSegment(false) 
    {
    }

    void skipVertex(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    NoZigZagConnectorProcessor(PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    {
    }

    void skipVertex(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};


} // namespace cura


#endif // ZIGZAG_CONNECTOR_PROCESSOR_H