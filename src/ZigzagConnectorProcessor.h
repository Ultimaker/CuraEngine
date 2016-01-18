/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef ZIGZAG_CONNECTOR_PROCESSOR_H
#define ZIGZAG_CONNECTOR_PROCESSOR_H

#include "utils/polygon.h"

namespace cura
{

class ZigzagConnectorProcessor 
{
protected:
    const PointMatrix& matrix;
    Polygons& result;

    virtual ~ZigzagConnectorProcessor()
    {}
public:
    ZigzagConnectorProcessor(const PointMatrix& matrix, Polygons& result)
    : matrix(matrix)
    , result(result)
    {}


    void addLine(Point from, Point to)
    {
        PolygonRef line_poly = result.newPoly();
        line_poly.add(matrix.unapply(from));
        line_poly.add(matrix.unapply(to));
    }

    virtual void registerPolyStart(const Point& vertex) = 0;
    virtual void registerVertex(const Point& vertex) = 0;
    virtual void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even) = 0;
    virtual void registerPolyFinished() = 0;
};

class ZigzagConnectorProcessorNoEndPieces : public ZigzagConnectorProcessor
{
    std::vector<Point> first_zigzag_connector;
    std::vector<Point> zigzag_connector;

    bool is_first_zigzag_connector;
    bool first_zigzag_connector_ends_in_even_scanline;
    bool last_scanline_is_even; 

public:
    ZigzagConnectorProcessorNoEndPieces(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    , is_first_zigzag_connector(true)
    , first_zigzag_connector_ends_in_even_scanline(true)
    , last_scanline_is_even(false) 
    {
    }

    void registerPolyStart(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class ZigzagConnectorProcessorEndPieces : public ZigzagConnectorProcessor
{
protected:
    std::vector<Point> first_zigzag_connector;
    std::vector<Point> zigzag_connector_starting_in_uneven_scanline;

    Point last_connector_point;

    bool is_first_zigzag_connector;
    bool first_zigzag_connector_ends_in_even_scanline;
    bool last_scanline_is_even; 

public:
    ZigzagConnectorProcessorEndPieces(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    , last_connector_point(0,0)
    , is_first_zigzag_connector(true)
    , first_zigzag_connector_ends_in_even_scanline(true)
    , last_scanline_is_even(false) 
    {
    }

    void registerPolyStart(const Point& vertex);
    void registerVertex(const Point& vertex);
//     void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
//     void registerPolyFinished();
};


class ZigzagConnectorProcessorConnectedEndPieces : public ZigzagConnectorProcessorEndPieces
{
public:
    ZigzagConnectorProcessorConnectedEndPieces(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessorEndPieces(matrix, result)
    {
    }
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class ZigzagConnectorProcessorDisconnectedEndPieces : public ZigzagConnectorProcessorEndPieces
{

public:
    ZigzagConnectorProcessorDisconnectedEndPieces(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessorEndPieces(matrix, result)
    {
    }
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};

class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    NoZigZagConnectorProcessor(const PointMatrix& matrix, Polygons& result)
    : ZigzagConnectorProcessor(matrix, result)
    {
    }

    void registerPolyStart(const Point& vertex);
    void registerVertex(const Point& vertex);
    void registerScanlineSegmentIntersection(const Point& intersection, bool scanline_is_even);
    void registerPolyFinished();
};


} // namespace cura


#endif // ZIGZAG_CONNECTOR_PROCESSOR_H