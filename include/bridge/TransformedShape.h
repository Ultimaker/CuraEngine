// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_TRANSFORMEDSHAPE_H
#define BRIDGE_TRANSFORMEDSHAPE_H

#include "bridge/TransformedSegment.h"
#include "geometry/PointMatrix.h"

namespace cura
{

class Polygon;
class Shape;

class TransformedShape
{
public:
    /*!
     * Constructs an empty transformed shape
     * @warning This constructor does not assign the transformation matrix, so any subsequent call to
     */
    TransformedShape() = default;

    explicit TransformedShape(const PointMatrix& matrix);

    explicit TransformedShape(const Shape& shape, const PointMatrix& matrix);

    explicit TransformedShape(const Polygon& polygon, const PointMatrix& matrix);

    /*!
     * Adds a raw shape to the transformed shape
     * @param shape The raw shape to be transformed
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     */
    void addShape(const Shape& shape, const bool filter_out_horizontal = false);

    /*!
     * Adds a raw polygon to the transformed shape
     * @param polygon The raw polygon to be transformed
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     * @param reserve_size Indicates if the segments should be pre-reserved in memory space. This is usually what you want unless
     *                     this has been done very specifically.
     */
    void addPolygon(const Polygon& polygon, const bool filter_out_horizontal = false, const bool reserve_size = true);

    /*!
     * Adds a transformed segment to the transformed shape
     * @param start The start position of the raw segment
     * @param end The end position of the raw segment
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     */
    void addSegment(const Point2LL& start, const Point2LL& end, const bool filter_out_horizontal = false);

    const std::vector<TransformedSegment>& getSegments() const
    {
        return segments_;
    }

    coord_t minY() const
    {
        return min_y_;
    }

    coord_t maxY() const
    {
        return max_y_;
    }

private:
    PointMatrix matrix_;
    std::vector<TransformedSegment> segments_;
    coord_t min_y_{ std::numeric_limits<coord_t>::max() };
    coord_t max_y_{ std::numeric_limits<coord_t>::lowest() };
};

} // namespace cura

#endif
