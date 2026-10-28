// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "bridge/TransformedSegment.h"
#include "geometry/PointMatrix.h"

namespace curaengine::geometry
{
class Polygon;
class Shape;
} // namespace curaengine::geometry

namespace curaengine::bridge
{


class TransformedShape
{
public:
    /*!
     * Constructs an empty transformed shape
     * @warning This constructor does not assign the transformation matrix, so any subsequent call to
     */
    TransformedShape() = default;

    explicit TransformedShape(const geometry::PointMatrix& matrix);

    explicit TransformedShape(const geometry::Shape& shape, const geometry::PointMatrix& matrix);

    explicit TransformedShape(const geometry::Polygon& polygon, const geometry::PointMatrix& matrix);

    /*!
     * Adds a raw shape to the transformed shape
     * @param shape The raw shape to be transformed
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     */
    void addShape(const geometry::Shape& shape, const bool filter_out_horizontal = false);

    /*!
     * Adds a raw polygon to the transformed shape
     * @param polygon The raw polygon to be transformed
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     * @param reserve_size Indicates if the segments should be pre-reserved in memory space. This is usually what you want unless
     *                     this has been done very specifically.
     */
    void addPolygon(const geometry::Polygon& polygon, const bool filter_out_horizontal = false, const bool reserve_size = true);

    /*!
     * Adds a transformed segment to the transformed shape
     * @param start The start position of the raw segment
     * @param end The end position of the raw segment
     * @param filter_out_horizontal Indicates whether horizontal segments should be filtered out
     */
    void addSegment(const geometry::Point2LL& start, const geometry::Point2LL& end, const bool filter_out_horizontal = false);

    const std::vector<TransformedSegment>& getSegments() const
    {
        return segments_;
    }

    utils::coord_t minY() const
    {
        return min_y_;
    }

    utils::coord_t maxY() const
    {
        return max_y_;
    }

private:
    geometry::PointMatrix matrix_;
    std::vector<TransformedSegment> segments_;
    utils::coord_t min_y_{ std::numeric_limits<utils::coord_t>::max() };
    utils::coord_t max_y_{ std::numeric_limits<utils::coord_t>::lowest() };
};

} // namespace curaengine::bridge
