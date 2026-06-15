// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_SEGMENT3LL_H
#define UTILS_SEGMENT3LL_H

#include <optional>

#include "geometry/Point3LL.h"


namespace cura
{

/*!
 * This represents a segment in 3D space
 */
class Segment3LL
{
public:
    Segment3LL(const Point3LL& start, const Point3LL& end);

    const Point3LL& start() const
    {
        return start_;
    }

    const Point3LL& end() const
    {
        return end_;
    }

    void setEnd(const Point3LL& end)
    {
        end_ = end;
    }

    /*!
     * Intersects the segment with a "layer" on the X plane, e.g. a pair or YZ plans
     * @param layer_start The X coordinate of the lowest YZ plane
     * @param layer_end The X coordinate of the highest YZ plane
     * @return The segment that is a sub-part of the current segment intersected with the layer, or nullopt if the intersection is empty
     */
    std::optional<Segment3LL> intersectionWithXLayer(const coord_t layer_start, const coord_t layer_end) const;

    /*!
     * Intersects the segment with a "layer" on the Y plane, e.g. a pair or XZ plans
     * @param layer_start The Y coordinate of the lowest XZ plane
     * @param layer_end The Y coordinate of the highest XZ plane
     * @return The segment that is a sub-part of the current segment intersected with the layer, or nullopt if the intersection is empty
     */
    std::optional<Segment3LL> intersectionWithYLayer(const coord_t layer_start, const coord_t layer_end) const;

    /*!
     * Intersects the segment with a "layer" on the Z plane, e.g. a pair or XY plans
     * @param layer_start The Z coordinate of the lowest XY plane
     * @param layer_end The Z coordinate of the highest XY plane
     * @return The segment that is a sub-part of the current segment intersected with the layer, or nullopt if the intersection is empty
     */
    std::optional<Segment3LL> intersectionWithZLayer(const coord_t layer_start, const coord_t layer_end) const;

private:
    enum class LayerLocation
    {
        Below,
        Inside,
        Above
    };

    /*!
     * Get the point on the segment that is at the given X coordinate. If the X coordinate doesn't fit in the segment, it is treated as an infinite line and the returned point
     * will be outside the segment
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the X axis. It will crash otherwise.
     */
    Point3LL pointAtX(const coord_t x) const;

    /*!
     * Get the point on the segment that is at the given Y coordinate. If the Y coordinate doesn't fit in the segment, it is treated as an infinite line and the returned point
     * will be outside the segment
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the Y axis. It will crash otherwise.
     */
    Point3LL pointAtY(const coord_t y) const;

    /*!
     * Get the point on the segment that is at the given Z coordinate. If the Z coordinate doesn't fit in the segment, it is treated as an infinite line and the returned point
     * will be outside the segment
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the Z axis. It will crash otherwise.
     */
    Point3LL pointAtZ(const coord_t z) const;

    /*!
     * Intersects the segment with a "layer" on an axis-aligned plane
     * @param start_coordinate The segment start coordinate on the relevant axis
     * @param end_coordinate The segment end coordinate on the relevant axis
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @param function_crop_point The function to be used to crop the segment on the proper axis
     * @return The segment that is a sub-part of the current segment intersected with the layer, or nullopt if the intersection is empty
     */
    std::optional<Segment3LL> intersectionWithLayer(
        const coord_t start_coordinate,
        const coord_t end_coordinate,
        const coord_t layer_start,
        const coord_t layer_end,
        const std::function<Point3LL(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end)>& function_crop_point) const;

    /*!
     * Crops an extremity of the segment so that it ends inside the given layer
     * @param point The extremity point to be cropped
     * @param insideness Indicates whether the point is below, inside or above the layer
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @param function_point_at The function to be used to get the new point given the proper axis
     * @return A point on the segment that is inside the layer
     */
    static Point3LL croppedPoint(
        const Point3LL& point,
        const LayerLocation insideness,
        const coord_t layer_start,
        const coord_t layer_end,
        const std::function<Point3LL(const coord_t)>& function_point_at);

    /*!
     * Crops an extremity of the segment so that it ends inside the given X-aligned layer
     * @param point The extremity point to be cropped
     * @param insideness Indicates whether the point is below, inside or above the layer
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @return A point on the segment that is inside the layer
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the X axis. It will crash otherwise.
     */
    Point3LL croppedPointX(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const;

    /*!
     * Crops an extremity of the segment so that it ends inside the given Y-aligned layer
     * @param point The extremity point to be cropped
     * @param insideness Indicates whether the point is below, inside or above the layer
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @return A point on the segment that is inside the layer
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the Y axis. It will crash otherwise.
     */
    Point3LL croppedPointY(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const;

    /*!
     * Crops an extremity of the segment so that it ends inside the given Z-aligned layer
     * @param point The extremity point to be cropped
     * @param insideness Indicates whether the point is below, inside or above the layer
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @return A point on the segment that is inside the layer
     * @warning Since this function is private, it does not perform any check and assume the segment is not parallel to the Z axis. It will crash otherwise.
     */
    Point3LL croppedPointZ(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const;

    /*!
     * Calculates the insideness of a point in regard to an axis-aligned layer
     * @param point The point coordinate on the relevant axis
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @return Whether the point is below, inside or above the layer
     */
    static LayerLocation pointIsInside(const coord_t point, const coord_t layer_start, const coord_t layer_end);

private:
    Point3LL start_;
    Point3LL end_;
};

} // namespace cura

#endif