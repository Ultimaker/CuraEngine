// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_PARAMETERIZEDSEGMENT_H
#define UTILS_PARAMETERIZEDSEGMENT_H

#include <optional>

#include "utils/Point3D.h"


namespace cura
{

/*!
 * The ParameterizedSegment is a helper to quickly calculate the intersections of a segment with the X and Y planes.
 */
class ParameterizedSegment
{
public:
    ParameterizedSegment(const Point3D& start, const Point3D& end);

    const Point3D& start() const
    {
        return start_;
    }

    const Point3D& end() const
    {
        return end_;
    }

    std::optional<ParameterizedSegment> intersectionWithXLayer(const double layer_start, const double layer_end) const;

    std::optional<ParameterizedSegment> intersectionWithYLayer(const double layer_start, const double layer_end) const;

private:
    Point3D pointAtX(const double x) const;

    Point3D pointAtY(const double y) const;

    std::optional<ParameterizedSegment> croppedSegmentX(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const;

    std::optional<ParameterizedSegment> croppedSegmentY(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const;

private:
    Point3D direction_;
    Point3D start_;
    Point3D end_;
};

} // namespace cura

#endif