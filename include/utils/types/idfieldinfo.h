// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef ID_FIELD_INFO_H
#define ID_FIELD_INFO_H

#include <optional>

#include "utils/AABB3D.h"
#include "utils/Point2F.h"

namespace cura
{
// FIXME?: This now goes per-axis, because that's what we get by creating it from the AABB, but it should be a 'free' plane-normal(s).
//         We'd probably need to do a little principal component analysis to _properly_ get the primary and secondary axii.

struct IdFieldInfo
{
    enum class Axis
    {
        X,
        Y,
        Z
    };
    static coord_t getAxisValue(const Axis ax, const Point3LL& pt)
    {
        switch (ax)
        {
        case Axis::X:
            return pt.x_;
        case Axis::Y:
            return pt.y_;
        case Axis::Z:
            return pt.z_;
        default:
            return 0;
        }
    }

    static float mirrorNegative(const float in)
    {
        return in < 0.0f ? 1.0 + in : in;
    }

    Axis primary_axis_ = Axis::X;
    Axis secondary_axis_ = Axis::Z;
    Axis normal_ = Axis::Y;
    AABB projection_field_;

public:
    static std::optional<IdFieldInfo> fromAabb3d(const AABB3D& aabb);

    Point2F worldPointToLabelUv(const Point3LL& pt) const
    {
        return Point2F(
            1.0f - mirrorNegative(static_cast<float>(getAxisValue(primary_axis_, pt) - projection_field_.min_.X) / (projection_field_.max_.X - projection_field_.min_.X)),
            1.0f - mirrorNegative(static_cast<float>(getAxisValue(secondary_axis_, pt) - projection_field_.min_.Y) / (projection_field_.max_.Y - projection_field_.min_.Y)));
    }
};
} // namespace cura

#endif // ID_FIELD_INFO_H
