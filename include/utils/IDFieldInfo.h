// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef ID_FIELD_INFO_H
#define ID_FIELD_INFO_H

#include <optional>

#include "geometry/Point3LL.h"
#include "utils/AABB.h"
#include "utils/Point2F.h"
#include "utils/Point3D.h"

namespace cura
{
const double CLOSE_1D = std::nextafter(1.0, 0.0);
const float CLOSE_1F = std::nextafter(1.0f, 0.0f);
// NOTE: nextafter isn't constexpr yet in c++20, replace with constexpr when we do C++23

struct IdFieldInfo
{
    typedef std::pair<double, double> span_t;

    Point3D primary_axis_;
    span_t primary_span_;

    Point3D secondary_axis_;
    span_t secondary_span_;

    Point3D normal_;

public:
    static std::optional<IdFieldInfo> fromPointCloud(const std::vector<Point3LL>& points);

    Point2F worldPointToLabelUv(const Point3LL& pt) const;
};
} // namespace cura

#endif // ID_FIELD_INFO_H
