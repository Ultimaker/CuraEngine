// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/IDFieldInfo.h"

#include <algorithm>
#include <array>
#include <tuple>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>

#include "utils/AABB3D.h"

using namespace cura;

double proj(const Point3D& b, const Point3LL& p)
{
    return (Point3D(p) * b) / (b * b);
}

// FIMXE: something doesn't quite work right yet -- the normal is OK (I think?) but the primary and secondary axii don't do what I want yet...

std::optional<IdFieldInfo> IdFieldInfo::fromPointCloud(const std::vector<Point3LL>& points)
{
    if (points.size() < 3)
    {
        return std::nullopt;
    }

    // instead of doing full PCA (which is more efficient probably), for now just try to find the 'flattest' of 13 orientations (full grid: 9 * 3 = 27, then: (27 - 1) / 2 = 13 to
    // get rid of the origin and mirrored directions)
    constexpr double D2 = 0.70710678118654752440084436210485;
    constexpr double D3 = 0.57735026918962576450914878050196;
    static std::vector<Point3D> normal_candidates = std::vector<Point3D>({
        Point3D(0, 0, 1),
        Point3D(0, 1, 0),
        Point3D(1, 0, 0),
        Point3D(0, D2, D2),
        Point3D(0, D2, -D2),
        Point3D(D2, 0, D2),
        Point3D(D2, 0, -D2),
        Point3D(D2, D2, 0),
        Point3D(D2, -D2, 0),
        Point3D(D3, D3, D3),
        Point3D(D3, D3, -D3),
        Point3D(D3, -D3, D3),
        Point3D(D3, -D3, -D3),
    });

    std::unordered_map<int, span_t> remapped_spans;
    for (const auto& [idx, norm] : normal_candidates | ranges::views::enumerate)
    {
        auto& [remapped_min, remapped_max] = (remapped_spans[idx] = { std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() });
        for (const auto& pt : points)
        {
            const auto val = proj(norm, pt);
            remapped_min = std::min(remapped_min, val);
            remapped_max = std::max(remapped_max, val);
        }
    }

    auto span_sizes = remapped_spans
                    | ranges::view::transform(
                          [](const auto& key_value)
                          {
                              return std::make_pair(key_value.second.second - key_value.second.first, key_value.first);
                          })
                    | ranges::to<std::vector<std::pair<double, int>>>();
    std::stable_sort(
        span_sizes.begin(),
        span_sizes.end(),
        [](const auto& a, const auto& b)
        {
            return a.first < b.first;
        });

    const auto& normal = normal_candidates[span_sizes.front().second];
    const auto& secondary = std::find_if(
        normal_candidates.begin(),
        normal_candidates.end(),
        [&normal](const auto& ax)
        {
            return normal.cross(ax).vSize() > CLOSE_1D;
        });
    if (secondary == normal_candidates.end() || *secondary == normal)
    {
        return std::nullopt;
    }
    const auto& primary = std::find_if(
        normal_candidates.begin(),
        normal_candidates.end(),
        [&normal, &secondary](const auto& ax)
        {
            return (*secondary).cross(ax).vSize() > CLOSE_1D && normal.cross(ax).vSize() > CLOSE_1D;
        });
    if (primary == normal_candidates.end() || *primary == normal || primary == secondary)
    {
        return std::nullopt;
    }
    const int primary_idx = std::distance(normal_candidates.begin(), primary);
    const int secondary_idx = std::distance(normal_candidates.begin(), secondary);
    return std::make_optional(IdFieldInfo{ .primary_axis_ = *primary,
                                           .primary_span_ = remapped_spans[primary_idx],
                                           .secondary_axis_ = *secondary,
                                           .secondary_span_ = remapped_spans[secondary_idx],
                                           .normal_ = normal });
}

float mirrorNegative(const float in)
{
    return in < 0.0f ? 1.0 + in : in;
}

Point2F IdFieldInfo::worldPointToLabelUv(const Point3LL& pt) const
{
    // FOXME: (but should make smaller? -> get the '''furthest''' points in the '''diagonal''' directions of the chosen primary and secondary axii, should approximate well enough)

    return Point2F(
        1.0f - (proj(primary_axis_, pt) - primary_span_.first) / (primary_span_.second - primary_span_.first),
        1.0f - (proj(secondary_axis_, pt) - secondary_span_.first) / (secondary_span_.second - secondary_span_.first));
}
