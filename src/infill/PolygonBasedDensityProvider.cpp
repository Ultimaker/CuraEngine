/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "PolygonBasedDensityProvider.h"

#include "../utils/AABB.h"
#include "../sliceDataStorage.h"

namespace cura {

constexpr double min_area_error = 100.0; // 10x10 micron

constexpr size_t max_density_steps = 20;

PolygonBasedDensityProvider::PolygonBasedDensityProvider(const SliceLayer& layer, float most_dense_density)
: fallback_density(0.0)
{
    for (size_t density_idx = 0; density_idx < max_density_steps; density_idx++)
    {
        for (const SliceLayerPart& part : layer.parts)
        {
            if (density_idx < part.infill_area_per_combine_per_density.size())
            {
                for (const Polygons& infill_area : part.infill_area_per_combine_per_density[density_idx])
                { // for each thickness
                    polygon_densities.emplace_back(&infill_area, most_dense_density * (1 << density_idx));
                }
            }
        }
    }
}

float PolygonBasedDensityProvider::operator()(const AABB& aabb) const
{
    Polygon aabb_polygon = aabb.toPolygon();
    Polygons aabb_polygons;
    aabb_polygons.add(aabb_polygon);
    double aabb_area = aabb_polygon.area();

    float average_density = 0.0;
    size_t overlapping_polygon_count = 0;
    double total_overlap_area = 0.0;
    for (PolygonDensity polygon_density : polygon_densities)
    {
        const Polygons& area = *polygon_density.area;
        Polygons intersection = area.intersection(aabb_polygons);
        if (!intersection.empty())
        {
            overlapping_polygon_count++;
            double intersection_area = intersection.area();
            if (std::abs(intersection_area - aabb_area) < min_area_error)
            {
                return polygon_density.density;
            }
            average_density += intersection_area * polygon_density.density;
            total_overlap_area += intersection_area;
        }
    }

    if (overlapping_polygon_count == 0)
    {
        return fallback_density;
    }

    return (average_density + (aabb_area - total_overlap_area) * fallback_density) / aabb_area;
}

}; // namespace cura
