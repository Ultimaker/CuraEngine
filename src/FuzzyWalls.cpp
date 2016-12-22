/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "FuzzyWalls.h"

namespace cura 
{

Polygons FuzzyWalls::makeFuzzy(const SliceMeshStorage& mesh, const unsigned int layer_nr, const Polygons& in)
{

    if (in.size() == 0)
    {
        return Polygons();
    }
    coord_t max_amplitude = mesh.getSettingInMicrons("magic_fuzzy_skin_thickness");
    coord_t avg_dist_between_points = mesh.getSettingInMicrons("magic_fuzzy_skin_point_dist");
    ColourUsage color_usage = mesh.getSettingAsColourUsage("fuzz_map_texture_color");
    coord_t min_dist_between_points = avg_dist_between_points * 3 / 4; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    coord_t range_random_point_dist = avg_dist_between_points / 2;
    std::function<coord_t (const unsigned int, const Point)> getAmplitude;
    if (mesh.getSettingBoolean("fuzz_map_enabled"))
    {
        assert(mesh.texture_proximity_processor && "texture_proximity_processor should have been initialized");
        getAmplitude = [&mesh, max_amplitude, color_usage](const unsigned int layer_nr, const Point p)
        {
            assert(mesh.texture_proximity_processor && "When fuzz_map_enabled there has to be a texture proximity processor!");
            TextureProximityProcessor& texture_proximity_processor = *mesh.texture_proximity_processor;
            float color = texture_proximity_processor.getColor(p, layer_nr, color_usage, 0.0); // TODO change default 0.0
            coord_t ret = color * max_amplitude;
            return ret;
        };
    }
    else
    {
        getAmplitude = [max_amplitude](const unsigned int layer_nr, const Point p)
        {
            return max_amplitude;
        };
    }
    
    //TODO
//     set the flow for each path relative so that it extgrudes as much as the normal line
    
    Polygons results;
    const Polygons& skin = in;
    for (const PolygonRef poly : const_cast<Polygons&>(skin))
    {
        // generate points in between p0 and p1
        PolygonRef result = results.newPoly();

        int64_t dist_left_over = rand() % (min_dist_between_points / 2); // the distance to be traversed on the line before making the first new point
        const Point* p0 = &poly.back();
        for (const Point& p1 : poly)
        { // 'a' is the (next) new point between p0 and p1
            Point p0p1 = p1 - *p0;
            int64_t p0p1_size = vSize(p0p1);    
            int64_t dist_last_point = dist_left_over + p0p1_size * 2; // so that p0p1_size - dist_last_point evaulates to dist_left_over - p0p1_size
            for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += min_dist_between_points + rand() % range_random_point_dist)
            {
                Point perp_to_p0p1 = turn90CCW(p0p1);
                Point px = *p0 + normal(p0p1, p0pa_dist);
                coord_t fuzziness = getAmplitude(layer_nr, px);
                if (fuzziness == 0)
                {
                    fuzziness = 1;
                }
                int r = rand() % (fuzziness * 2) - fuzziness;
                Point fuzz = normal(perp_to_p0p1, r);
                Point pa = px + fuzz;
                result.add(pa);
                dist_last_point = p0pa_dist;
            }
            dist_left_over = p0p1_size - dist_last_point;

            p0 = &p1;
        }
        while (result.size() < 3 )
        {
            unsigned int point_idx = poly.size() - 2;
            result.add(poly[point_idx]);
            if (point_idx == 0)
            {
                break;
            }
            point_idx--;
        }
        if (result.size() < 3)
        {
            result.clear();
            for (const Point& p : poly)
            {
                result.add(p);
            }
        }
    }
    return results;
}


}//namespace cura