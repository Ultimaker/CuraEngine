//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
#include <stb_image.h>

#include "ImageBasedDensityProvider.h"
#include "SierpinskiFill.h"
#include "../utils/AABB3D.h"
#include "../utils/logoutput.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

ImageBasedDensityProvider::ImageBasedDensityProvider(const std::string filename, const AABB model_aabb)
{
    int desired_channel_count = 0; // keep original amount of channels
    int img_x, img_y, img_z; // stbi requires pointer to int rather than to coord_t
    image = stbi_load(filename.c_str(), &img_x, &img_y, &img_z, desired_channel_count);
    image_size = Point3(img_x, img_y, img_z);
    if (!image)
    {
        const char* reason = "[unknown reason]";
        if (stbi_failure_reason())
        {
            reason = stbi_failure_reason();
        }
        logError("Cannot load image %s: '%s'.\n", filename.c_str(), reason);
        std::exit(-1);
    }
    { // compute aabb
        Point middle = model_aabb.getMiddle();
        Point model_aabb_size = model_aabb.max - model_aabb.min;
        Point image_size2 = Point(image_size.x, image_size.y);
        float aabb_aspect_ratio = float(model_aabb_size.X) / float(model_aabb_size.Y);
        float image_aspect_ratio = float(image_size.x) / float(image_size.y);
        Point aabb_size;
        if (image_aspect_ratio < aabb_aspect_ratio)
        {
            aabb_size = image_size2 * model_aabb_size.X / image_size.x;
        }
        else
        {
            aabb_size = image_size2 * model_aabb_size.Y / image_size.y;
        }
        print_aabb = AABB(middle - aabb_size / 2, middle + aabb_size / 2);
        assert(aabb_size.X >= model_aabb_size.X && aabb_size.Y >= model_aabb_size.Y);
    }
}


ImageBasedDensityProvider::~ImageBasedDensityProvider()
{
    if (image)
    {
        stbi_image_free(image);
    }
}

float ImageBasedDensityProvider::operator()(const AABB3D& query_cube) const
{
    AABB query_box(Point(query_cube.min.x, query_cube.min.y), Point(query_cube.max.x, query_cube.max.y));
    Point img_min = (query_box.min - print_aabb.min - Point(1,1)) * image_size.x / (print_aabb.max.X - print_aabb.min.X);
    Point img_max = (query_box.max - print_aabb.min + Point(1,1)) * image_size.y / (print_aabb.max.Y - print_aabb.min.Y);
    long total_lightness = 0;
    int value_count = 0;
    for (int x = std::max((coord_t)0, img_min.X); x <= std::min((coord_t)image_size.x - 1, img_max.X); x++)
    {
        for (int y = std::max((coord_t)0, img_min.Y); y <= std::min((coord_t)image_size.y - 1, img_max.Y); y++)
        {
            for (int z = 0; z < image_size.z; z++)
            {
                total_lightness += image[((image_size.y - 1 - y) * image_size.x + x) * image_size.z + z];
                value_count++;
            }
        }
    }
    if (value_count == 0)
    { // triangle falls outside of image or in between pixels, so we return the closest pixel
        Point closest_pixel = (img_min + img_max) / 2;
        closest_pixel.X = std::max((coord_t)0, std::min((coord_t)image_size.x - 1, (coord_t)closest_pixel.X));
        closest_pixel.Y = std::max((coord_t)0, std::min((coord_t)image_size.y - 1, (coord_t)closest_pixel.Y));
        assert(total_lightness == 0);
        for (int z = 0; z < image_size.z; z++)
        {
            total_lightness += image[((image_size.y - 1 - closest_pixel.Y) * image_size.x + closest_pixel.X) * image_size.z + z];
            value_count++;
        }
    }
    return 1.0f - ((float)total_lightness) / value_count / 255.0f;
}

} // namespace cura
