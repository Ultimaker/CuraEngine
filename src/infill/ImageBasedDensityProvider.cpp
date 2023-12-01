// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
#include "infill/ImageBasedDensityProvider.h"

#include <stb_image.h>

#include <spdlog/spdlog.h>

#include "infill/SierpinskiFill.h"
#include "utils/AABB3D.h"

namespace cura
{

static constexpr bool diagonal = true;
static constexpr bool straight = false;

ImageBasedDensityProvider::ImageBasedDensityProvider(const std::string filename, const AABB model_aabb)
{
    int desired_channel_count = 0; // keep original amount of channels
    int img_x, img_y, img_z; // stbi requires pointer to int rather than to coord_t
    image = stbi_load(filename.c_str(), &img_x, &img_y, &img_z, desired_channel_count);
    image_size = Point3LL(img_x, img_y, img_z);
    if (! image)
    {
        const char* reason = "[unknown reason]";
        if (stbi_failure_reason())
        {
            reason = stbi_failure_reason();
        }
        spdlog::error("Cannot load image {}: {}", filename, reason);
        std::exit(-1);
    }
    { // compute aabb
        Point2LL middle = model_aabb.getMiddle();
        Point2LL model_aabb_size = model_aabb.max_ - model_aabb.min_;
        Point2LL image_size2 = Point2LL(image_size.x_, image_size.y_);
        double aabb_aspect_ratio = double(model_aabb_size.X) / double(model_aabb_size.Y);
        double image_aspect_ratio = double(image_size.x_) / double(image_size.y_);
        Point2LL aabb_size;
        if (image_aspect_ratio < aabb_aspect_ratio)
        {
            aabb_size = image_size2 * model_aabb_size.X / image_size.x_;
        }
        else
        {
            aabb_size = image_size2 * model_aabb_size.Y / image_size.y_;
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

double ImageBasedDensityProvider::operator()(const AABB3D& query_cube) const
{
    AABB query_box(Point2LL(query_cube.min_.x_, query_cube.min_.y_), Point2LL(query_cube.max_.x_, query_cube.max_.y_));
    Point2LL img_min = (query_box.min_ - print_aabb.min_ - Point2LL(1, 1)) * image_size.x_ / (print_aabb.max_.X - print_aabb.min_.X);
    Point2LL img_max = (query_box.max_ - print_aabb.min_ + Point2LL(1, 1)) * image_size.y_ / (print_aabb.max_.Y - print_aabb.min_.Y);
    long total_lightness = 0;
    int value_count = 0;
    for (int x = std::max((coord_t)0, img_min.X); x <= std::min((coord_t)image_size.x_ - 1, img_max.X); x++)
    {
        for (int y = std::max((coord_t)0, img_min.Y); y <= std::min((coord_t)image_size.y_ - 1, img_max.Y); y++)
        {
            for (int z = 0; z < image_size.z_; z++)
            {
                total_lightness += image[((image_size.y_ - 1 - y) * image_size.x_ + x) * image_size.z_ + z];
                value_count++;
            }
        }
    }
    if (value_count == 0)
    { // triangle falls outside of image or in between pixels, so we return the closest pixel
        Point2LL closest_pixel = (img_min + img_max) / 2;
        closest_pixel.X = std::max((coord_t)0, std::min((coord_t)image_size.x_ - 1, (coord_t)closest_pixel.X));
        closest_pixel.Y = std::max((coord_t)0, std::min((coord_t)image_size.y_ - 1, (coord_t)closest_pixel.Y));
        assert(total_lightness == 0);
        for (int z = 0; z < image_size.z_; z++)
        {
            total_lightness += image[((image_size.y_ - 1 - closest_pixel.Y) * image_size.x_ + closest_pixel.X) * image_size.z_ + z];
            value_count++;
        }
    }
    return 1.0 - ((double)total_lightness) / value_count / 255.0;
}

} // namespace cura
