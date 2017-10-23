/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "ImageBasedSubdivider.h"

#include "SierpinskiFill.h"

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
#include "stb/stb_image.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

ImageBasedSubdivider::ImageBasedSubdivider(const std::string filename, const AABB model_aabb, const coord_t line_width)
: line_width(line_width)
{
    int desired_channel_count = 0; // keep original amount of channels
    image = stbi_load(filename.c_str(), &image_size.x, &image_size.y, &image_size.z, desired_channel_count);
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
        aabb = AABB(middle - aabb_size / 2, middle + aabb_size / 2);
        assert(aabb_size.X >= model_aabb_size.X && aabb_size.Y >= model_aabb_size.Y);
    }
}


ImageBasedSubdivider::~ImageBasedSubdivider()
{
    if (image)
    {
        stbi_image_free(image);
    }
}

bool ImageBasedSubdivider::operator()(const SierpinskiFillEdge& e1, const SierpinskiFillEdge& e2) const
{
    AABB aabb_here;
    aabb_here.include(e1.l);
    aabb_here.include(e1.r);
    aabb_here.include(e2.l);
    aabb_here.include(e2.r);
    Point min = (aabb_here.min - aabb.min - Point(1,1)) * image_size.x / (aabb.max.X - aabb.min.X);
    Point max = (aabb_here.max - aabb.min + Point(1,1)) * image_size.y / (aabb.max.Y - aabb.min.Y);
    long total_lightness = 0;
    int value_count = 0;
    for (int x = std::max((coord_t)0, min.X); x <= std::min((coord_t)image_size.x - 1, max.X); x++)
    {
        for (int y = std::max((coord_t)0, min.Y); y <= std::min((coord_t)image_size.y - 1, max.Y); y++)
        {
            for (int z = 0; z < image_size.z; z++)
            {
                total_lightness += image[((image_size.y - 1 - y) * image_size.x + x) * image_size.z + z];
                value_count++;
            }
        }
    }
    const coord_t average_length = vSize((e1.l + e1.r) - (e2.l + e2.r)) / 2;
    // calculate area of triangle: base times height * .5
    coord_t area;
    {
        const coord_t base_length = vSize(e1.l - e1.r);
        const Point v1 = e1.r - e1.l;
        const Point v2 = e2.r - e2.l;
        const Point height_vector = dot(v2, turn90CCW(v1)) / vSize(v1);
        const coord_t height = vSize(height_vector);
        area = base_length * height / 2;
    }
    if (value_count == 0)
    { // triangle falls outside of image, so we subdivide it so that it cannot limit the subdivision of other cells
        return true;
    }
    if (255 - total_lightness / value_count > 255 * average_length * line_width / area)
    {
        return true;
    }
    return false;
};

}; // namespace cura
