/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "ImageBasedDensityProvider.h"

#include <fstream>
#include <string>
#include <sstream>

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
#include <stb/stb_image.h>

#include "../utils/logoutput.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

ImageBasedDensityProvider::ImageBasedDensityProvider(const std::string filename, const AABB3D model_aabb)
{
    std::string last_file = filename;
    std::string file_now = filename;
    bool is_first_image = true;
    while (std::ifstream(file_now.c_str()).good())
    {
        loadImage(file_now, is_first_image);
        last_file = file_now;
        is_first_image = false;
        file_now = advanceFilename(file_now);
    }
    grid_size = image_size;
    grid_size.z = images.size();
    logDebug("Found %d images. Last one is called '%s'.\n", grid_size.z, last_file.c_str());
    { // compute aabb
        Point3 middle = model_aabb.getMiddle();
        Point3 model_aabb_size = model_aabb.max - model_aabb.min;
        float aabb_aspect_ratio = float(model_aabb_size.x) / float(model_aabb_size.y);
        float image_aspect_ratio = float(image_size.x) / float(image_size.y);
        Point3 aabb_size;
        if (image_aspect_ratio < aabb_aspect_ratio)
        {
            aabb_size = grid_size * model_aabb_size.x / image_size.x;
        }
        else
        {
            aabb_size = grid_size * model_aabb_size.y / image_size.y;
        }
        aabb_size.z = model_aabb.size().z; // z doesn't scale. voxel model is always stretched over the hwole model Z
        print_aabb = AABB3D(middle - aabb_size / 2, middle + aabb_size / 2);
        assert(aabb_size.x >= model_aabb_size.x && aabb_size.y >= model_aabb_size.y && "print_aabb is as least as big as model_aabb");
    }
}

void ImageBasedDensityProvider::loadImage(const std::string filename, const bool set_size)
{
    int desired_channel_count = 0; // keep original amount of channels
    int img_x, img_y, img_z; // stbi requires pointer to int rather than to coord_t
    unsigned char* image = stbi_load(filename.c_str(), &img_x, &img_y, &img_z, desired_channel_count);
    Point3 image_size_here = Point3(img_x, img_y, img_z);
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
    images.push_back(image);
    if (set_size)
    {
        image_size = image_size_here;
    }
    else if (image_size != image_size_here)
    {
        logError("Image sequence is not of consistent size! Previous image size: (%d, %d, %d). New image size: (%d, %d, %d). For image '%s'.\n", image_size.x, image_size.y, image_size.z, image_size_here.x, image_size_here.y, image_size_here.z, filename.c_str());
        std::exit(-1);
    }
}

std::string ImageBasedDensityProvider::advanceFilename(const std::string& filename)
{
    // get filename without extension
    const size_t last_dot_idx = filename.find_last_of(".");
    if (last_dot_idx == std::string::npos) return "";
    const std::string raw_name = filename.substr(0, last_dot_idx);

    // extract number from file name
    size_t last_non_digit_idx = raw_name.find_last_not_of("0123456789");
    if (last_non_digit_idx == std::string::npos)
    {
        last_non_digit_idx = -1; // first character in the name is already a digit
    }
    if (last_non_digit_idx == raw_name.length() - 1) return "";
    const std::string file_number_str = raw_name.substr(last_non_digit_idx + 1);
    const int file_number = std::stoi(file_number_str);

    for (uint_fast8_t skip_idx = 1; skip_idx < 100; skip_idx++)
    {
        std::ostringstream oss;
        oss << raw_name.substr(0, last_non_digit_idx + 1) << (file_number + skip_idx) << filename.substr(last_dot_idx);
        std::string next = oss.str();
        if (std::ifstream(next.c_str()).good())
        {
            return next;
        }
    }
    return ""; // no next file was found in the next 100 in the sequence.
}

ImageBasedDensityProvider::~ImageBasedDensityProvider()
{
    for (unsigned char* image : images)
    {
        if (image)
        {
            stbi_image_free(image);
        }
    }
}

float ImageBasedDensityProvider::operator()(const AABB3D& query_cube) const
{
    Point3 print_aabb_size = print_aabb.size();
    Point3 img_min = (query_cube.min - print_aabb.min - Point3(1,1,1)) * grid_size.x / print_aabb_size.x;
    Point3 img_max = (query_cube.max - print_aabb.min) * grid_size.y / print_aabb_size.y + Point3(1,1,1);
    img_min.z = (query_cube.min.z - print_aabb.min.z - 1) * grid_size.z / print_aabb_size.z;
    img_max.z = (query_cube.max.z - print_aabb.min.z) * grid_size.z / print_aabb_size.z + 1;

    auto grid_to_print_coord = [this, print_aabb_size](const Point3 grid_loc)
        {
            return Point3(
                print_aabb.min.x + grid_loc.x * print_aabb_size.x / grid_size.x,
                print_aabb.min.y + grid_loc.y * print_aabb_size.y / grid_size.y,
                print_aabb.min.z + grid_loc.z * print_aabb_size.z / grid_size.z
                   );
        };

    double total_weighted_lightness = 0.0;
    double total_volume = 0.0;
    for (int z = std::max(static_cast<coord_t>(0), img_min.z); z <= std::min(grid_size.z - 1, img_max.z); z++)
    {
        const unsigned char* image = images[z];
        assert(image);
        for (int x = std::max(static_cast<coord_t>(0), img_min.x); x <= std::min(grid_size.x - 1, img_max.x); x++)
        {
            for (int y = std::max(static_cast<coord_t>(0), img_min.y); y <= std::min(grid_size.y - 1, img_max.y); y++)
            {
                const AABB3D voxel_cube(grid_to_print_coord(Point3(x, y, z)), grid_to_print_coord(Point3(x + 1, y + 1, z + 1)));
                const double voxel_volume = voxel_cube.volumeMM3();
                const double scanned_voxel_volume = voxel_cube.intersect(query_cube).volumeMM3();
                for (int channel = 0; channel < image_size.z; channel++)
                {
                    const unsigned char lightness = image[((grid_size.y - 1 - y) * grid_size.x + x) * image_size.z + channel];
                    double scanned_voxel_ratio = std::max(0.0, std::min(1.0, scanned_voxel_volume / voxel_volume));
                    total_weighted_lightness += static_cast<double>(lightness) * scanned_voxel_ratio;
                    total_volume += scanned_voxel_ratio;
                }
            }
        }
    }
    assert(total_weighted_lightness >= 0.0);
    assert(total_volume >= 0.0);
    
    if (total_volume == 0.0)
    { // triangle falls outside of image or in between pixels, so we return the closest pixel
        Point3 closest_pixel = (img_min + img_max) / 2;
        closest_pixel.x = std::max(static_cast<coord_t>(0), std::min(grid_size.x - 1, closest_pixel.x));
        closest_pixel.y = std::max(static_cast<coord_t>(0), std::min(grid_size.y - 1, closest_pixel.y));
        closest_pixel.z = std::max(static_cast<coord_t>(0), std::min(grid_size.z - 1, closest_pixel.z));
        const unsigned char* image = images[closest_pixel.z];
        assert(total_weighted_lightness == 0.0);
        for (int channel = 0; channel < image_size.z; channel++)
        {
            total_weighted_lightness += image[((grid_size.y - 1 - closest_pixel.y) * grid_size.x + closest_pixel.x) * image_size.z + channel];
            total_volume += 1.0;
        }
        assert(total_weighted_lightness > 0);
        assert(total_volume > 0);
    }
    float ret = 1.0f - (total_weighted_lightness / total_volume / 255.0f);
    assert(ret >= 0.0f);
    assert(ret <= 1.0f);
    return ret;
};

}; // namespace cura
