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
    if (grid_size.z == 0)
    {
        logError("Couldn't find density image '%s'.\n", filename.c_str());
        std::exit(-1);
        // would otherwise in a division by zero below, because image_size.x = 0
    }
    if (grid_size.x == 0 || grid_size.y == 0)
    {
        logError("Density image is empty: '%s'.\n", filename.c_str());
        std::exit(-1);
        // would otherwise in a division by zero below
    }
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
    int img_x = 0, img_y = 0, img_z = 0; // stbi requires pointer to int rather than to coord_t
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
    assert(file_number_str.length() > 0);
    const int file_number = std::stoi(file_number_str);

    bool filename_has_padding = file_number_str.c_str()[0] == '0';

    for (uint_fast8_t skip_idx = 1; skip_idx < 100; skip_idx++)
    {
        std::ostringstream oss;
        oss << raw_name.substr(0, last_non_digit_idx + 1);
        char next_file_number_str[100];
        int n_chars_written = sprintf(next_file_number_str, "%d", file_number + skip_idx);
        if (filename_has_padding)
        {
            for (uint_fast8_t zero_number = 0; zero_number < file_number_str.length() - n_chars_written; zero_number++)
            { // pad filename with zeroes
                oss << '0';
            }
        }
        oss << next_file_number_str << filename.substr(last_dot_idx);
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

float ImageBasedDensityProvider::operator()(const AABB3D& query_cube, const int_fast8_t averaging_statistic) const
{
    Point3 print_aabb_size = print_aabb.size();
    Point3 img_min = (query_cube.min - print_aabb.min - Point3(1,1,1)) * grid_size.x / print_aabb_size.x;
    Point3 img_max = (query_cube.max - print_aabb.min) * grid_size.y / print_aabb_size.y + Point3(1,1,1);
    img_min.z = (query_cube.min.z - print_aabb.min.z - 1) * grid_size.z / print_aabb_size.z;
    img_max.z = (query_cube.max.z - print_aabb.min.z) * grid_size.z / print_aabb_size.z + 1;

    uint_fast64_t total_lightness = 0;
    if (averaging_statistic < 0)
    {
        total_lightness = std::numeric_limits<uint_fast64_t>::max();
    }
    uint_fast32_t cell_count = 0;
    for (int z = std::max(static_cast<coord_t>(0), img_min.z); z <= std::min(grid_size.z - 1, img_max.z); z++)
    {
        const unsigned char* image = images[z];
        assert(image);
        for (int x = std::max(static_cast<coord_t>(0), img_min.x); x <= std::min(grid_size.x - 1, img_max.x); x++)
        {
            for (int y = std::max(static_cast<coord_t>(0), img_min.y); y <= std::min(grid_size.y - 1, img_max.y); y++)
            {
                uint_fast64_t combined_lightness_here = 0;
                for (int channel = 0; channel < image_size.z; channel++)
                {
                    const unsigned char lightness = image[((grid_size.y - 1 - y) * grid_size.x + x) * image_size.z + channel];
                    combined_lightness_here += lightness;
                    cell_count++;
                }
                if (averaging_statistic == 0)
                {
                    total_lightness += combined_lightness_here;
                }
                else if (averaging_statistic < 0)
                {
                    total_lightness = std::min(total_lightness, combined_lightness_here / static_cast<uint_fast64_t>(image_size.z));
                }
                else
                {
                    total_lightness = std::max(total_lightness, combined_lightness_here / static_cast<uint_fast64_t>(image_size.z));
                }
            }
        }
    }
    
    if (cell_count == 0)
    { // cube falls outside of image or in between pixels, so we return the closest pixel
        assert(total_lightness == 0.0 || (averaging_statistic < 0 && total_lightness == std::numeric_limits<uint_fast64_t>::max()));
        Point3 closest_pixel = (img_min + img_max) / 2;
        closest_pixel.x = std::max(static_cast<coord_t>(0), std::min(grid_size.x - 1, closest_pixel.x));
        closest_pixel.y = std::max(static_cast<coord_t>(0), std::min(grid_size.y - 1, closest_pixel.y));
        closest_pixel.z = std::max(static_cast<coord_t>(0), std::min(grid_size.z - 1, closest_pixel.z));
        const unsigned char* image = images[closest_pixel.z];
        uint_fast64_t combined_lightness_here = 0;
        for (int channel = 0; channel < image_size.z; channel++)
        {
            combined_lightness_here += image[((grid_size.y - 1 - closest_pixel.y) * grid_size.x + closest_pixel.x) * image_size.z + channel];
            cell_count++;
        }
        total_lightness += (averaging_statistic == 0)? combined_lightness_here : combined_lightness_here / image_size.z;
    }
    float ret = 1.0f - (static_cast<float>(total_lightness) / ((averaging_statistic == 0)? static_cast<float>(cell_count) : 1.0) / 255.0f);
    assert(ret >= 0.0f);
    assert(ret <= 1.0f);
    return ret;
};

}; // namespace cura
