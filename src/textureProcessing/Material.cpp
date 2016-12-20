/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits
#include <algorithm> // min max

#include <iostream>
#include <cassert>


#include "Material.h"

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
#include "stb/stb_image.h"

namespace cura
{
/*!
 * custom destructor for the data to be used by the shared_pointer
 */
struct ArrayDeleter
{
    void operator ()(unsigned char* p)
    { 
        stbi_image_free(p); 
    }
};

Material::Material()
: data(nullptr, ArrayDeleter())
, width(0)
, height(0)
, depth(0)
{

}

Material::~Material()
{
}


void Material::loadImage(const char* filename)
{
    int w, h, d;
    // in RGBA order
    int desired_channel_count = 0; // keep original amount of channels
    unsigned char* data = stbi_load(filename, &w, &h, &d, desired_channel_count);
    if (data)
    {
        width = w;
        height = h;
        depth = d;
        this->data = std::shared_ptr<unsigned char>(data);
    }
    else
    {
        const char* reason = "[unknown reason]";
        if (stbi_failure_reason())
        {
            reason = stbi_failure_reason();
        }
        logError("Cannot load image %s: '%s'.\n", filename, reason);
        std::exit(-1);
    }
}

float Material::getColor(float x, float y, ColourUsage color) const
{
    assert(x >= 0.0f && x <= 1.0f);
    assert(y >= 0.0f && y <= 1.0f);
    switch (color)
    {
        case ColourUsage::RED:
        case ColourUsage::GREEN:
        case ColourUsage::BLUE:
        case ColourUsage::ALPHA:
        {
            assert((int)color >= 0 && (unsigned int)color < depth && "Z out of bounds!");
            return getColorData(x, y, (unsigned int) color);
        }
        case ColourUsage::GREY:
        default:
        {
            float r = getColorData(x, y, (unsigned int) ColourUsage::RED);
            float g = getColorData(x, y, (unsigned int) ColourUsage::GREEN);
            float b = getColorData(x, y, (unsigned int) ColourUsage::BLUE);
            return (r + g + b) / 3.0;
        }
    }
}


float Material::getColorData(float x, float y, unsigned int z) const
{
    unsigned int x_idx = (unsigned int) (x * (width - 1) + 0.5);
    assert(x_idx >= 0 && x_idx < width && "requested X is out of bounds!");
    unsigned int y_idx = (unsigned int) (y * (height - 1) + 0.5);
    assert(y_idx >= 0 && y_idx < height && "requested Y is out of bounds!");

    unsigned char col = data.get()[((height - y_idx) * width + x_idx) * depth + z];
    return (float) col / std::numeric_limits<unsigned char>::max();
}




void Material::debugOutput(bool dw) const
{
    std::cerr << "\nImage size: " << width << " x " << height << " (" << depth << "channels)\n";
    std::cerr << '+';
    for (unsigned int i = 0; i < width; i++)
    {
        std::cerr << ((dw)? "--" : "-");
    }
    std::cerr << "+\n";
    for (unsigned int y = 0; y < height; y++)
    {
        std::cerr << "|";
        for (unsigned int x = 0; x < width; x++)
        {
            int val = (data.get()[((height - y) * width + x) * depth] * 10 / 256);

            switch (val)
            {
                case 0:
                    std::cerr << ((dw)? "  " : " ");
                    break;
                case 1:
                    std::cerr << ((dw)? ".." : ".");
                    break;
                case 2:
                    std::cerr << ((dw)? ",," : ",");
                    break;
                case 3:
                    std::cerr << ((dw)? "::" : ":");
                    break;
                case 4:
                    std::cerr << ((dw)? ";;" : ";");
                    break;
                case 5:
                    std::cerr << ((dw)? "++" : "+");
                    break;
                case 6:
                    std::cerr << ((dw)? "░░" : "░");
                    break;
                case 7:
                    std::cerr << ((dw)? "▒▒" : "▒");
                    break;
                case 8:
                    std::cerr << ((dw)? "▓▓" : "▓");
                    break;
                default:
                    if (val > 8)
                    {
                        std::cerr << ((dw)? "██" : "█");
                    }
                    else
                    {
                        std::cerr << ((dw)? "  " : " ");
                    }
            }
        }
        std::cerr << "|\n";
    }
    std::cerr << '+';
    for (unsigned int i = 0; i < width; i++)
    {
        std::cerr << ((dw)? "--" : "-");
    }
    std::cerr << "+\n";
}

} // namespace cura