/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits
#include <algorithm> // min max

#include <iostream>
#include <cassert>

#include "Material.h"


namespace cura
{

Material::Material()
: data(nullptr)
, width(0)
, height(0)
, depth(0)
{

}


void Material::setData(unsigned char* data)
{
    this->data = data;
}

void Material::setDimensions(unsigned int width, unsigned int height, unsigned int depth)
{
    this->width = width;
    this->height = height;
    this->depth = depth;
}

float Material::getColor(float x, float y, ColourUsage color) const
{
    unsigned int w_idx = std::max(0u, std::min((unsigned int) (x * width), width - 1));
    unsigned int h_idx = std::max(0u, std::min((unsigned int) (y * height), height - 1));
    switch (color)
    {
        case ColourUsage::RED:
        case ColourUsage::GREEN:
        case ColourUsage::BLUE:
        case ColourUsage::ALPHA:
        {
            assert((int)color >= 0 && (unsigned int)color < depth && "Z out of bounds!");
            unsigned char col = getColorData(w_idx, h_idx, (unsigned int) color);
            return (float) col / std::numeric_limits<unsigned char>::max();
        }
        case ColourUsage::GREY:
        default:
        {
            unsigned int r = getColorData(w_idx, h_idx, (unsigned int) ColourUsage::RED);
            unsigned int g = getColorData(w_idx, h_idx, (unsigned int) ColourUsage::GREEN);
            unsigned int b = getColorData(w_idx, h_idx, (unsigned int) ColourUsage::BLUE);
            return (float) (r + g + b) / std::numeric_limits<unsigned char>::max() / 3.0;
        }
    }
}


unsigned char Material::getColorData(unsigned int x, unsigned int y, unsigned int z) const
{
    return data[(y * width + x) * depth + z];
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
            int val = (data[(y*width+x)*depth] * 10 / 256);

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